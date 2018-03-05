/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

//#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include "myRRTConnect.h" // Avishai

// Debugging tool
template <class T>
void o(T a) {
	cout << a << endl;
}

ompl::geometric::RRTConnect::RRTConnect(const base::SpaceInformationPtr &si, double q1_6, double q2_6) : base::Planner(si, "RRTConnect"), StateValidityChecker(si, q1_6, q2_6)
{
	specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
	specs_.directed = true;

	maxDistance_ = 0.0;

	Planner::declareParam<double>("range", this, &RRTConnect::setRange, &RRTConnect::getRange, "0.:1.:10000.");
	connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);

	defaultSettings(); // Avishai

	Range = 2; // As tested

}

ompl::geometric::RRTConnect::~RRTConnect()
{
	freeMemory();
}

void ompl::geometric::RRTConnect::setup()
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configurePlannerRange(maxDistance_);

	if (!tStart_)
		tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	if (!tGoal_)
		tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	tStart_->setDistanceFunction(std::bind(&RRTConnect::activeDistance, this, std::placeholders::_1, std::placeholders::_2)); //activeDistance
	tGoal_->setDistanceFunction(std::bind(&RRTConnect::activeDistance, this, std::placeholders::_1, std::placeholders::_2)); //distanceFunction
}

void ompl::geometric::RRTConnect::freeMemory()
{
	std::vector<Motion*> motions;

	if (tStart_)
	{
		tStart_->list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		{
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}

	if (tGoal_)
	{
		tGoal_->list(motions);
		for (unsigned int i = 0 ; i < motions.size() ; ++i)
		{
			if (motions[i]->state)
				si_->freeState(motions[i]->state);
			delete motions[i];
		}
	}
}

void ompl::geometric::RRTConnect::clear()
{
	Planner::clear();
	sampler_.reset();
	freeMemory();
	if (tStart_)
		tStart_->clear();
	if (tGoal_)
		tGoal_->clear();
	connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);
}

double ompl::geometric::RRTConnect::activeDistance(const Motion *a, const Motion *b) {

	Vector aa(6), qa(6), qa_dummy(6);
	Vector ab(6), qb(6), qb_dummy(6);

	if (!active_chain) {
		retrieveStateVector(a->state, aa, qa, qa_dummy);
		retrieveStateVector(b->state, ab, qb, qb_dummy);
	}
	else {
		retrieveStateVector(a->state, aa, qa_dummy, qa);
		retrieveStateVector(b->state, ab, qb_dummy, qb);
	}

	double sum = 0;
	for (int i=0; i < aa.size(); i++)
		sum += pow(aa[i]-ab[i], 2);
	for (int i=0; i < qa.size(); i++)
		sum += pow(qa[i]-qb[i], 2);
	return sqrt(sum);
}

ompl::geometric::RRTConnect::Motion* ompl::geometric::RRTConnect::growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *nmotion, Motion *tmotion, int mode, int count_iterations)
// tmotion - target
// nmotion - nearest
// mode = 1 -> extend, mode = 2 -> connect.
{
	Vector a(6), q1(6), q2(6), ik(2);

	// Choose active chain
	active_chain = rand() % 2; // 0 - (q1,a) is the active chain, 1 - (q2,a) is the active chain

	bool reach = false;
	growTree_reached = false;

	//int count_iterations = 500;
	while (count_iterations) {
		count_iterations--;

		// find state to add
		base::State *dstate = tmotion->state;
		double d = activeDistance(nmotion, tmotion);

		if (d > maxDistance_)
		{
			si_->getStateSpace()->interpolate(nmotion->state, tmotion->state, maxDistance_ / d, tgi.xstate);
			dstate = tgi.xstate;
			reach = false;
		}
		else
			reach = true;

		if (mode==1 || !reach) {
			// Project
			// Check that 'a' on the random state is feasible
			retrieveStateVector(dstate, a, q1, q2);
			if (!isRodFeasible(a))
				return nmotion;
			Matrix T = getT(get_Points_on_Rod()-1);

			// Project dstate (currently not on the manifold)
			if (!active_chain) {
				if (!calc_specific_IK_solution_R1(T, q1, nmotion->ik_q1_active)) {
					if (!calc_specific_IK_solution_R2(T, q2, nmotion->ik_q2_active))
						return nmotion;
					active_chain = !active_chain;
					q1 = get_IK_solution_q1();
				}
				else
					q2 = get_IK_solution_q2();
			}
			else {
				if (!calc_specific_IK_solution_R2(T, q2, nmotion->ik_q2_active)) {
					if (!calc_specific_IK_solution_R1(T, q1, nmotion->ik_q1_active))
						return nmotion;
					active_chain = !active_chain;
					q2 = get_IK_solution_q2();
				}
				else
					q1 = get_IK_solution_q1();
			}
			if (collision_state(getPMatrix(), q1, q2))
				return nmotion;

			updateStateVector(tgi.xstate, q1, q2);
			updateStateVectorA(tgi.xstate, a);
			dstate = tgi.xstate;

			ik = identify_state_ik(dstate, T);
		}
		else { // Added but not tested
			ik = identify_state_ik(dstate);
			if (nmotion->ik_q1_active==ik[0])
				active_chain = 0;
			else if (nmotion->ik_q2_active==ik[1])
				active_chain = 1;
			else
				continue;
		}

		//if (!check_neighbors(dstate ,nmotion->state, active_chain))
			//return nmotion;

		// Check motion
		//bool validMotion = checkMotion(nmotion->state, dstate, active_chain, (!active_chain ? nmotion->ik_q1_active : nmotion->ik_q2_active)); // Avishai
		bool validMotion = checkMotionSerial(nmotion->state, dstate, active_chain, (!active_chain ? nmotion->ik_q1_active : nmotion->ik_q2_active)); // Avishai

		if (validMotion)
		{
			/* Update advanced motion */
			Motion *motion = new Motion(si_);
			motion->ik_q1_active = ik[0];
			motion->ik_q2_active = ik[1];
			si_->copyState(motion->state, dstate);
			motion->parent = nmotion;
			motion->root = nmotion->root;
			motion->a_chain = active_chain;
			tgi.xmotion = motion;
			tree->add(motion);

			nmotion = motion;

			//cout << "Trees distance: " << trees_distance << ", ik_s: " << ik_start[0] << " " << ik_start[1] << ", ik_g: " << ik_goal[0] << " " << ik_goal[1] << ", ik added: " << ik[0] << " " << ik[1] << endl;

			if (reach) {
				cout << nmotion->ik_q1_active << " " << nmotion->ik_q2_active << endl;
				cout << ik[0] << " " << ik[0] << endl;
				cout << mode << endl;

				growTree_reached = true;
				return nmotion;
			}
		}
		else
			return nmotion;
	}
	return nmotion;
}

ompl::base::PlannerStatus ompl::geometric::RRTConnect::solve(const base::PlannerTerminationCondition &ptc)
{
	initiate_log_parameters();
	setRange(Range);
	base::State *start_node = si_->allocState();

	Vector a(6), q1(6), q2(6), ik(2);
	ik_start.resize(2);
	ik_goal.resize(2);

	checkValidity();

	base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

	if (!goal)
	{
		OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
		return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
	}

	while (const base::State *st = pis_.nextStart())
	{
		if (0) {//!close_chain(st, 1)) { // In the profiling, the nodes are already valid //
			OMPL_ERROR("%s: Start state not feasible!", getName().c_str());
			return base::PlannerStatus::INVALID_START;
		}
		retrieveStateVector(st, a, q1, q2);
		ik = identify_state_ik(st);
		Motion *motion = new Motion(si_);
		si_->copyState(motion->state, st);
		motion->root = motion->state;
		motion->ik_q1_active = ik[0];
		motion->ik_q2_active = ik[1];
		motion->a_chain = 0;
		tStart_->add(motion);

		si_->copyState(start_node,st);

		o("Real start state: ");
		printStateVector(st);
		printVector(ik);
		ik_start = ik;
	}

	// ------------------ Test --------------------------------

	/*q2 = {0,0,0,0,0,0};
	FKsolve_rob(q2, 2);
	printMatrix(get_FK_solution_T2());


	return base::PlannerStatus::INVALID_START;*/

	// ------------------ Test --------------------------------

	if (tStart_->size() == 0)
	{
		OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
		return base::PlannerStatus::INVALID_START;
	}

	if (!goal->couldSample())
	{
		OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
		return base::PlannerStatus::INVALID_GOAL;
	}

	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(), (int)(tStart_->size() + tGoal_->size()));

	TreeGrowingInfo tgi;
	tgi.xstate = si_->allocState();

	Motion   *rmotion   = new Motion(si_);
	base::State *rstate = rmotion->state;
	bool startTree      = true;
	bool solved         = false;
	startTime = clock();
	while (ptc == false)
	{
		TreeData &tree      = startTree ? tStart_ : tGoal_;
		tgi.start = startTree;
		startTree = !startTree;
		TreeData &otherTree = startTree ? tStart_ : tGoal_;

		if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
		{
			const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
			if (st)
			{
				if (0){//!close_chain(st, 1)) { // In the profiling, the nodes are already valid //!close_chain(st, 0)
					OMPL_ERROR("%s: Goal state not feasible!", getName().c_str());
					return base::PlannerStatus::INVALID_GOAL;
				}
				retrieveStateVector(st, a, q1, q2);
				ik = identify_state_ik(st);
				Motion *motion = new Motion(si_);
				si_->copyState(motion->state, st);
				motion->root = motion->state;
				motion->ik_q1_active = ik[0];
				motion->ik_q2_active = ik[1];
				motion->a_chain = 0;
				tGoal_->add(motion);
				PlanDistance = si_->distance(start_node, st);

				o("Real goal state: ");
				printStateVector(st);
				printVector(ik);

				ik_goal = ik;
			}

			if (tGoal_->size() == 0)
			{
				OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
				break;
			}
		}

		trees_distance = distanceBetweenTrees(tree, otherTree);
		cout << "Current trees distance: " << distanceBetweenTrees(tree, otherTree) << endl;

		//===================== New ==========================

		Motion* reached_motion;
		// sample random state
		sampler_->sampleUniform(rstate);

		Motion *nmotion = tree->nearest(rmotion); // NN over the active distance
		//int sizeb = tree->size();
		reached_motion = growTree(tree, tgi, nmotion, rmotion, 1, 100);
		//if (tree->size()==sizeb)
		//continue;

		// remember which motion was just added
		Motion *addedMotion = reached_motion;

		tgi.xmotion = nullptr;
		nmotion = otherTree->nearest(reached_motion); // NN over the active distance
		reached_motion = growTree(otherTree, tgi, nmotion, reached_motion, 2, 500);

		Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
		Motion *goalMotion  = startTree ? addedMotion : tgi.xmotion;

		// if we connected the trees in a valid way (start and goal pair is valid)
		if (growTree_reached) {
			cout << "Connection point active chain is " << ((!a_chain_connection) ? "q1." : "q2.") << endl;

			// Report computation time
			endTime = clock();
			total_runtime = double(endTime - startTime) / CLOCKS_PER_SEC;
			cout << "Solved in " << total_runtime << "s." << endl;

			// it must be the case that either the start tree or the goal tree has made some progress
			// so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
			// on the solution path

			cout << addedMotion << " " << tgi.xmotion << endl;

			if (startMotion->parent)
				startMotion = startMotion->parent;
			else
				goalMotion = goalMotion->parent;

			connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

			// construct the solution path
			Motion *solution = startMotion;
			std::vector<Motion*> mpath1;
			while (solution != nullptr)
			{
				mpath1.push_back(solution);
				solution = solution->parent;
			}

			solution = goalMotion;
			std::vector<Motion*> mpath2;
			while (solution != nullptr)
			{
				mpath2.push_back(solution);
				solution = solution->parent;
			}

			cout << "Path from tree 1 size: " << mpath1.size() << ", path from tree 2 size: " << mpath2.size() << endl;
			nodes_in_path = mpath1.size() + mpath2.size();
			nodes_in_trees = tStart_->size() + tGoal_->size();

			PathGeometric *path = new PathGeometric(si_);
			path->getStates().reserve(mpath1.size() + mpath2.size());
			for (int i = mpath1.size() - 1 ; i >= 0 ; --i)
				path->append(mpath1[i]->state);
			for (unsigned int i = 0 ; i < mpath2.size() ; ++i)
				path->append(mpath2[i]->state);
			save2file(mpath1, mpath2);
			pdef_->addSolutionPath(base::PathPtr(path), false, 0.0, getName());
			solved = true;
			break;
		}

		//====================================================
	}
	if (!solved)
	{
		// Report computation time
		endTime = clock();
		total_runtime = double(endTime - startTime) / CLOCKS_PER_SEC;

		nodes_in_trees = tStart_->size() + tGoal_->size();
	}

	si_->freeState(tgi.xstate);
	si_->freeState(rstate);
	delete rmotion;

	OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(), tStart_->size(), tGoal_->size());

	final_solved = solved;
	LogPerf2file();

	return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}


double ompl::geometric::RRTConnect::distanceBetweenTrees(TreeData &tree1, TreeData &tree2) {

	std::vector<Motion*> motions;
	tree1->list(motions);

	Motion *nmotion;
	double minD = 1e10, curD;
	for (unsigned int i = 0 ; i < motions.size() ; ++i)
	{
		nmotion = tree2->nearest(motions[i]);
		curD = distanceFunction(nmotion, motions[i]);
		if (curD < minD) {
			minD = curD;
		}
	}
	return minD;
}

void ompl::geometric::RRTConnect::getPlannerData(base::PlannerData &data) const
{
	Planner::getPlannerData(data);

	std::vector<Motion*> motions;
	if (tStart_)
		tStart_->list(motions);

	for (unsigned int i = 0 ; i < motions.size() ; ++i)
	{
		if (motions[i]->parent == nullptr)
			data.addStartVertex(base::PlannerDataVertex(motions[i]->state, 1));
		else
		{
			data.addEdge(base::PlannerDataVertex(motions[i]->parent->state, 1),
					base::PlannerDataVertex(motions[i]->state, 1));
		}
	}

	motions.clear();
	if (tGoal_)
		tGoal_->list(motions);

	for (unsigned int i = 0 ; i < motions.size() ; ++i)
	{
		if (motions[i]->parent == nullptr)
			data.addGoalVertex(base::PlannerDataVertex(motions[i]->state, 2));
		else
		{
			// The edges in the goal tree are reversed to be consistent with start tree
			data.addEdge(base::PlannerDataVertex(motions[i]->state, 2),
					base::PlannerDataVertex(motions[i]->parent->state, 2));
		}
	}

	// Add the edge connecting the two trees
	data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));
}

void ompl::geometric::RRTConnect::save2file(vector<Motion*> mpath1, vector<Motion*> mpath2) {

	cout << "Logging path to files..." << endl;
	//return;

	Vector a(6), q1(6), q2(6);
	int active_chain, ik_sol;

	{
		// Open a_path file
		std::ofstream myfile, afile, pfile, ai;
		myfile.open("robot_paths.txt");
		afile.open("afile.txt");
		pfile.open("rod_path.txt");
		ai.open("active_ik.txt");

		myfile << mpath1.size() + mpath2.size() << endl;
		pfile << (mpath1.size() + mpath2.size())*501 << endl;

		Vector temp;
		for (int i = mpath1.size() - 1 ; i >= 0 ; --i) {
			retrieveStateVector(mpath1[i]->state, a, q1, q2);
			//cout << mpath1[i]->a_chain << " " << mpath1[i]->ik_q1_active << " " << mpath1[i]->ik_q2_active << endl;
			for (int j = 0; j<6; j++) {
				myfile << q1[j] << ",";
				afile << a[j] << " ";
			}
			for (int j = 0; j<6; j++) {
				myfile << q2[j] << ",";
			}
			myfile << endl;
			afile << endl;

			rod_solve(a);
			// Log points on rod to file
			for (int k = 0; k < get_Points_on_Rod(); k++) {
				temp = getP(k);
				pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
			}
			pfile << endl;

			ai << mpath1[i]->a_chain << " " << mpath1[i]->ik_q1_active << " " << mpath1[i]->ik_q2_active << endl;
		}
		for (unsigned int i = 0 ; i < mpath2.size() ; ++i) {
			retrieveStateVector(mpath2[i]->state, a, q1, q2);
			//cout << mpath2[i]->a_chain << " " << mpath2[i]->ik_q1_active << " " << mpath2[i]->ik_q2_active << endl;
			for (int j = 0; j<6; j++) {
				myfile << q1[j] << ",";
				afile << a[j] << " ";
			}
			for (int j = 0; j<6; j++) {
				myfile << q2[j] << ",";
			}
			myfile << endl;
			afile << endl;

			rod_solve(a);
			// Log points on rod to file
			for (int k = 0; k < get_Points_on_Rod(); k++) {
				temp = getP(k);
				pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
			}
			pfile << endl;

			ai << mpath2[i]->a_chain << " " << mpath2[i]->ik_q1_active << " " << mpath2[i]->ik_q2_active << endl;
		}
		myfile.close();
		afile.close();
		pfile.close();
		ai.close();
	}
}


void ompl::geometric::RRTConnect::LogPerf2file() {

	std::ofstream myfile;
	myfile.open("perf_log.txt"); // <====== Change it before profiling.

	myfile << final_solved << endl;
	myfile << PlanDistance << endl; // Distance between nodes 1
	myfile << total_runtime << endl; // Overall planning runtime 2
	myfile << get_odes_counter() << endl; // How many ode's checks? 3
	myfile << get_valid_odes_counter() << endl; // How many ode's checks and returned valid? 4
	myfile << get_odes_time() << endl; // ODE computation time 4
	myfile << get_IK_counter() << endl; // How many IK checks? 5
	myfile << get_IK_time() << endl; // IK computation time 6
	myfile << get_collisionCheck_counter() << endl; // How many collision checks? 7
	myfile << get_collisionCheck_time() << endl; // Collision check computation time 8
	myfile << get_isValid_counter() << endl; // How many nodes checked 9
	myfile << nodes_in_path << endl; // Nodes in path 10
	myfile << nodes_in_trees << endl; // 11

	myfile.close();
}
