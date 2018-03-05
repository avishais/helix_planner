/*
 * testplanner.h
 *
 *  Created on: Nov 22, 2016
 *      Author: avishai
 */

#ifndef PLANNERS_TESTPLANNER_H_
#define PLANNERS_TESTPLANNER_H_

#include <ompl/base/Planner.h>
// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>

#include "../StateValidityChecker.h"



namespace ompl
{
class testplanner : public base::Planner, public StateValidityChecker
{
public:
	testplanner(const base::SpaceInformationPtr &si, double q1_6, double q2_6) : base::Planner(si, "testplanner"), StateValidityChecker(si, q1_6, q2_6)
	{
		specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
		specs_.directed = true;

		maxDistance_ = 0.0;

		defaultSettings(); // Avishai

		Planner::declareParam<double>("range", this, &testplanner::setRange, &testplanner::getRange, "0.:1.:10000.");
		connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);

	}
	virtual ~testplanner(void)
	{
		// free any allocated memory
	}

	/* motion to be added in the tree of motions. */
	void setRange(double distance)
	{
		maxDistance_ = distance;
	}

	/** \brief Get the range the planner is using */
	double getRange() const
	{
		return maxDistance_;
	}

	void setup()
	{
		Planner::setup();
		tools::SelfConfig sc(si_, getName());
		sc.configurePlannerRange(maxDistance_);
	}



	virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc)
	{
		// make sure the planner is configured correctly; ompl::base::Planner::checkValidity
		// ensures that there is at least one input state and a ompl::base::Goal object specified
		checkValidity();

		const base::State *st1 = pis_.nextStart();

		const base::State *st2 = pis_.nextGoal();

		Vector a1(6), a2(6), q11(6), q21(6), q12(6), q22(6), ik1(2), ik2(2);

		// Start state
		retrieveStateVector(st1, a1, q11, q21);
		base::State *s1 = si_->allocState();
		updateStateVector(s1,q11,q21);
		updateStateVectorA(s1,a1);
		//printStateVector(s1);
		ik1 = identify_state_ik(s1);

		cout << isRodFeasible(a1) << endl;
		printVector(ik1);

		cout << "Closed chain s1? " << close_chain(s1, 0) << endl;
		retrieveStateVector(s1, a1, q11, q21);
		printStateVector(s1);

		// Goal state
		retrieveStateVector(st2, a2, q12, q22);
		base::State *s2 = si_->allocState();
		updateStateVector(s2,q12,q22);
		updateStateVectorA(s2, a2);
		printStateVector(s2);
		ik2 = identify_state_ik(s2);

		cout << isRodFeasible(a2) << endl;
		printVector(ik2);

		cout << "Closed chain s2? " << close_chain(s2, 0) << endl;
		retrieveStateVector(s2, a2, q12, q22);
		printStateVector(s2);

		collision_state(getPMatrix(), q12, q22);

		// Open a_path file
		std::ofstream myfile, afile, pfile, ai;
		myfile.open("robot_paths.txt");
		afile.open("afile.txt");
		pfile.open("rod_path.txt");

		myfile << 2 << endl;
		pfile << 2*501 << endl;

		// Log start point
		retrieveStateVector(s1, a1, q11, q21);
		for (int j = 0; j<6; j++) {
			myfile << q11[j] << ",";
			afile << a1[j] << " ";
		}
		for (int j = 0; j<6; j++) {
			myfile << q21[j] << ",";
		}
		myfile << endl;
		afile << endl;

		Vector temp;
		rod_solve(a1);
		// Log points on rod to file
		for (int k = 0; k < get_Points_on_Rod(); k++) {
			temp = getP(k);
			pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
		}
		pfile << endl;

		// Log goal point
		retrieveStateVector(s2, a2, q12, q22);
		for (int j = 0; j<6; j++) {
			myfile << q12[j] << ",";
			afile << a2[j] << " ";
		}
		for (int j = 0; j<6; j++) {
			myfile << q22[j] << ",";
		}
		myfile << endl;
		afile << endl;

		rod_solve(a2);
		// Log points on rod to file
		for (int k = 0; k < get_Points_on_Rod(); k++) {
			temp = getP(k);
			pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
		}
		pfile << endl;

		myfile.close();
		pfile.close();


		return base::PlannerStatus::EXACT_SOLUTION;
	}

	Vector rad2deg(Vector q) {
		Vector qd(6);
		for (int i = 0; i < q.size(); i++)
			qd[i] = q[i] * 180/3.14;
		return qd;
	}

	virtual void clear(void)
	{
		Planner::clear();
		// clear the data structures here
	}

	virtual void getPlannerData(base::PlannerData &data) const
	{
		// fill data with the states and edges that were created
		// in the exploration data structure
		// perhaps also fill control::PlannerData
	}

	base::ValidStateSamplerPtr        sampler_;

	/** \brief The maximum length of a motion to be added to a tree */
	double                        maxDistance_;

	/** \brief The random number generator */
	RNG                           rng_;

	/** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
	std::pair<base::State*, base::State*>      connectionPoint_;
};
}

#endif /* PLANNERS_TESTPLANNER_H_ */
