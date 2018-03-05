/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Avishai Sintov, Ioan Sucan */

#include "plan_C_space.h"
#include <time.h>       /* time */

bool isStateValidC(const ob::State *state)
{
	return true;
}

void plan_C::plan(Vector c_start, Vector c_goal, double runtime) {

	// construct the state space we are planning in
	ob::CompoundStateSpace *cs = new ob::CompoundStateSpace(); // Compound R^12 configuration space
	ob::StateSpacePtr A(new ob::RealVectorStateSpace(6)); // A-space - state space of the rod - R^6
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(12)); // Angles of Robot 1 & 2 - R^12
	cs->addSubspace(A, 1.0);
	cs->addSubspace(Q, 1.0);

	// set the bounds for the A=R^6
	ob::RealVectorBounds Abounds(6);
	Abounds.setLow(-31);
	Abounds.setHigh(30);

	// set the bounds for the Q=R^12 part of 'Cspace'
	ob::RealVectorBounds Qbounds(12);
	Qbounds.setLow(0, -2.88); // Robot 1
	Qbounds.setHigh(0, 2.88);
	Qbounds.setLow(1, -1.919);
	Qbounds.setHigh(1, 1.919);
	Qbounds.setLow(2, -1.919);
	Qbounds.setHigh(2, 1.22);
	Qbounds.setLow(3, -2.79);
	Qbounds.setHigh(3, 2.79);
	Qbounds.setLow(4, -2.09);
	Qbounds.setHigh(4, 2.09);
	Qbounds.setLow(5, -PI);// -6.98); // Should be -6.98 but currently the IK won't allow it - this impacts the sampler
	Qbounds.setHigh(5, PI);// 6.98); // Should be 6.98 but currently the IK won't allow it
	Qbounds.setLow(6, -2.88); // Robot 2
	Qbounds.setHigh(6, 2.88);
	Qbounds.setLow(7, -1.919);
	Qbounds.setHigh(7, 1.919);
	Qbounds.setLow(8, -1.919);
	Qbounds.setHigh(8, 1.22);
	Qbounds.setLow(9, -2.79);
	Qbounds.setHigh(9, 2.79);
	Qbounds.setLow(10, -2.09);
	Qbounds.setHigh(10, 2.09);
	Qbounds.setLow(11, -PI);// -6.98); // Should be -6.98 but currently the IK won't allow it
	Qbounds.setHigh(11, PI);// 6.98); // Should be 6.98 but currently the IK won't allow it

	// set the bound for the compound space
	cs->as<ob::RealVectorStateSpace>(0)->setBounds(Abounds);
	cs->as<ob::RealVectorStateSpace>(1)->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Cspace(cs);

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Cspace));

	//si->setValidStateSamplerAllocator(allocMyValidStateSampler);

	// set state validity checking for this space
	//si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));
	si->setStateValidityChecker(std::bind(&isStateValidC, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.005); // 3% ???

	// create start state
	ob::ScopedState<ob::CompoundStateSpace> start(Cspace);
	for (int i = 0; i < 6; i++) {
		start->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = c_start[i]; // Access the first component of the start a-state
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_start[i+6];
		start->as<ob::RealVectorStateSpace::StateType>(1)->values[i+6] = c_start[i+12];//q2[i];
	}

	// create goal state
	ob::ScopedState<ob::CompoundStateSpace> goal(Cspace);
	for (int i = 0; i < 6; i++) {
		goal->as<ob::RealVectorStateSpace::StateType>(0)->values[i] = c_goal[i]; // Access the first component of the goal a-state
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[i] = c_goal[i+6];
		goal->as<ob::RealVectorStateSpace::StateType>(1)->values[i+6] = c_goal[i+12];//q2[i];
	}

	// create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	pdef->print();

	// create a planner for the defined space
	// To add a planner, the #include library must be added above
	ob::PlannerPtr planner(new og::RRTConnect(si));
	//ob::PlannerPtr planner(new og::RRT(si));
	//ob::PlannerPtr planner(new og::BiTRRT(si));
	//ob::	PlannerPtr planner(new og::LazyRRT(si));
	//ob::PlannerPtr planner(new og::pSBL(si));
	//ob::PlannerPtr planner(new og::PRM(si));
	//ob::PlannerPtr planner(new og::LazyPRM(si));
	//ob::PlannerPtr planner(new ompl::decoupled_rod(si));
	//ob::PlannerPtr planner(new ompl::testplanner(si));

	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

	// perform setup steps for the planner
	planner->setup();

	//planner->printSettings(std::cout); // Prints some parameters such as range
	//planner->printProperties(std::cout); // Prints some decisions such as multithreading, display approx solutions, and optimize?

	// print the settings for this space
	//si->printSettings(std::cout); // Prints state space settings such as check resolution, segmant count factor and bounds
	//si->printProperties(std::cout); // Prints state space properties, average length, dimension ...

	// print the problem settings
	//pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	// attempt to solve the problem within one second of planning time
	clock_t begin = clock();
	ob::PlannerStatus solved = planner->solve(runtime);
	clock_t end = clock();
	total_runtime = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Runtime: " << total_runtime << endl;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		/*ob::PathPtr path = pdef->getSolutionPath();
		std::cout << "Found solution:" << std::endl;

		// print the path to screen
		//path->print(std::cout);  // Print as vectors

		// Save path to file
		std::ofstream myfile;
		myfile.open("pathRRTC.txt");
		og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		pog.printAsMatrix(myfile); // Print as matrix to file
		myfile.close();*/
		solved_bool = true;
	}
	else {
		std::cout << "No solutions found" << std::endl;
		solved_bool = false;
	}
}



void load_random_nodes(Matrix &Cdb) {

	std::ifstream File;
	File.open("/home/avishai/Downloads/omplapp/ompl/Workspace/cplanner/generate_random_nodes/randomnodes.txt");
	//File.open("randomnodes.txt");

	Vector c_temp(18);

	int i = 0;
	while(!File.eof()) {
		for (int j=0; j < 18; j++) {
			File >> c_temp[j];
			//cout << c_temp[j] << " ";
		}
		//cout << endl;
		Cdb.push_back(c_temp);
		i++;
	}
	File.close();
	Cdb.pop_back();

}

void load_query_points(Matrix &C) {
	Vector c_temp(18);

	ifstream fq;
	fq.open("query.txt");
	int i = 0;
	while(!fq.eof()) {
		for (int j=0; j < 18; j++) {
			fq >> c_temp[j];
		}
		C.push_back(c_temp);
		i++;
	}
	fq.close();
}


double distance(Vector a, Vector b) {
	double sum = 0;
	for (int i = 0; i < a.size(); i++)
		sum += pow(a[i]-b[i],2);
	return sqrt(sum);
}

void extract_from_perf_file(ofstream &ToFile) {
	ifstream FromFile;
	FromFile.open("perf_log.txt");

	string line;
	while (getline(FromFile, line))
		ToFile << line << "\t";

	FromFile.close();
}


int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime;

	if (argn == 1)
		runtime = 1; // sec
	else {
		runtime = atof(args[1]);
	}

	plan_C Plan;


	Matrix Cdb;
	load_random_nodes(Cdb);

	int mode = 1;
	switch (mode) {
	case 1 : {
		srand (time(NULL));

		std::ofstream ft;
		ft.open("timesC.txt", ios::app);

		int p1, p2;
		for (int j = 0; j < 1; j++) {
			do {
				p1 = rand() % Cdb.size();
				p2 = rand() % Cdb.size();

			} while (p1==p2);

			cout << p1 << " " << p2 << endl;

			Plan.plan(Cdb[p1], Cdb[p2], runtime);

			// Log
			ft << p1 << "\t" << p2 << "\t";
			extract_from_perf_file(ft);
			ft << endl;
		}
		ft.close();
		break;
	}
	case 2 : {
		//Vector c_start = {2.17, -4.2, 2, 1.9, 13.6, -2.4, 0.0873, 0.1745,  0, 0, 0.1745, 0, 1.0529, 0.4354, 0.5597, 1.1289, -1.7280, 0.3042};
		//Vector c_start = {-1.5, -5, 0, -30, 2, 10, 0, -1.047, 0.8727, 0, -0.3491, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		//Vector c_goal = {0, -4, 0, -25, 0, 30, 1.5707, 0.8726, -0.1745, -1.5707, 1.0472, 0.5,0, 0, 0, 0, 0, 0};

		//Vector c_start = {2.17, -4.2, 2, 1.9, 13.6, -2.4, 0.0873, 0.1745,  0, 0, 0.1745, 0, 1.06008, 0.433869, 0.559712, -1.99757, 1.72538, -2.84337 };
		//Vector c_goal = {0, -4, 0, -25, 0, 30, 1.5707, 0.8726, -0.1745, -1.5707, 1.0472, 0.5, -0.62553, 0.788661, 0.668315, 2.57829, 1.71216, -1.24766};
		//Vector c_start = {0.5911,   -1.2856,   -5.8199 ,  -0.7626 ,   0.3748  ,  0.0214, -0.390328,0.612012,-0.281784,-2.10627,-1.01849,-2.05768,-0.00197797,0.888097,-0.686747,-0.664729,-0.661837,-1.75615 };
		//Vector c_goal = {-0.5938  , -2.3700 ,  -5.0451 ,  -0.2548  ,  0.8103 ,  -0.4187, 0.0444899,0.441007,-0.275984,-2.51569,-0.948493,-1.57105,0.0492653,0.884583,-0.607802,-0.627877,-0.618528,-1.85671};


		Vector c_start = Cdb[23];
		Vector c_goal = Cdb[26];
		Plan.plan(c_start, c_goal, runtime);
		break;
	}
	case 3 : {
		Matrix C;
		load_query_points(C);
		Plan.plan(C[0], C[1], runtime);
		break;
	}
	}

	std::cout << std::endl << std::endl;

	return 0;
}


