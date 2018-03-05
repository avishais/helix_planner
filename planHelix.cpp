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

#include "planHelix.h"

//#define PI 3.14

bool isStateValidC(const ob::State *state)
{
	return true;
}

void plan_H::plan(Vector c_start, Vector c_goal, double runtime) {

	// construct the state space we are planning in
	ob::CompoundStateSpace *cs = new ob::CompoundStateSpace(); // Compound R^12 configuration space
	ob::StateSpacePtr A(new ob::RealVectorStateSpace(6)); // A-space - state space of the rod - R^6
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(12)); // Angles of Robot 1 & 2 - R^12
	cs->addSubspace(A, 1.0);
	cs->addSubspace(Q, 1.0);

	// set the bounds for the A=R^6
	ob::RealVectorBounds Abounds(6);
	Abounds.setLow(-100);
	Abounds.setHigh(100);

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
	Qbounds.setLow(5, -6.98); // Should be -6.98 but currently the IK won't allow it - this impacts the sampler
	Qbounds.setHigh(5, 6.98); // Should be 6.98 but currently the IK won't allow it
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
	Qbounds.setLow(11, -6.98); // Should be -6.98 but currently the IK won't allow it
	Qbounds.setHigh(11, 6.98); // Should be 6.98 but currently the IK won't allow it

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
	si->setStateValidityCheckingResolution(0.15); // 3% ???

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
	ob::PlannerPtr planner(new og::RRTConnect(si, q1_6_init, q2_6_init));
	//ob::PlannerPtr planner(new ompl::testplanner(si, q1_6_init, q2_6_init));

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

void log_init_6_angles() {
	std::ofstream File;
	File.open("init_6.txt");

	File << q1_6_init << endl;
	File << q2_6_init << endl;

	File.close();
}

void load_A_nodes(Matrix &A) {

	std::ifstream File;
	File.open("/home/avishai/Downloads/omplapp/ompl/Workspace/helixplanner/helices_A.txt");

	Vector a_temp(6);

	int i = 0;
	while(!File.eof()) {
		for (int j=0; j < 6; j++) {
			File >> a_temp[j];
			//cout << a_temp[j] << " ";
		}
		//cout << endl;
		A.push_back(a_temp);
		i++;
	}
	File.close();
	//A.pop_back();
}

Vector setHomePosition(Vector a) {

	Vector c(18), q1(6), q2(6);

	sv.rod_solve(a);
	Matrix Q = sv.getT(sv.get_Points_on_Rod()-1);

	Vector p_s = sv.getP(0);
	Vector p_e = sv.getP(sv.get_Points_on_Rod()-1);
	Vector p_trans = {sv.getW()/2-(p_s[2]-p_e[2])/2, 0, 750};
	double al = -PI/2;
	Matrix T1 = {{ cos(al), 0, sin(al), p_trans[0]},
			{ 0, 1, 0, p_trans[1]},
			{ -sin(al), 0, cos(al), p_trans[2]},
			{  0, 0, 0, 1}};

	// R1
	sv.IKsolve_rob(T1, 1, 1);
	q1 = sv.get_IK_solution_q1();
	sv.FKsolve_rob(q1, 1);
	Matrix T1_fk = sv.get_FK_solution_T1();

	// R2
	Matrix T2 = sv.MatricesMult(T1, Q); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = sv.MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
	sv.IKsolve_rob(T2, 2, 1);
	q2 = sv.get_IK_solution_q2();
	sv.FKsolve_rob(q2, 2);
	Matrix T2_fk = sv.get_FK_solution_T2();

	for (int i = 0; i < 6; i++) {
		c[i] = a[i];
		c[i+6] = q1[i];
		c[i+12] = q2[i];
	}
	/*cout << "q1: "; sv.printVector(q1);
	cout << "T1: "; sv.printMatrix(T1);
	cout << "T1_fk: "; sv.printMatrix(T1_fk);
	cout << "q2: "; sv.printVector(q2);
	cout << "T2: "; sv.printMatrix(T2);
	cout << "T2_fk: "; sv.printMatrix(T2_fk);*/

	return c;
}

double norm(Vector q) {
	double sum = 0;
	for (int i = 0; i < q.size(); i++)
		sum += q[i];
	return sqrt(sum);
}

Vector setGoalPosition(Vector a) {
	Vector c(18), q1(6), q2(6);
	const double pi = 3.14;

	sv.rod_solve(a);
	Matrix Q = sv.getT(sv.get_Points_on_Rod()-1);

	// Set possible pitch angles
	int N = 51;
	Vector pitch;
	for (int i = 0; i < N; i++) {
		double p = -pi + (double)i/(N-1) * 2*pi;
		pitch.push_back( p );
	}
	Vector yaw = pitch;

	// Set possible heights
	int M = 10;
	Vector h;
	for (int i = 0; i < M; i++) {
		double p = 300 + (double)i/(M-1) * 400;
		h.push_back( p );
	}

	// Search for best pose
	Vector q1_min(6), q2_min(6);
	Matrix T1_min, T2_min;
	sv.initMatrix(T1_min,4,4);
	sv.initMatrix(T2_min,4,4);
	double Min = 10000;
	for (int i = 0; i < M; i++) {
		for (int j = 0; j < N; j++) {
			for (int k = 0; k < N; k++) {

				// End position of rod at
				Vector ps = sv.getP(0);
				Vector pe = sv.getP(sv.get_Points_on_Rod()-1);

				ps = { ps[0]*cos(pitch[j])*cos(yaw[k]) - ps[1]*sin(yaw[k]) + ps[2]*cos(yaw[k])*sin(pitch[j]),
						ps[1]*cos(yaw[k]) + ps[0]*cos(pitch[j])*sin(yaw[k]) + ps[2]*sin(pitch[j])*sin(yaw[k]),
						ps[2]*cos(pitch[j]) - ps[0]*sin(pitch[j])};
				pe = { pe[0]*cos(pitch[j])*cos(yaw[k]) - pe[1]*sin(yaw[k]) + pe[2]*cos(yaw[k])*sin(pitch[j]),
						pe[1]*cos(yaw[k]) + pe[0]*cos(pitch[j])*sin(yaw[k]) + pe[2]*sin(pitch[j])*sin(yaw[k]),
						pe[2]*cos(pitch[j]) - pe[0]*sin(pitch[j])};
				Vector pc = {sv.getW()/2-(ps[0]+pe[0])/2,-(ps[1]+pe[1])/2, -(ps[2]+pe[2])/2};
				//sv.printVector(pc);
				pc[2] += h[i];

				Matrix T1 = {{ cos(pitch[j])*cos(yaw[k]), -sin(yaw[k]), cos(yaw[k])*sin(pitch[j]), pc[0]},
						{ cos(pitch[j])*sin(yaw[k]),  cos(yaw[k]), sin(pitch[j])*sin(yaw[k]), pc[1]},
						{ -sin(pitch[j]), 0, cos(pitch[j]), pc[2]},
						{ 0, 0, 0, 1}};

				Matrix T2 = {{   sin(yaw[k])*Q[1][0] - cos(pitch[j])*cos(yaw[k])*Q[0][0] - cos(yaw[k])*sin(pitch[j])*Q[2][0],   sin(yaw[k])*Q[1][1] - cos(pitch[j])*cos(yaw[k])*Q[0][1] - cos(yaw[k])*sin(pitch[j])*Q[2][1], cos(pitch[j])*cos(yaw[k])*Q[0][2] - sin(yaw[k])*Q[1][2] + cos(yaw[k])*sin(pitch[j])*Q[2][2], pc[0] - sin(yaw[k])*Q[1][3] + cos(pitch[j])*cos(yaw[k])*Q[0][3] + cos(yaw[k])*sin(pitch[j])*Q[2][3]},
						{ - cos(yaw[k])*Q[1][0] - cos(pitch[j])*sin(yaw[k])*Q[0][0] - sin(pitch[j])*sin(yaw[k])*Q[2][0], - cos(yaw[k])*Q[1][1] - cos(pitch[j])*sin(yaw[k])*Q[0][1] - sin(pitch[j])*sin(yaw[k])*Q[2][1], cos(yaw[k])*Q[1][2] + cos(pitch[j])*sin(yaw[k])*Q[0][2] + sin(pitch[j])*sin(yaw[k])*Q[2][2], cos(yaw[k])*Q[1][3] + pc[1] + cos(pitch[j])*sin(yaw[k])*Q[0][3] + sin(pitch[j])*sin(yaw[k])*Q[2][3]},
						{                                                 sin(pitch[j])*Q[0][0] - cos(pitch[j])*Q[2][0],                                                 sin(pitch[j])*Q[0][1] - cos(pitch[j])*Q[2][1],                                               cos(pitch[j])*Q[2][2] - sin(pitch[j])*Q[0][2],                                               cos(pitch[j])*Q[2][3] - sin(pitch[j])*Q[0][3] + pc[2]},
						{                                                                                             0,                                                                                             0,                                                                                           0,                                                                                                   1}};

				/*cout << i << " " << j << " " << k << endl;
				cout << pitch[j] << " " << yaw[k] << " " << h[i] << endl;
				sv.printMatrix(T1);
				sv.printMatrix(T2);
				cin.ignore();*/

				// If solved, pick the solution that minimizes angles
				sv.tight_joint_limits();
				if (sv.IKsolve_rob(T1, 1, 1) && sv.IKsolve_rob(T2, 2, 1)) {
					q1 = sv.get_IK_solution_q1();
					q2 = sv.get_IK_solution_q2();
					//if (norm(q1) + norm(q2) < Min) {
					if (T1[0][3]-T2[0][3]+norm(q1) + norm(q2) < Min) {
						Min = norm(q1) + norm(q2);
						q1_min = q1;
						q2_min = q2;
						T1_min = T1;
						T2_min = T2;
					}
				}
			}
		}
	}

	if (Min == 10000) {
		cout << "No goal solution found." << endl;
		exit(1);
	}

	for (int i = 0; i < 6; i++) {
		c[i] = a[i];
		c[i+6] = q1_min[i];
		c[i+12] = q2_min[i];
	}
	/*sv.printVector(q1_min);
	sv.printMatrix(T1_min);
	sv.printVector(q2_min);
	sv.printMatrix(T2_min);*/


	return c;
}

void save_start_goal(Vector cs, Vector cg) {
	Vector a(6), temp(3);
	// Open a_path file
	std::ofstream myfile, afile, pfile;
	myfile.open("robot_paths.txt");
	afile.open("afile.txt");
	pfile.open("rod_path.txt");

	myfile << 2 << endl;
	pfile << 2*501 << endl;

	// Start point
	for (int j = 0; j<6; j++) {
		afile << cs[j] << " ";
		myfile << cs[j+6] << ",";
		a[j] = cs[j];
	}
	for (int j = 12; j<18; j++) {
		myfile << cs[j] << ",";
	}
	myfile << endl;
	afile << endl;

	sv.rod_solve(a);
	// Log points on rod to file
	for (int k = 0; k < sv.get_Points_on_Rod(); k++) {
		temp = sv.getP(k);
		pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
	}
	pfile << endl;

	// Goal point
	for (int j = 0; j<6; j++) {
		afile << cg[j] << " ";
		myfile << cg[j+6] << ",";
		a[j] = cg[j];
	}
	for (int j = 12; j<18; j++) {
		myfile << cg[j] << ",";
	}
	myfile << endl;
	afile << endl;

	sv.rod_solve(a);
	// Log points on rod to file
	for (int k = 0; k < sv.get_Points_on_Rod(); k++) {
		temp = sv.getP(k);
		pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
	}
	pfile << endl;

	myfile.close();
	afile.close();
	pfile.close();

}

void copyfiles(string from, string to) {
	ifstream a;
	ofstream b;
	char ch;
	a.open(from); //The file from which the content will be copied
	b.open(to); //The file to which the content will be copied
	while (!a.eof())
	{
		a.get(ch); //reading from file object 'a'
		b<<ch; //writing to file 'b'
	}
	a.close();
	b.close();
}

int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime;

	if (argn == 1)
		runtime = 1; // sec
	else {
		runtime = atof(args[1]);
	}

	plan_H Plan;

	Vector a(6);

	double t, K, psi0, tau, L;
	L = sv.getL();
	t = 0;
	tau = 0;
	psi0 = 0;
	K = PI/L;

	a[0] = t;
	a[1] = K*cos(psi0);
	a[2] = K*sin(psi0);
	a[3] = tau*(a[0]-tau);
	a[4] = a[1]*(a[0]-tau);
	a[5] = a[2]*(a[0]-tau);

	Vector c_start(18);
	c_start =  setHomePosition(a);

	/*tau = 5.5;
	K = 1 * PI/L;
	a[0] = t;
	a[1] = K*cos(psi0);
	a[2] = K*sin(psi0);
	a[3] = tau*(a[0]-tau);
	a[4] = a[1]*(a[0]-tau);
	a[5] = a[2]*(a[0]-tau);*/

	Matrix A;
	load_A_nodes(A);

	for (int i = 18; i < 19/*A.size()*/; i++) {
		if (i==10 || i==11 || i==14 || i==15 || i==17)
			continue;
		a = A[i];
		sv.printVector(a);

		Vector c_goal = setGoalPosition(a);

		Plan.plan(c_start, c_goal, runtime);
		//save_start_goal(c_start, c_goal);
		//cin.ignore();

		if (Plan.solved_bool) {
			log_init_6_angles();

			system("./smo 15");
			system("./fill");
			system("./twist");

			copyfiles("robot_paths.txt", "./paths/robot_paths_" + std::to_string(i) + ".txt");
			copyfiles("robot_path_twist.txt", "./paths/robot_path_twist_" + std::to_string(i) + ".txt");
			copyfiles("afile.txt", "./paths/afile_" + std::to_string(i) + ".txt");
			copyfiles("rod_path.txt", "./paths/rod_path_" + std::to_string(i) + ".txt");
		}
	}


	std::cout << std::endl << std::endl;

	return 0;
}

