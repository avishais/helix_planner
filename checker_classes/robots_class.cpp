#include "robots_class.h"

// Constructor for the robots
two_robots::two_robots(Vector pose_1, Vector pose_2, double q1_6, double q2_6) {
	b = 166; // Height of base
	l1 = 124; // Distance from top facet of base to axis of link 1.
	l2 = 270; // Length between axes of link 2.
	l3a = 70; // Shortest distance from top axis of link 2 to axis of link 3. (possibly 69.96mm).
	l3b = 149.6; // Horizontal distance(in home position) from top axis of link 2 mutual face of link 3 with link 4.
	l4 = 152.4; // Distance from mutual face of link 3 and link 4 to axis of links 4 - 5.
	l5 = 72;//-13/2; // Distance from axis of link 5 to its facet.  - Updated measure 4/13/2017
	lee = 95;//86;//13/2+60; // Measurement of new printed EE // Distance from facet of link 5 to EE point  - Updated measure 4/13/2017

	// Joint limits
	q1minmax = deg2rad(165);
	q2minmax = deg2rad(110);
	q3min = deg2rad(-110); 
	q3max = deg2rad(70);
	q4minmax = deg2rad(160);
	q5minmax = deg2rad(120);
	q6minmax = deg2rad(180);//400);

	V_pose_rob_1_o = pose_1;
	V_pose_rob_2_o = pose_2;

	// Initial joint 6 angles of both robots
	q1_6_init = q1_6;
	q2_6_init = q2_6;

	initMatrix1(T_fk_solution_1, 4, 4);
	initVector1(p_fk_solution_1, 3);
	initMatrix1(T_fk_solution_2, 4, 4);
	initVector1(p_fk_solution_2, 3);
	initMatrix1(T_fk_temp, 4, 4);
	initVector1(p_fk_temp, 3);
	initVector1(V_pose, 3);
	initMatrix1(T1,4, 4);
	initMatrix1(T2,4, 4);
	initMatrix1(T_mult_temp,4, 4);
	initMatrix1(Q_IK_solutions_1, NUM_IK_SOLUTIONS, 6);
	initMatrix1(Q_IK_solutions_2, NUM_IK_SOLUTIONS, 6);
	initVector1(valid_IK_solutions_indices_1, NUM_IK_SOLUTIONS);
	initVector1(valid_IK_solutions_indices_2, NUM_IK_SOLUTIONS);

	msg = false;

	IK_counter = 0;
	IK_time = 0;

	cout << "Robots initialized." << endl;
}

// -----FK-------

void two_robots::FKsolve_rob(Vector q, int robot_num) {

	if (robot_num == 1)
		V_pose = V_pose_rob_1_o;
	else
		V_pose = V_pose_rob_2_o;

	// Move with relative position to rod
	if (robot_num==1)
		q[5] += q1_6_init;
	else
		q[5] += q2_6_init;

	Matrix R02 = {{cos(V_pose[5]), -sin(V_pose[5]), 0},{sin(V_pose[5]), cos(V_pose[5]), 0},{0, 0, 1}};

	// q - joint angles
	p_fk_temp = { (sin(V_pose[3])*sin(V_pose[5]) + cos(V_pose[3])*cos(V_pose[5])*sin(V_pose[4]))*(b + l1 + l3a*cos(q[1] + q[2]) - l4*sin(q[1] + q[2]) - l3b*sin(q[1] + q[2]) + l2*cos(q[1]) + (l5*cos(q[1] + q[2])*sin(q[3] - q[4]))/2 + (lee*cos(q[1] + q[2])*sin(q[3] - q[4]))/2 - l5*cos(q[4])*sin(q[1] + q[2]) - lee*cos(q[4])*sin(q[1] + q[2]) - (l5*cos(q[1] + q[2])*sin(q[3] + q[4]))/2 - (lee*cos(q[1] + q[2])*sin(q[3] + q[4]))/2) - (cos(V_pose[3])*sin(V_pose[5]) - cos(V_pose[5])*sin(V_pose[3])*sin(V_pose[4]))*(l2*sin(q[0])*sin(q[1]) + l4*sin(q[0])*cos(q[1] + q[2]) + l3b*sin(q[0])*cos(q[1] + q[2]) + l3a*sin(q[0])*sin(q[1] + q[2]) + l5*cos(q[0])*sin(q[3])*sin(q[4]) + lee*cos(q[0])*sin(q[3])*sin(q[4]) + l5*cos(q[4])*sin(q[0])*cos(q[1] + q[2]) + lee*cos(q[4])*sin(q[0])*cos(q[1] + q[2]) - l5*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - l5*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4]) - lee*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - lee*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4])) + V_pose[0] + cos(V_pose[4])*cos(V_pose[5])*(l2*cos(q[0])*sin(q[1]) + l4*cos(q[0])*cos(q[1] + q[2]) + l3b*cos(q[0])*cos(q[1] + q[2]) + l3a*cos(q[0])*sin(q[1] + q[2]) + l5*cos(q[0])*cos(q[4])*cos(q[1] + q[2]) - l5*sin(q[0])*sin(q[3])*sin(q[4]) + lee*cos(q[0])*cos(q[4])*cos(q[1] + q[2]) - lee*sin(q[0])*sin(q[3])*sin(q[4]) - l5*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4]) - l5*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]) - lee*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4]) - lee*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4])),
		 (cos(V_pose[3])*cos(V_pose[5]) + sin(V_pose[3])*sin(V_pose[4])*sin(V_pose[5]))*(l2*sin(q[0])*sin(q[1]) + l4*sin(q[0])*cos(q[1] + q[2]) + l3b*sin(q[0])*cos(q[1] + q[2]) + l3a*sin(q[0])*sin(q[1] + q[2]) + l5*cos(q[0])*sin(q[3])*sin(q[4]) + lee*cos(q[0])*sin(q[3])*sin(q[4]) + l5*cos(q[4])*sin(q[0])*cos(q[1] + q[2]) + lee*cos(q[4])*sin(q[0])*cos(q[1] + q[2]) - l5*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - l5*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4]) - lee*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - lee*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4])) - (cos(V_pose[5])*sin(V_pose[3]) - cos(V_pose[3])*sin(V_pose[4])*sin(V_pose[5]))*(b + l1 + l3a*cos(q[1] + q[2]) - l4*sin(q[1] + q[2]) - l3b*sin(q[1] + q[2]) + l2*cos(q[1]) + (l5*cos(q[1] + q[2])*sin(q[3] - q[4]))/2 + (lee*cos(q[1] + q[2])*sin(q[3] - q[4]))/2 - l5*cos(q[4])*sin(q[1] + q[2]) - lee*cos(q[4])*sin(q[1] + q[2]) - (l5*cos(q[1] + q[2])*sin(q[3] + q[4]))/2 - (lee*cos(q[1] + q[2])*sin(q[3] + q[4]))/2) + V_pose[1] + cos(V_pose[4])*sin(V_pose[5])*(l2*cos(q[0])*sin(q[1]) + l4*cos(q[0])*cos(q[1] + q[2]) + l3b*cos(q[0])*cos(q[1] + q[2]) + l3a*cos(q[0])*sin(q[1] + q[2]) + l5*cos(q[0])*cos(q[4])*cos(q[1] + q[2]) - l5*sin(q[0])*sin(q[3])*sin(q[4]) + lee*cos(q[0])*cos(q[4])*cos(q[1] + q[2]) - lee*sin(q[0])*sin(q[3])*sin(q[4]) - l5*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4]) - l5*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]) - lee*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4]) - lee*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4])),
		                                                                                                                  V_pose[2] - sin(V_pose[4])*(l2*cos(q[0])*sin(q[1]) + l4*cos(q[0])*cos(q[1] + q[2]) + l3b*cos(q[0])*cos(q[1] + q[2]) + l3a*cos(q[0])*sin(q[1] + q[2]) + l5*cos(q[0])*cos(q[4])*cos(q[1] + q[2]) - l5*sin(q[0])*sin(q[3])*sin(q[4]) + lee*cos(q[0])*cos(q[4])*cos(q[1] + q[2]) - lee*sin(q[0])*sin(q[3])*sin(q[4]) - l5*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4]) - l5*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]) - lee*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4]) - lee*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4])) + cos(V_pose[4])*sin(V_pose[3])*(l2*sin(q[0])*sin(q[1]) + l4*sin(q[0])*cos(q[1] + q[2]) + l3b*sin(q[0])*cos(q[1] + q[2]) + l3a*sin(q[0])*sin(q[1] + q[2]) + l5*cos(q[0])*sin(q[3])*sin(q[4]) + lee*cos(q[0])*sin(q[3])*sin(q[4]) + l5*cos(q[4])*sin(q[0])*cos(q[1] + q[2]) + lee*cos(q[4])*sin(q[0])*cos(q[1] + q[2]) - l5*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - l5*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4]) - lee*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - lee*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4])) + cos(V_pose[3])*cos(V_pose[4])*(b + l1 + l3a*cos(q[1] + q[2]) - l4*sin(q[1] + q[2]) - l3b*sin(q[1] + q[2]) + l2*cos(q[1]) + (l5*cos(q[1] + q[2])*sin(q[3] - q[4]))/2 + (lee*cos(q[1] + q[2])*sin(q[3] - q[4]))/2 - l5*cos(q[4])*sin(q[1] + q[2]) - lee*cos(q[4])*sin(q[1] + q[2]) - (l5*cos(q[1] + q[2])*sin(q[3] + q[4]))/2 - (lee*cos(q[1] + q[2])*sin(q[3] + q[4]))/2)};

	T_fk_temp[0] = { (sin(q[4])*(cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) - cos(q[0])*sin(q[3])) - cos(q[4])*(cos(q[1])*cos(q[2])*sin(q[0]) - sin(q[0])*sin(q[1])*sin(q[2])))*(cos(V_pose[3])*sin(V_pose[5]) - cos(V_pose[5])*sin(V_pose[3])*sin(V_pose[4])) - (sin(V_pose[3])*sin(V_pose[5]) + cos(V_pose[3])*cos(V_pose[5])*sin(V_pose[4]))*(cos(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) + cos(q[3])*sin(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))) - cos(V_pose[4])*cos(V_pose[5])*(cos(q[4])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + sin(q[4])*(cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) + sin(q[0])*sin(q[3]))), cos(V_pose[4])*cos(V_pose[5])*(cos(q[5])*(sin(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) - cos(q[3])*sin(q[0])) - sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[4])*(cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) + sin(q[0])*sin(q[3])))) - (sin(V_pose[3])*sin(V_pose[5]) + cos(V_pose[3])*cos(V_pose[5])*sin(V_pose[4]))*(sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) - cos(q[3])*cos(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))) - (sin(q[5])*(sin(q[4])*(cos(q[1])*cos(q[2])*sin(q[0]) - sin(q[0])*sin(q[1])*sin(q[2])) + cos(q[4])*(cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) - cos(q[0])*sin(q[3]))) + cos(q[5])*(sin(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) + cos(q[0])*cos(q[3])))*(cos(V_pose[3])*sin(V_pose[5]) - cos(V_pose[5])*sin(V_pose[3])*sin(V_pose[4])), (sin(q[5])*(sin(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) + cos(q[0])*cos(q[3])) - cos(q[5])*(sin(q[4])*(cos(q[1])*cos(q[2])*sin(q[0]) - sin(q[0])*sin(q[1])*sin(q[2])) + cos(q[4])*(cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) - cos(q[0])*sin(q[3]))))*(cos(V_pose[3])*sin(V_pose[5]) - cos(V_pose[5])*sin(V_pose[3])*sin(V_pose[4])) - (sin(V_pose[3])*sin(V_pose[5]) + cos(V_pose[3])*cos(V_pose[5])*sin(V_pose[4]))*(cos(q[5])*(sin(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) - cos(q[3])*cos(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))) + sin(q[3])*sin(q[5])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))) - cos(V_pose[4])*cos(V_pose[5])*(cos(q[5])*(sin(q[4])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[4])*(cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) + sin(q[0])*sin(q[3]))) + sin(q[5])*(sin(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) - cos(q[3])*sin(q[0]))), (sin(V_pose[3])*sin(V_pose[5]) + cos(V_pose[3])*cos(V_pose[5])*sin(V_pose[4]))*(b + l1 + l3a*cos(q[1] + q[2]) - l4*sin(q[1] + q[2]) - l3b*sin(q[1] + q[2]) + l2*cos(q[1]) + (l5*cos(q[1] + q[2])*sin(q[3] - q[4]))/2 + (lee*cos(q[1] + q[2])*sin(q[3] - q[4]))/2 - l5*cos(q[4])*sin(q[1] + q[2]) - lee*cos(q[4])*sin(q[1] + q[2]) - (l5*cos(q[1] + q[2])*sin(q[3] + q[4]))/2 - (lee*cos(q[1] + q[2])*sin(q[3] + q[4]))/2) - (cos(V_pose[3])*sin(V_pose[5]) - cos(V_pose[5])*sin(V_pose[3])*sin(V_pose[4]))*(l2*sin(q[0])*sin(q[1]) + l4*sin(q[0])*cos(q[1] + q[2]) + l3b*sin(q[0])*cos(q[1] + q[2]) + l3a*sin(q[0])*sin(q[1] + q[2]) + l5*cos(q[0])*sin(q[3])*sin(q[4]) + lee*cos(q[0])*sin(q[3])*sin(q[4]) + l5*cos(q[4])*sin(q[0])*cos(q[1] + q[2]) + lee*cos(q[4])*sin(q[0])*cos(q[1] + q[2]) - l5*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - l5*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4]) - lee*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - lee*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4])) + V_pose[0] + cos(V_pose[4])*cos(V_pose[5])*(l2*cos(q[0])*sin(q[1]) + l4*cos(q[0])*cos(q[1] + q[2]) + l3b*cos(q[0])*cos(q[1] + q[2]) + l3a*cos(q[0])*sin(q[1] + q[2]) + l5*cos(q[0])*cos(q[4])*cos(q[1] + q[2]) - l5*sin(q[0])*sin(q[3])*sin(q[4]) + lee*cos(q[0])*cos(q[4])*cos(q[1] + q[2]) - lee*sin(q[0])*sin(q[3])*sin(q[4]) - l5*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4]) - l5*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]) - lee*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4]) - lee*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]))};
	T_fk_temp[1] = { (cos(V_pose[5])*sin(V_pose[3]) - cos(V_pose[3])*sin(V_pose[4])*sin(V_pose[5]))*(cos(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) + cos(q[3])*sin(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))) - (sin(q[4])*(cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) - cos(q[0])*sin(q[3])) - cos(q[4])*(cos(q[1])*cos(q[2])*sin(q[0]) - sin(q[0])*sin(q[1])*sin(q[2])))*(cos(V_pose[3])*cos(V_pose[5]) + sin(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])) - cos(V_pose[4])*sin(V_pose[5])*(cos(q[4])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + sin(q[4])*(cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) + sin(q[0])*sin(q[3]))), (sin(q[5])*(sin(q[4])*(cos(q[1])*cos(q[2])*sin(q[0]) - sin(q[0])*sin(q[1])*sin(q[2])) + cos(q[4])*(cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) - cos(q[0])*sin(q[3]))) + cos(q[5])*(sin(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) + cos(q[0])*cos(q[3])))*(cos(V_pose[3])*cos(V_pose[5]) + sin(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])) + (cos(V_pose[5])*sin(V_pose[3]) - cos(V_pose[3])*sin(V_pose[4])*sin(V_pose[5]))*(sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) - cos(q[3])*cos(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))) + cos(V_pose[4])*sin(V_pose[5])*(cos(q[5])*(sin(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) - cos(q[3])*sin(q[0])) - sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[4])*(cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) + sin(q[0])*sin(q[3])))), (cos(V_pose[5])*sin(V_pose[3]) - cos(V_pose[3])*sin(V_pose[4])*sin(V_pose[5]))*(cos(q[5])*(sin(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) - cos(q[3])*cos(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))) + sin(q[3])*sin(q[5])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))) - (sin(q[5])*(sin(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) + cos(q[0])*cos(q[3])) - cos(q[5])*(sin(q[4])*(cos(q[1])*cos(q[2])*sin(q[0]) - sin(q[0])*sin(q[1])*sin(q[2])) + cos(q[4])*(cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) - cos(q[0])*sin(q[3]))))*(cos(V_pose[3])*cos(V_pose[5]) + sin(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])) - cos(V_pose[4])*sin(V_pose[5])*(cos(q[5])*(sin(q[4])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[4])*(cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) + sin(q[0])*sin(q[3]))) + sin(q[5])*(sin(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) - cos(q[3])*sin(q[0]))), (cos(V_pose[3])*cos(V_pose[5]) + sin(V_pose[3])*sin(V_pose[4])*sin(V_pose[5]))*(l2*sin(q[0])*sin(q[1]) + l4*sin(q[0])*cos(q[1] + q[2]) + l3b*sin(q[0])*cos(q[1] + q[2]) + l3a*sin(q[0])*sin(q[1] + q[2]) + l5*cos(q[0])*sin(q[3])*sin(q[4]) + lee*cos(q[0])*sin(q[3])*sin(q[4]) + l5*cos(q[4])*sin(q[0])*cos(q[1] + q[2]) + lee*cos(q[4])*sin(q[0])*cos(q[1] + q[2]) - l5*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - l5*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4]) - lee*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - lee*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4])) - (cos(V_pose[5])*sin(V_pose[3]) - cos(V_pose[3])*sin(V_pose[4])*sin(V_pose[5]))*(b + l1 + l3a*cos(q[1] + q[2]) - l4*sin(q[1] + q[2]) - l3b*sin(q[1] + q[2]) + l2*cos(q[1]) + (l5*cos(q[1] + q[2])*sin(q[3] - q[4]))/2 + (lee*cos(q[1] + q[2])*sin(q[3] - q[4]))/2 - l5*cos(q[4])*sin(q[1] + q[2]) - lee*cos(q[4])*sin(q[1] + q[2]) - (l5*cos(q[1] + q[2])*sin(q[3] + q[4]))/2 - (lee*cos(q[1] + q[2])*sin(q[3] + q[4]))/2) + V_pose[1] + cos(V_pose[4])*sin(V_pose[5])*(l2*cos(q[0])*sin(q[1]) + l4*cos(q[0])*cos(q[1] + q[2]) + l3b*cos(q[0])*cos(q[1] + q[2]) + l3a*cos(q[0])*sin(q[1] + q[2]) + l5*cos(q[0])*cos(q[4])*cos(q[1] + q[2]) - l5*sin(q[0])*sin(q[3])*sin(q[4]) + lee*cos(q[0])*cos(q[4])*cos(q[1] + q[2]) - lee*sin(q[0])*sin(q[3])*sin(q[4]) - l5*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4]) - l5*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]) - lee*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4]) - lee*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]))};
	T_fk_temp[2] = { sin(V_pose[4])*(cos(q[4])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + sin(q[4])*(cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) + sin(q[0])*sin(q[3]))) - cos(V_pose[4])*sin(V_pose[3])*(sin(q[4])*(cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) - cos(q[0])*sin(q[3])) - cos(q[4])*(cos(q[1])*cos(q[2])*sin(q[0]) - sin(q[0])*sin(q[1])*sin(q[2]))) - cos(V_pose[3])*cos(V_pose[4])*(cos(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) + cos(q[3])*sin(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))),                                                                                                                  cos(V_pose[4])*sin(V_pose[3])*(sin(q[5])*(sin(q[4])*(cos(q[1])*cos(q[2])*sin(q[0]) - sin(q[0])*sin(q[1])*sin(q[2])) + cos(q[4])*(cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) - cos(q[0])*sin(q[3]))) + cos(q[5])*(sin(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) + cos(q[0])*cos(q[3]))) - sin(V_pose[4])*(cos(q[5])*(sin(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) - cos(q[3])*sin(q[0])) - sin(q[5])*(sin(q[4])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[4])*(cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) + sin(q[0])*sin(q[3])))) - cos(V_pose[3])*cos(V_pose[4])*(sin(q[5])*(sin(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) - cos(q[3])*cos(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))) - cos(q[5])*sin(q[3])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))),                                                                                                                  sin(V_pose[4])*(cos(q[5])*(sin(q[4])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - cos(q[4])*(cos(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) + sin(q[0])*sin(q[3]))) + sin(q[5])*(sin(q[3])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) - cos(q[3])*sin(q[0]))) - cos(V_pose[4])*sin(V_pose[3])*(sin(q[5])*(sin(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) + cos(q[0])*cos(q[3])) - cos(q[5])*(sin(q[4])*(cos(q[1])*cos(q[2])*sin(q[0]) - sin(q[0])*sin(q[1])*sin(q[2])) + cos(q[4])*(cos(q[3])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) - cos(q[0])*sin(q[3])))) - cos(V_pose[3])*cos(V_pose[4])*(cos(q[5])*(sin(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) - cos(q[3])*cos(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))) + sin(q[3])*sin(q[5])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]))),                                                                                                                  V_pose[2] - sin(V_pose[4])*(l2*cos(q[0])*sin(q[1]) + l4*cos(q[0])*cos(q[1] + q[2]) + l3b*cos(q[0])*cos(q[1] + q[2]) + l3a*cos(q[0])*sin(q[1] + q[2]) + l5*cos(q[0])*cos(q[4])*cos(q[1] + q[2]) - l5*sin(q[0])*sin(q[3])*sin(q[4]) + lee*cos(q[0])*cos(q[4])*cos(q[1] + q[2]) - lee*sin(q[0])*sin(q[3])*sin(q[4]) - l5*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4]) - l5*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4]) - lee*cos(q[0])*cos(q[1])*cos(q[3])*sin(q[2])*sin(q[4]) - lee*cos(q[0])*cos(q[2])*cos(q[3])*sin(q[1])*sin(q[4])) + cos(V_pose[4])*sin(V_pose[3])*(l2*sin(q[0])*sin(q[1]) + l4*sin(q[0])*cos(q[1] + q[2]) + l3b*sin(q[0])*cos(q[1] + q[2]) + l3a*sin(q[0])*sin(q[1] + q[2]) + l5*cos(q[0])*sin(q[3])*sin(q[4]) + lee*cos(q[0])*sin(q[3])*sin(q[4]) + l5*cos(q[4])*sin(q[0])*cos(q[1] + q[2]) + lee*cos(q[4])*sin(q[0])*cos(q[1] + q[2]) - l5*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - l5*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4]) - lee*cos(q[1])*cos(q[3])*sin(q[0])*sin(q[2])*sin(q[4]) - lee*cos(q[2])*cos(q[3])*sin(q[0])*sin(q[1])*sin(q[4])) + cos(V_pose[3])*cos(V_pose[4])*(b + l1 + l3a*cos(q[1] + q[2]) - l4*sin(q[1] + q[2]) - l3b*sin(q[1] + q[2]) + l2*cos(q[1]) + (l5*cos(q[1] + q[2])*sin(q[3] - q[4]))/2 + (lee*cos(q[1] + q[2])*sin(q[3] - q[4]))/2 - l5*cos(q[4])*sin(q[1] + q[2]) - lee*cos(q[4])*sin(q[1] + q[2]) - (l5*cos(q[1] + q[2])*sin(q[3] + q[4]))/2 - (lee*cos(q[1] + q[2])*sin(q[3] + q[4]))/2)};
	T_fk_temp[3] = { 0, 0, 0, 1 };

	if (robot_num == 1) {
		p_fk_solution_1 = p_fk_temp;
		T_fk_solution_1 = T_fk_temp;
	}
	else {
		p_fk_solution_2 = p_fk_temp;
		T_fk_solution_2 = T_fk_temp;
	}
}

Matrix two_robots::get_FK_solution_T1() {

	//printMatrix(T_fk_solution_1);

	return T_fk_solution_1;
}

Vector two_robots::get_FK_solution_p1() {

	//printVector(p_fk_solution_1);

	return p_fk_solution_1;
}

Matrix two_robots::get_FK_solution_T2() {

	//printMatrix(T_fk_solution_2);

	return T_fk_solution_2;
}

Vector two_robots::get_FK_solution_p2() {

	//printVector(p_fk_solution_2);

	return p_fk_solution_2;
}

int two_robots::get_countSolutions() {
	return countSolutions;
}

// -------IK----------

bool two_robots::IKsolve_rob(Matrix T, int robot_num, int solution_num) {
	Vector q(6);

	IK_counter++;
	clock_t begin = clock();

	double q1_add_sol[NUM_IK_SOLUTIONS] = { 0, 0, 0, 0, PI, PI, PI, PI, -PI, -PI, -PI, -PI};
	double q2_sign[NUM_IK_SOLUTIONS] = { -1, -1, -1, -1,  1,  1,  1,  1,   1,   1,   1,   1};
	double q3_sign[NUM_IK_SOLUTIONS] = { 1, -1,  1, -1,  1, -1,  1, -1,   1,  -1,   1,  -1};
	double sign456[NUM_IK_SOLUTIONS] = { 1, -1, -1,  1, -1,  1,  1, -1,  -1,   1,   1,  -1 };
	Vector V_pose(3);

	if (robot_num == 1)
		V_pose = V_pose_rob_1_o;
	else
		V_pose = V_pose_rob_2_o;

	Matrix R = {{cos(V_pose[4])*cos(V_pose[5])*T[0][0] - sin(V_pose[4])*T[2][0] + cos(V_pose[4])*sin(V_pose[5])*T[1][0],  cos(V_pose[4])*cos(V_pose[5])*T[0][1] - sin(V_pose[4])*T[2][1] + cos(V_pose[4])*sin(V_pose[5])*T[1][1], cos(V_pose[4])*cos(V_pose[5])*T[0][2] - sin(V_pose[4])*T[2][2] + cos(V_pose[4])*sin(V_pose[5])*T[1][2]},
			{ cos(V_pose[3])*cos(V_pose[5])*T[1][0] - cos(V_pose[3])*sin(V_pose[5])*T[0][0] + cos(V_pose[4])*sin(V_pose[3])*T[2][0] + cos(V_pose[5])*sin(V_pose[3])*sin(V_pose[4])*T[0][0] + sin(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])*T[1][0], cos(V_pose[3])*cos(V_pose[5])*T[1][1] - cos(V_pose[3])*sin(V_pose[5])*T[0][1] + cos(V_pose[4])*sin(V_pose[3])*T[2][1] + cos(V_pose[5])*sin(V_pose[3])*sin(V_pose[4])*T[0][1] + sin(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])*T[1][1], cos(V_pose[3])*cos(V_pose[5])*T[1][2] - cos(V_pose[3])*sin(V_pose[5])*T[0][2] + cos(V_pose[4])*sin(V_pose[3])*T[2][2] + cos(V_pose[5])*sin(V_pose[3])*sin(V_pose[4])*T[0][2] + sin(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])*T[1][2]},
			{ cos(V_pose[3])*cos(V_pose[4])*T[2][0] - cos(V_pose[5])*sin(V_pose[3])*T[1][0] + sin(V_pose[3])*sin(V_pose[5])*T[0][0] + cos(V_pose[3])*cos(V_pose[5])*sin(V_pose[4])*T[0][0] + cos(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])*T[1][0], cos(V_pose[3])*cos(V_pose[4])*T[2][1] - cos(V_pose[5])*sin(V_pose[3])*T[1][1] + sin(V_pose[3])*sin(V_pose[5])*T[0][1] + cos(V_pose[3])*cos(V_pose[5])*sin(V_pose[4])*T[0][1] + cos(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])*T[1][1], cos(V_pose[3])*cos(V_pose[4])*T[2][2] - cos(V_pose[5])*sin(V_pose[3])*T[1][2] + sin(V_pose[3])*sin(V_pose[5])*T[0][2] + cos(V_pose[3])*cos(V_pose[5])*sin(V_pose[4])*T[0][2] + cos(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])*T[1][2]}};

	Vector p = {sin(V_pose[4])*V_pose[2] - sin(V_pose[4])*T[2][3] - cos(V_pose[4])*cos(V_pose[5])*V_pose[0] - cos(V_pose[4])*sin(V_pose[5])*V_pose[1] + cos(V_pose[4])*cos(V_pose[5])*T[0][3] + cos(V_pose[4])*sin(V_pose[5])*T[1][3],
			cos(V_pose[3])*sin(V_pose[5])*V_pose[0] - cos(V_pose[3])*cos(V_pose[5])*V_pose[1] - cos(V_pose[4])*sin(V_pose[3])*V_pose[2] + cos(V_pose[3])*cos(V_pose[5])*T[1][3] - cos(V_pose[3])*sin(V_pose[5])*T[0][3] + cos(V_pose[4])*sin(V_pose[3])*T[2][3] - cos(V_pose[5])*sin(V_pose[3])*sin(V_pose[4])*V_pose[0] - sin(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])*V_pose[1] + cos(V_pose[5])*sin(V_pose[3])*sin(V_pose[4])*T[0][3] + sin(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])*T[1][3],
			cos(V_pose[5])*sin(V_pose[3])*V_pose[1] - cos(V_pose[3])*cos(V_pose[4])*V_pose[2] - sin(V_pose[3])*sin(V_pose[5])*V_pose[0] + cos(V_pose[3])*cos(V_pose[4])*T[2][3] - cos(V_pose[5])*sin(V_pose[3])*T[1][3] + sin(V_pose[3])*sin(V_pose[5])*T[0][3] - cos(V_pose[3])*cos(V_pose[5])*sin(V_pose[4])*V_pose[0] - cos(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])*V_pose[1] + cos(V_pose[3])*cos(V_pose[5])*sin(V_pose[4])*T[0][3] + cos(V_pose[3])*sin(V_pose[4])*sin(V_pose[5])*T[1][3]};

	Vector x6 = { R[0][0], R[1][0], R[2][0] };
	Vector p5 = { p[0] - x6[0] * (l5 + lee), p[1] - x6[1] * (l5 + lee), p[2] - x6[2] * (l5 + lee) };
	
	//q1
	{
		q[0] = atan2(p5[1], p5[0]) + q1_add_sol[solution_num];
		if (fabs(q[0]) > q1minmax) {
			//cout << "q[0]: " << q[0] << endl;
			clock_t end = clock();
			IK_time += double(end - begin) / CLOCKS_PER_SEC;
			return false;
		}
	}

	double k2 = p5[0] * p5[0] + p5[1] * p5[1]; // = k^2
	double D2 = k2 + (p5[2] - (l1 + b)) * (p5[2] - (l1 + b)); // = D ^ 2
	double l34 = sqrt((l3a*l3a + (l3b + l4)*(l3b + l4))); // Distance from joint 23 axis to joint 45 axis.
	
	//q3
	double sinphi;
	{
		double alpha = atan2(l3b + l4, l3a); // Angle in the base of the triangle of link 3.
		double cosphi = (D2 - l2*l2 - l34*l34) / (2 * l2*l34);
		sinphi = (1 - cosphi*cosphi);
		if (sinphi < 0) {
			//cout << "sinphi" << endl;
			clock_t end = clock();
			IK_time += double(end - begin) / CLOCKS_PER_SEC;
			return false;
		}
		sinphi = q3_sign[solution_num]*sqrt(sinphi);
		//q[2] = -(atan2(sinphi, cosphi) - PI + alpha);
		q[2] = -(atan2(sinphi, cosphi)+alpha);
		if (q[2] < q3min || q[2] > q3max) {
			//cout << "q[2]: " << q[2]*180/PI << endl;
			clock_t end = clock();
			IK_time += double(end - begin) / CLOCKS_PER_SEC;
			return false;
		}
	}

	//q2
	{
		double sinalpha1 = -l34*(sinphi) / sqrt(D2);
		double alpha1 = atan2(-q2_sign[solution_num]*sinalpha1, sqrt(1 - sinalpha1*sinalpha1));
		double alpha2 = atan2(p5[2] - l1 - b, sqrt(k2));
		q[1] = q2_sign[solution_num] * (alpha1 + alpha2 - PI / 2);
		if (fabs(q[1]) > q2minmax) {
			//cout << "q[1]: " << q[1]*180/PI << endl;
			clock_t end = clock();
			IK_time += double(end - begin) / CLOCKS_PER_SEC;
			return false;
		}
	}

	// Solved according to Spong's "Robot dynamics and control" page 91,96.
	double c23 = cos(q[1] + q[2]); 
	double s23 = sin(q[1] + q[2]);
	double c1 = cos(q[0]); 
	double s1 = sin(q[0]);

	//q5
	{
		double S = R[0][0]*c23*c1 - R[2][0]*s23 + R[1][0]*c23*s1;
		q[4] = atan2(sign456[solution_num]*sqrt(1 - S*S), S);
		q[4] = fabs(q[4]) < 1e-4 ? 0 : q[4];
		if (fabs(q[4]) > q5minmax) {
			clock_t end = clock();
			IK_time += double(end - begin) / CLOCKS_PER_SEC;
			//cout << "q[4]: " << q[4]*180/PI << endl;
			return false;
		}
	}

	// q4 and a6
	if (q[4]) {
		//q4
		double sinq4 = R[1][0]*c1 - R[0][0]*s1;
		double cosq4 = R[2][0]*c23 + R[0][0]*s23*c1 + R[1][0]*s23*s1;
		q[3] = atan2(sign456[solution_num]*sinq4, -sign456[solution_num]*cosq4);

		//q6
		double sinq6 = R[0][1]*c23*c1 - R[2][1]*s23 + R[1][1]*c23*s1;
		double cosq6 = R[0][2]*c23*c1 - R[2][2]*s23 + R[1][2]*c23*s1;
		q[5] = atan2(sign456[solution_num]*sinq6, sign456[solution_num]*cosq6);
		if (robot_num==1)
			q[5] -= q1_6_init;
		else
			q[5] -= q2_6_init;
	}
	else {
		//q4
		q[3] = 0;
		//q6
		q[5] = atan2(R[2][1], R[2][2]); // or q6 = atan2(R(3, 2), R(2, 2)) or q6 = atan2(-R(2, 3), R(3, 3));
	}
	if (fabs(q[3]) > q4minmax || fabs(q[5]) > q6minmax){
		clock_t end = clock();
		IK_time += double(end - begin) / CLOCKS_PER_SEC;
		//cout << "q[3] & q[5]: " << q[3]*180/PI << " " << q[5]*180/PI << endl;
		return false;
	}

	if (robot_num == 1)
		q_IK_solution_1 = q;
	else
		q_IK_solution_2 = q;

	clock_t end = clock();
	IK_time += double(end - begin) / CLOCKS_PER_SEC;

	return true;
}

// Computes all the IK solutions to 'Q_IK_solutions_1' matrix.
// Returns the number of existing solutions
int two_robots::calc_all_IK_solutions_1(Matrix T) {

	int countSolutions = 0;
	for (int i = 0; i < NUM_IK_SOLUTIONS; i++) {
		if (IKsolve_rob(T, 1, i)) {
			Q_IK_solutions_1[countSolutions] = q_IK_solution_1;
			valid_IK_solutions_indices_1[countSolutions] = i;
			countSolutions++;
		}
	}


	return countSolutions;
}

// Computes all the IK solutions to 'Q_IK_solutions_2' matrix.
// Returns the number of existing solutions
int two_robots::calc_all_IK_solutions_2(Matrix T) {

	clock_t begin = clock();

	int countSolutions = 0;
	for (int i = 0; i < NUM_IK_SOLUTIONS; i++) {
		if (IKsolve_rob(T, 2, i)) {
			Q_IK_solutions_2[countSolutions] = q_IK_solution_2;
			valid_IK_solutions_indices_2[countSolutions] = i;
			countSolutions++;
		}
	}

	return countSolutions;
}

Vector two_robots::get_all_IK_solutions_1(int num_sol) {
	return Q_IK_solutions_1[num_sol];
}

Vector two_robots::get_all_IK_solutions_2(int num_sol) {
	return Q_IK_solutions_2[num_sol];
}

Vector two_robots::get_IK_solution_q1() {
	return q_IK_solution_1;
}

Vector two_robots::get_IK_solution_q2() {
	return q_IK_solution_2;
}

int two_robots::get_valid_IK_solutions_indices_1(int i) {
	return valid_IK_solutions_indices_1[i];
}
int two_robots::get_valid_IK_solutions_indices_2(int i) {
	return valid_IK_solutions_indices_2[i];
}

//------------------------

double two_robots::normDistance(Vector q_a, Vector q_b) {

	double max = 0, cur;
	for (int i=0; i<q_a.size(); i++) {
		cur = fabs(q_a[i]-q_b[i]);
		if (cur > max)
			max = cur;
	}
	return max;

	/*double sum = 0;
	for (int i=0; i<q_a.size(); i++)
		sum += pow(q_a[i]-q_b[i],2);
	return sum;*/
}

// Compute a specific IK solution when q1 is the active chain
// Checks that IK exists for Robot 2
bool two_robots::calc_specific_IK_solution_R1(Matrix T, Vector q1, int IKsol) {
	// T - Trans. matrix for the end of the rod while its start is in the origin
	// q - angles of robot 1.

	FKsolve_rob(q1, 1);
	T2 = MatricesMult(get_FK_solution_T1(), T); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2

	if (IKsolve_rob(T2, 2, IKsol))
		return true;
	else
		return false;
}

// Compute a specific IK solution when q2 is the active chain
// Checks that IK exists for Robot 1
bool two_robots::calc_specific_IK_solution_R2(Matrix T, Vector q2, int IKsol) {
	// T - Trans. matrix for the end of the rod while its start is in the origin
	// q - angles of robot 1.

	Matrix Tinv = T;
	InvertMatrix(T, Tinv); // Invert matrix

	FKsolve_rob(q2, 2);
	T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob

	if (IKsolve_rob(T1, 1, IKsol))
		return true;
	else
		return false;
}

// Check the feasibility of the state with no other information
// Checks that IK exists for Robot 2
bool two_robots::IsRobotsFeasible(Matrix T, Vector q) {
	// T - Trans. matrix for the end of the rod while its start is in the origin
	// q - angles of robot 1.

	if (msg) printVector(q);

	FKsolve_rob(q, 1);
	T2 = MatricesMult(get_FK_solution_T1(), T); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2

	countSolutions = calc_all_IK_solutions_2(T2);

	if (countSolutions >= 1) {
		if (msg) cout << "Successfully solved IK." << endl;
		return true;
	}
	else {
		if (msg) cout << "IK solution not found." << endl;
		return false;
	}
}

// Check the feasibility of the state with no other information
// Checks that IK exists for Robot 1
bool two_robots::IsRobotsFeasible_R2(Matrix T, Vector q) {
	// T - Trans. matrix for the end of the rod while its start is in the origin
	// q - angles of robot 2.

	if (msg) printVector(q);

	FKsolve_rob(q, 2);

	T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = MatricesMult(T1, T); // Returns the REQUIRED matrix of the rods tip at rob

	countSolutions = calc_all_IK_solutions_1(T1);
	if (countSolutions >= 1) {
		if (msg) cout << "Successfully solved IK." << endl;
		return true;
	}
	else {
		if (msg) cout << "IK solution not found." << endl;
		return false;
	}
	return false;
}

Matrix two_robots::MatricesMult(Matrix M1, Matrix M2) {
	clearMatrix(T_mult_temp);
	for(unsigned i = 0; i < 4; ++i)
		for(unsigned j = 0; j < 4; ++j)
			for(int k = 0; k < 4; ++k)
			{
				T_mult_temp[i][j] += M1[i][k] * M2[k][j];
			}
	return T_mult_temp;
}

bool two_robots::InvertMatrix(Matrix M, Matrix &Minv) {
	Vector m(16),  inv(16);

	int k = 0;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++, k++)
			m[k] = M[j][i];

    double det;
    int i;

    inv[0] = m[5]  * m[10] * m[15] -
             m[5]  * m[11] * m[14] -
             m[9]  * m[6]  * m[15] +
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] -
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
              m[4]  * m[11] * m[14] +
              m[8]  * m[6]  * m[15] -
              m[8]  * m[7]  * m[14] -
              m[12] * m[6]  * m[11] +
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
             m[4]  * m[11] * m[13] -
             m[8]  * m[5] * m[15] +
             m[8]  * m[7] * m[13] +
             m[12] * m[5] * m[11] -
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] -
               m[8]  * m[6] * m[13] -
               m[12] * m[5] * m[10] +
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
              m[1]  * m[11] * m[14] +
              m[9]  * m[2] * m[15] -
              m[9]  * m[3] * m[14] -
              m[13] * m[2] * m[11] +
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
             m[0]  * m[11] * m[14] -
             m[8]  * m[2] * m[15] +
             m[8]  * m[3] * m[14] +
             m[12] * m[2] * m[11] -
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
              m[0]  * m[11] * m[13] +
              m[8]  * m[1] * m[15] -
              m[8]  * m[3] * m[13] -
              m[12] * m[1] * m[11] +
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
              m[0]  * m[10] * m[13] -
              m[8]  * m[1] * m[14] +
              m[8]  * m[2] * m[13] +
              m[12] * m[1] * m[10] -
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
             m[1]  * m[7] * m[14] -
             m[5]  * m[2] * m[15] +
             m[5]  * m[3] * m[14] +
             m[13] * m[2] * m[7] -
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
              m[0]  * m[7] * m[14] +
              m[4]  * m[2] * m[15] -
              m[4]  * m[3] * m[14] -
              m[12] * m[2] * m[7] +
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
              m[0]  * m[7] * m[13] -
              m[4]  * m[1] * m[15] +
              m[4]  * m[3] * m[13] +
              m[12] * m[1] * m[7] -
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
               m[0]  * m[6] * m[13] +
               m[4]  * m[1] * m[14] -
               m[4]  * m[2] * m[13] -
               m[12] * m[1] * m[6] +
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
              m[1] * m[7] * m[10] +
              m[5] * m[2] * m[11] -
              m[5] * m[3] * m[10] -
              m[9] * m[2] * m[7] +
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
             m[0] * m[7] * m[10] -
             m[4] * m[2] * m[11] +
             m[4] * m[3] * m[10] +
             m[8] * m[2] * m[7] -
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
               m[0] * m[7] * m[9] +
               m[4] * m[1] * m[11] -
               m[4] * m[3] * m[9] -
               m[8] * m[1] * m[7] +
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
              m[0] * m[6] * m[9] -
              m[4] * m[1] * m[10] +
              m[4] * m[2] * m[9] +
              m[8] * m[1] * m[6] -
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return false;

    det = 1.0 / det;

    k = 0;
    for (int i = 0; i < 4; i++)
    	for (int j = 0; j < 4; j++, k++)
    		Minv[j][i] = inv[k] * det;

    return true;
}

//------------------------

// Misc
void two_robots::initMatrix1(Matrix &M, int n, int m) {
	M.resize(n);
	for (int i = 0; i < n; ++i)
		M[i].resize(m);
}

void two_robots::initVector1(Vector &V, int n) {
	V.resize(n);
}

double two_robots::deg2rad(double deg) {
	return deg * PI / 180.0;
}

void two_robots::printMatrix(Matrix M) {
	for (unsigned i = 0; i < M.size(); i++) {
		for (unsigned j = 0; j < M[i].size(); j++) {
			if (fabs(M[i][j])>1e-4)
				cout << M[i][j] << " ";
			else
				cout << 0 << " ";
		}
		cout << endl;
	}
}

void two_robots::printVector(Vector p) {
	cout << "[";
	for (unsigned i = 0; i < p.size(); i++)
		cout << p[i] << " ";
	cout << "]" << endl;
}

void two_robots::clearMatrix(Matrix &M) {
	for (unsigned i=0; i<M.size(); ++i)
		for (unsigned j=0; j<M[i].size(); ++j)
			M[i][j] = 0;;
}

int two_robots::get_IK_number() {
	return IK_number;
}

Matrix two_robots::get_T2() {
	return T2;
}

void two_robots::setInitAngles(double q1_6, double q2_6) {
	q1_6_init = q1_6;
	q2_6_init = q2_6;
}


void two_robots::relax_joint_limits() {
	// Joint limits
	/*q1minmax = deg2rad(175);
	q2minmax = deg2rad(115);
	q3min = deg2rad(-115);
	q3max = deg2rad(75);
	q4minmax = deg2rad(165);
	q5minmax = deg2rad(125);
	q6minmax = deg2rad(400);*/

	q1minmax = deg2rad(165);
	q2minmax = deg2rad(110);
	q3min = deg2rad(-110);
	q3max = deg2rad(70);
	q4minmax = deg2rad(160);
	q5minmax = deg2rad(120);
	q6minmax = deg2rad(400);
}

void two_robots::tight_joint_limits() {
	// Joint limits
	q1minmax = deg2rad(164);
	q2minmax = deg2rad(109);
	q3min = deg2rad(-109);
	q3max = deg2rad(69);
	q4minmax = deg2rad(157);
	q5minmax = deg2rad(117);
	q6minmax = deg2rad(180);//400);
}

// ==================================
