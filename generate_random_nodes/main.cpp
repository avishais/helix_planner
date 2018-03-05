#include "/home/avishai/Downloads/omplapp/ompl/Workspace/cplanner/checker_classes//Rod_ODE_class.h"
#include "/home/avishai/Downloads/omplapp/ompl/Workspace/cplanner/checker_classes//robots_class.h"
#include "/home/avishai/Downloads/omplapp/ompl/Workspace/cplanner/checker_classes//collisionDetection.h"

#include <iostream>
#include <fstream>
#include <time.h>       /* time */

#define PI 3.1416

Vector q2G(6);
Vector ikG(2);

using namespace std;

rod_ode rod;
two_robots robots({-1085.85/2, 0, 0 }, {1085.85/2, 0, PI});
collisionDetection CD(1085.85,0,0,0);


void sample_a(Vector &a_rand) {
	for (int i=0; i<6; i++)
		a_rand[i] = -31.0 + ((double)rand() / RAND_MAX)*61; // Random number in [-31,30]
}

void sample_q(Vector &q_rand) {
	Matrix bounds;
	rod.initMatrix(bounds, 6, 2);
	bounds[0][0] = -2.88;
	bounds[0][1] = 2.88;
	bounds[1][0] = -1.919;
	bounds[1][1] = 1.919;
	bounds[2][0] = -1.919;
	bounds[2][1] = 1.22;
	bounds[3][0] = -2.79;
	bounds[3][1] = 2.79;
	bounds[4][0] = -2.09;
	bounds[4][1] = 2.09;
	bounds[5][0] = -3.14; //6.98;
	bounds[5][1] = 3.14; //6.98;
	
	for (int i=0; i<6; i++)
		q_rand[i] = bounds[i][0] + ((double)rand() / RAND_MAX)*(bounds[i][1]-bounds[i][0]); // Random number in [-31,30]
}

bool close_chain_q1_active(Vector a, Vector q1) {
	// c is a 20 dimensional vector composed of [a q1 q2 ik]

	Vector q2(6), ik(2), q1_temp;

	if (!rod.isRodFeasible(a))
		return false;
	Matrix Q = rod.getT(rod.get_Points_on_Rod()-1);

	robots.FKsolve_rob(q1, 1);
	Matrix T2 = robots.MatricesMult(robots.get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = robots.MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
	int num_sol = robots.calc_all_IK_solutions_2(T2);
	if (num_sol == 0)
		return false;
	int q1_active_ik_sol = 0; // rand() % num_sol;
	q2 = robots.get_all_IK_solutions_2(q1_active_ik_sol);
	ik[0] = robots.get_valid_IK_solutions_indices_2(q1_active_ik_sol);

	if (CD.collision_state(rod.getPMatrix(), q1, q2))
		return false;

	Matrix Tinv = Q;
	robots.InvertMatrix(Q, Tinv); // Invert matrix
	robots.FKsolve_rob(q2, 2);
	Matrix T1 = robots.MatricesMult(robots.get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = robots.MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
	int i;
	for (i=0; i<12; i++) {
		if (!robots.IKsolve_rob(T1, 1, i))
			continue;
		q1_temp = robots.get_IK_solution_q1();
		if (robots.normDistance(q1_temp,q1)<1e-1) {
			ik[1] = i;
			break;
		}
	}
	if (i==12)
		return false;

	q2G = q2;
	ikG = ik;

	return true;
}


int main(int argn, char ** args) {
	int N; // Number of random points to add.
	if (argn == 1)
		N = 5; // sec
	else {
		N = atof(args[1]);
	}
	
	srand (time(NULL));
	
	Vector a(6);
	Vector q1(6), q2(6), ik(2);
	
	ofstream F;
	F.open ("randomnodes.txt", ios::app);
	
	int i = 0;
	while (i<N) {
		sample_a(a);
		if (!rod.isRodFeasible(a))
			continue;

		bool flag = true;
		int tries = 50;
		while (tries && flag) {
			tries--;
			sample_q(q1);
			if (close_chain_q1_active(a, q1)) {
				q2 = q2G;
				ik = ikG;
				flag = false;
			}
			else
				continue;
		}
		if (!tries)
			continue;

		for (int j = 0; j < 6; j++)
			F << a[j] << " ";
		for (int j = 0; j < 6; j++)
			F << q1[j] << " ";
		for (int j = 0; j < 6; j++)
			F << q2[j] << " ";
		/*for (int j = 0; j < 2; j++)
			F << ik[j] << " ";*/
		F << endl;			
		
		cout << "Added new node # " << ++i << "." << endl;
	}
	
	F.close();
	
}
