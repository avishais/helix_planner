#ifndef COLLISIONDETECTION
#define COLLISIONDETECTION 

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>
#include "PQP.h"
#include "model.h"
#include "/home/avishai/Downloads/omplapp/ompl/Workspace/cplanner/simulator/MatVec.h"
#include <iostream>
#include <string>
#include <vector>

typedef std::vector<std::vector< double > > Matrix;
typedef std::vector< double > Vector;


class collisionDetection
{
public:

	double offsetX, offsetY, offsetZ, offsetRot;
	collisionDetection(double,double,double,double,double,double);
	void load_models();
	int collision_state(Matrix M, Vector q1, Vector q2);
	PQP_Model base, link1, link2, link3, link4, link5, link6, EE, table;
	PQP_Model base2, link12, link22, link32, link42, link52, link62, EE2, rod;
	PQP_Model rod_EE;

	// Initial joint 6 angles of both robots
	double q1_6_init;
	double q2_6_init;

	// Performance parameters
	int collisionCheck_counter;
	double collisionCheck_time;
	int get_collisionCheck_counter() {
		return collisionCheck_counter;
	}
	double get_collisionCheck_time() {
		return collisionCheck_time;
	}
};

#endif
