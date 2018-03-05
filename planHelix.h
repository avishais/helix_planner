/*
 * plan_A_space.h
 *
 *  Created on: Nov 10, 2016
 *      Author: avishai
 */

#ifndef plan_A_SPACE_H_
#define plan_A_SPACE_H_

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// Modified and custom planners
#include "myRRTConnect.h"
//#include "testplanner.h"

#include "StateValidityChecker.h"

// Standard libraries
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;

// Initial and goal offset for joints 6's
double q1_6_init = -PI*0.95*1, q2_6_init = -PI*0.95*1;

bool isStateValidC(const ob::State *state);

StateValidityChecker sv(q1_6_init, q2_6_init); // The checker class

// Prototypes
class plan_H
{
public:

	plan_H() {};

	void plan(Vector c_start, Vector c_goal, double runtime);

	bool solved_bool;
	double total_runtime;
	int ode_count;
};

#endif /* plan_A_SPACE_H_ */
