/*
 * Checker.h
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

#ifndef CHECKER_H_
#define CHECKER_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include "ompl/base/MotionValidator.h"
#include "ompl/base/State.h"
#include <ompl/config.h>

#include "Rod_ODE_class.h"
#include "robots_class.h"
#include "collisionDetection.h"

#include <iostream>

namespace ob = ompl::base;
using namespace std;

// ---- Mechanical calibration
/*#define osX 1092.1
#define osY 3.4
#define osZ 0.1
#define osXRot 0.064270
#define osYRot 0.0
#define osZRot 0.0*/

// ---- CV calibration
#define osX 1107.71260244183
#define osY 31.5793210069981
#define osZ 3.55700094945815
#define osXRot 0.0022
#define osYRot 0.0069
#define osZRot 0.0548

class StateValidityChecker : public rod_ode, public two_robots, public collisionDetection
{
public:
	//StateValidityChecker(const ob::SpaceInformationPtr &si, double q1_6, double q2_6) : mysi_(si.get()), two_robots({-osX/2, -osY/2, -osZ/2, 0, 0, 0}, {osX/2, osY/2, osZ/2, 0 ,0 , osRot+PI}, q1_6, q2_6), collisionDetection(osX, osY, osZ, osRot, q1_6, q2_6)
	StateValidityChecker(const ob::SpaceInformationPtr &si, double q1_6, double q2_6) : mysi_(si.get()), two_robots({0, 0, 0, 0, 0, 0}, {osX, osY, osZ, osXRot ,osYRot , osZRot+PI}, q1_6, q2_6), collisionDetection(osX, osY, osZ, osZRot, q1_6, q2_6)

{
		q_temp.resize(6);
		close_chain_return_IK.resize(2);
		cout << q1_6 << " " << q2_6 << endl;

		q1_prev.resize(6);
		q2_prev.resize(6);
		W = osX;
}; //Constructor // Avishai
	//StateValidityChecker(double q1_6, double q2_6) : two_robots({-osX/2, -osY/2, -osZ/2, 0, 0, 0}, {osX/2, osY/2, osZ/2, 0 ,0 , osRot+PI}, q1_6, q2_6), collisionDetection(osX, osY, osZ, osRot, q1_6, q2_6) {};
	StateValidityChecker(double q1_6, double q2_6) : two_robots({0, 0, 0, 0, 0, 0}, {osX, osY, osZ, osXRot ,osYRot , osZRot+PI}, q1_6, q2_6), collisionDetection(osX, osY, osZ, osZRot, q1_6, q2_6) {W = osX;};



	// Validity check
	bool isValid(const ob::State *state);
	bool isValid(const ob::State *state, int active_chain, int IK_sol);
	bool isValidSerial(const ob::State *state, int active_chain, int IK_sol);
	bool checkMotion(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol);
	bool checkMotionSerial(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol);
	bool checkMotionDecoupled(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol, int nd_out);
	bool close_chain(const ob::State *state, int);
    bool check_neighbors(ob::State *dstate, ob::State *nstate, int active_chain);
    bool check_angles_offset(Vector qa, Vector qb);
	Vector identify_state_ik(const ob::State *state);
	Vector identify_state_ik(const ob::State *state, Matrix Q);
	Vector identify_state_ik(const ob::State *state, double);

	// Retrieve and update
	void retrieveStateVector(const ob::State *state, Vector &a, Vector &q);
	void retrieveStateVectorA(const ob::State *state, Vector &a);
	void retrieveStateVector(const ob::State *state, Vector &a, Vector &q1, Vector &q2);
	void updateStateVector(const ob::State *state, Vector q1, Vector q2);
	void updateStateVectorA(const ob::State *state, Vector a);
	void printStateVector(const ob::State *state);

	void defaultSettings();
	double normDistance(Vector, Vector);
	double StateDistance(const ob::State *s1, const ob::State *s2);

	int get_valid_solution_index() {
		return valid_solution_index;
	}

	Vector get_close_chain_return_IK() {
		return close_chain_return_IK;
	}

	// Performance parameters
	int isValid_counter;
	int get_isValid_counter() {
		return isValid_counter;
	}

	double getW() {
		return W;
	}
private:
	ob::StateSpace *stateSpace_;
	ob::SpaceInformation    *mysi_;
	Vector q_temp;
	int valid_solution_index;
	Vector close_chain_return_IK;

	double W; // Distance between robots

	Vector q1_prev, q2_prev;

};





#endif /* CHECKER_H_ */
