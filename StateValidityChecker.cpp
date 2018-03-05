/*
 * Checker.cpp
 *
 *  Created on: Oct 28, 2016
 *      Author: avishai
 */

/*
myStateValidityCheckerClass::myStateValidityCheckerClass(const ob::SpaceInformationPtr &si) {

}*/

#include "StateValidityChecker.h"
#include <queue>

void StateValidityChecker::defaultSettings()
{
	stateSpace_ = mysi_->getStateSpace().get();
	if (!stateSpace_)
		OMPL_ERROR("No state space for motion validator");
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, Vector &a, Vector &q) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < 6; i++) {
		a[i] = A->values[i]; // Get state of rod
		q[i] = Q->values[i]; // Get state of robots
	}
}

void StateValidityChecker::retrieveStateVectorA(const ob::State *state, Vector &a) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);

	for (unsigned i = 0; i < 6; i++)
		a[i] = A->values[i]; // Get state of rod
}

void StateValidityChecker::retrieveStateVector(const ob::State *state, Vector &a, Vector &q1, Vector &q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < 6; i++) {
		a[i] = A->values[i]; // Set state of rod
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+6]; // Set state of robot1
	}
}

void StateValidityChecker::updateStateVector(const ob::State *state, Vector q1, Vector q2) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	//const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	for (unsigned i = 0; i < 6; i++) {
		Q->values[i] = q1[i];
		Q->values[i+6]= q2[i];
	}
}

void StateValidityChecker::updateStateVectorA(const ob::State *state, Vector a) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);

	for (unsigned i = 0; i < 6; i++)
		A->values[i] = a[i];
}

void StateValidityChecker::printStateVector(const ob::State *state) {
	// cast the abstract state type to the type we expect
	const ob::CompoundStateSpace::StateType *C_state = state->as<ob::CompoundStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType *A = C_state->as<ob::RealVectorStateSpace::StateType>(0);
	const ob::RealVectorStateSpace::StateType *Q = C_state->as<ob::RealVectorStateSpace::StateType>(1);

	Vector a(6), q1(6), q2(6), ap(2);

	for (unsigned i = 0; i < 6; i++) {
		a[i] = A->values[i]; // Set state of rod
		q1[i] = Q->values[i]; // Set state of robot1
		q2[i] = Q->values[i+6]; // Set state of robot1
	}

	cout << "a: "; printVector(a);
	cout << "q1: "; printVector(q1);
	cout << "q2: "; printVector(q2);
}

bool StateValidityChecker::close_chain(const ob::State *state, int q1_active_ik_sol) {
	// c is a 20 dimensional vector composed of [a q1 q2 ik]

	Vector a(6), q1(6), q2(6), ik(2), q1_temp;
	retrieveStateVector(state, a, q1, q2);

	close_chain_return_IK = {-1, -1};

	if (!isRodFeasible(a))
		return false;
	Matrix Q = getT(get_Points_on_Rod()-1);

	FKsolve_rob(q1, 1);
	Matrix T2 = MatricesMult(get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
	if (calc_all_IK_solutions_2(T2) == 0)
		return false;
	q2 = get_all_IK_solutions_2(q1_active_ik_sol);
	ik[0] = get_valid_IK_solutions_indices_2(q1_active_ik_sol);

	Matrix Tinv = Q;
	InvertMatrix(Q, Tinv); // Invert matrix
	FKsolve_rob(q2, 2);
	Matrix T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
	int i;
	for (i=0; i<12; i++) {
		if (!IKsolve_rob(T1, 1, i))
			continue;
		q1_temp = get_IK_solution_q1();
		if (normDistance(q1_temp,q1)<1e-1) {
			ik[1] = i;
			break;
		}
	}
	if (i==12)
		return false;

	close_chain_return_IK = ik;
	updateStateVector(state, q1, q2);
	return true;
}


Vector StateValidityChecker::identify_state_ik(const ob::State *state) {

	State a(6), q1(6), q2(6), q_temp(6), ik(2);
	retrieveStateVector(state, a, q1, q2);

	ik = {-1, -1};

	if (!isRodFeasible(a))
		return ik;
	Matrix Q = getT(get_Points_on_Rod()-1);

	// q1 is the active chain
	FKsolve_rob(q1, 1);
	Matrix T2 = MatricesMult(get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
	int n = calc_all_IK_solutions_2(T2);
	if (n == 0)
		return ik;
	for (int i = 0; i < n; i++) {
		q_temp = get_all_IK_solutions_2(i);
		if (normDistance(q_temp,q2)<1e-1) {
			ik[0] = get_valid_IK_solutions_indices_2(i);
			break;
		}
	}

	// If match was not found, find the closest
	if (ik[0]==-1) {
		int ik_min = 20;
		double minD = 1000;
		double D;
		for (int i = 0; i < n; i++) {
			q_temp = get_all_IK_solutions_2(i);
			D = normDistance(q_temp,q2);
			if (D < minD) {
				ik_min = get_valid_IK_solutions_indices_2(i);
				minD = D;
			}
		}
		if (minD>0.6)
			return ik;
		else
			ik[0] = ik_min;
	}

	// q2 is the active chain
	Matrix Tinv = Q;
	InvertMatrix(Q, Tinv); // Invert matrix
	FKsolve_rob(q2, 2);
	Matrix T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
	n = calc_all_IK_solutions_1(T1);
	if (n == 0)
		return ik;
	for (int i = 0; i < n; i++) {
		q_temp = get_all_IK_solutions_1(i);
		if (normDistance(q_temp,q1)<1e-1) {
			ik[1] = get_valid_IK_solutions_indices_1(i);
			break;
		}
	}

	// If match was not found, find the closest
	if (ik[1]==-1) {
		int ik_min = 20;
		double minD = 1000;
		double D;
		for (int i = 0; i < n; i++) {
			q_temp = get_all_IK_solutions_1(i);
			D = normDistance(q_temp,q1);
			if (D < minD) {
				ik_min = get_valid_IK_solutions_indices_1(i);
				minD = D;
			}
		}
		if (minD>0.6)
			return ik;
		else
			ik[1] = ik_min;
	}

	return ik;
}

Vector StateValidityChecker::identify_state_ik(const ob::State *state, double tol) {

	State a(6), q1(6), q2(6), q_temp(6), ik(2);
	retrieveStateVector(state, a, q1, q2);

	ik = {-1, -1};

	if (!isRodFeasible(a))
		return ik;
	Matrix Q = getT(get_Points_on_Rod()-1);

	// q1 is the active chain
	FKsolve_rob(q1, 1);
	Matrix T2 = MatricesMult(get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
	int n = calc_all_IK_solutions_2(T2);
	if (n == 0)
		return ik;
	for (int i = 0; i < n; i++) {
		q_temp = get_all_IK_solutions_2(i);
		if (normDistance(q_temp,q2)<tol) {
			ik[0] = get_valid_IK_solutions_indices_2(i);
			break;
		}
	}

	// q2 is the active chain
	Matrix Tinv = Q;
	InvertMatrix(Q, Tinv); // Invert matrix
	FKsolve_rob(q2, 2);
	Matrix T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
	n = calc_all_IK_solutions_1(T1);
	if (n == 0)
		return ik;
	for (int i = 0; i < n; i++) {
		q_temp = get_all_IK_solutions_1(i);
		if (normDistance(q_temp,q1)<tol) {
			ik[1] = get_valid_IK_solutions_indices_1(i);
			break;
		}
	}

	return ik;
}

Vector StateValidityChecker::identify_state_ik(const ob::State *state, Matrix Q) {

	State a(6), q1(6), q2(6), q_temp(6), ik(2);
	retrieveStateVector(state, a, q1, q2);

	ik = {-1, -1};

	// q1 is the active chain
	FKsolve_rob(q1, 1);
	Matrix T2 = MatricesMult(get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
	int n = calc_all_IK_solutions_2(T2);
	if (n == 0)
		return ik;
	for (int i = 0; i < n; i++) {
		q_temp = get_all_IK_solutions_2(i);
		if (normDistance(q_temp,q2)<1e-1) {
			ik[0] = get_valid_IK_solutions_indices_2(i);
			break;
		}
	}

	// q2 is the active chain
	Matrix Tinv = Q;
	InvertMatrix(Q, Tinv); // Invert matrix
	FKsolve_rob(q2, 2);
	Matrix T1 = MatricesMult(get_FK_solution_T2(), {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the opposing required matrix of the rods tip at robot 2
	T1 = MatricesMult(T1, Tinv); // Returns the REQUIRED matrix of the rods tip at rob
	n = calc_all_IK_solutions_1(T1);
	if (n == 0)
		return ik;
	for (int i = 0; i < n; i++) {
		q_temp = get_all_IK_solutions_1(i);
		if (normDistance(q_temp,q1)<1e-1) {
			ik[1] = get_valid_IK_solutions_indices_1(i);
			break;
		}
	}

	return ik;
}

// ------------------- Validity check

// Validates a state by switching between the two possible active chains and computing the specific IK solution (input) and checking collision
bool StateValidityChecker::isValid(const ob::State *state) {

	isValid_counter++;

	State a(6), q1(6), q2(6), ik(2);
	retrieveStateVector(state, a, q1, q2);

	if (!isRodFeasible(a))
		return false;
	Matrix Q = getT(get_Points_on_Rod()-1);
	ik = identify_state_ik(state, Q);

	// q1 is the active chain
	if (calc_specific_IK_solution_R1(Q, q1, ik[0])) {
		Vector q_IK = get_IK_solution_q2();
		if (normDistance(q2, q_IK) > 0.5e-1)
			return false;
		if (collision_state(getPMatrix(), q1, q_IK))
			return false;
	}
	else
		return false;

	// q2 is the active chain
	if (calc_specific_IK_solution_R2(Q, q2, ik[1])) {
		Vector q_IK = get_IK_solution_q1();
		if (normDistance(q1, q_IK) > 0.12)
			return false;
		if (collision_state(getPMatrix(), q_IK, q2))
			return false;
	}
	else {
		Vector q_IK = get_IK_solution_q1();
				printVector(q_IK);
			return false;
	}

	return true;
}

// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision
bool StateValidityChecker::isValid(const ob::State *state, int active_chain, int IK_sol) {

	isValid_counter++;

	State a(6), q1(6), q2(6);
	retrieveStateVector(state, a, q1, q2);

	if (!isRodFeasible(a))
		return false;

	switch (active_chain) {
	case 0:
		if (calc_specific_IK_solution_R1(getT(get_Points_on_Rod()-1), q1, IK_sol)) {
			Vector q_IK = get_IK_solution_q2();
			if (!collision_state(getPMatrix(), q1, q_IK))
				return true;
		}
		else
			return false;
		break;
	case 1:
		if (calc_specific_IK_solution_R2(getT(get_Points_on_Rod()-1), q2, IK_sol)) {
			Vector q_IK = get_IK_solution_q1();
			if (!collision_state(getPMatrix(), q_IK, q2))
				return true;
		}
		else
			return false;
	}
	return false;
}

bool StateValidityChecker::checkMotion(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol)
{
	// We assume motion starts and ends in a valid configuration - due to projection
	bool result = true;
	int nd = stateSpace_->validSegmentCount(s1, s2);

	// initialize the queue of test positions
	std::queue< std::pair<int, int> > pos;
	if (nd >= 2)
	{
		pos.push(std::make_pair(1, nd - 1));

		// temporary storage for the checked state
		ob::State *test = mysi_->allocState();

		// repeatedly subdivide the path segment in the middle (and check the middle)
		while (!pos.empty())
		{
			std::pair<int, int> x = pos.front();

			int mid = (x.first + x.second) / 2;
			stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);

			if (!isValid(test, active_chain, ik_sol))
			{
				result = false;
				break;
			}

			pos.pop();

			if (x.first < mid)
				pos.push(std::make_pair(x.first, mid - 1));
			if (x.second > mid)
				pos.push(std::make_pair(mid + 1, x.second));
		}

		mysi_->freeState(test);
	}

	return result;
}

// Validates a state by computing the passive chain based on a specific IK solution (input) and checking collision, also check angle distance from previous node
bool StateValidityChecker::isValidSerial(const ob::State *state, int active_chain, int IK_sol) {

	isValid_counter++;

	State a(6), q1(6), q2(6);
	retrieveStateVector(state, a, q1, q2);

	if (!isRodFeasible(a))
		return false;

	bool valid = false;
	switch (active_chain) {
	case 0:
		if (calc_specific_IK_solution_R1(getT(get_Points_on_Rod()-1), q1, IK_sol)) {
			q2 = get_IK_solution_q2();
			if (!collision_state(getPMatrix(), q1, q2) && check_angles_offset(q2, q2_prev))
				valid = true;
		}
		break;
	case 1:
		if (calc_specific_IK_solution_R2(getT(get_Points_on_Rod()-1), q2, IK_sol)) {
			q1 = get_IK_solution_q1();
			if (!collision_state(getPMatrix(), q1, q2) && check_angles_offset(q1, q1_prev))
				valid = true;
		}
	}

	q1_prev = q1;
	q2_prev = q2;

	//printVector(q1_prev);
	//printVector(q2_prev);

	return valid;
}

bool StateValidityChecker::checkMotionSerial(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol)
{
	// We assume motion starts and ends in a valid configuration - due to projection
	bool result = true;
	//int nd = stateSpace_->validSegmentCount(s1, s2);
	double d = StateDistance(s1, s2);
	int nd = (int)(d/0.25);
	//cout << "----------------------- nd: " << nd << ", " << d << endl;

	Vector a_dummy(6);
	retrieveStateVector(s1, a_dummy, q1_prev, q2_prev);
	//printVector(q1_prev);
	//printVector(q2_prev);

	// temporary storage for the checked state
	ob::State *test = mysi_->allocState();

	for (int i = 1; i < nd; i++) {
		stateSpace_->interpolate(s1, s2, (double)i / (double)(nd-1), test);

		if (!isValidSerial(test, active_chain, ik_sol))
		{
			result = false;
			break;
		}
	}

	//if (!result)
	//	cin.ignore();

	return result;
}

bool StateValidityChecker::check_angles_offset(Vector qa, Vector qb) {

	double lim = 0.6;

	Vector v = {0, 3, 5};
	for (int i=0; i < v.size(); i++) {
		if (fabs(qa[v[i]]-qb[v[i]]) > lim)
			return false;
	}

	return true;
}

double StateValidityChecker::StateDistance(const ob::State *s1, const ob::State *s2) {

	Vector aa(6), qa1(6), qa2(6);
	Vector ab(6), qb1(6), qb2(6);

	retrieveStateVector(s1, aa, qa1, qa2);
	retrieveStateVector(s2, ab, qb1, qb2);

	double sum = 0;
	for (int i=0; i < aa.size(); i++)
		sum += pow(aa[i]-ab[i], 2);
	for (int i=0; i < qa1.size(); i++)
		sum += pow(qa1[i]-qb1[i], 2) + pow(qa2[i]-qb2[i], 2);
	return sqrt(sum);
}

bool StateValidityChecker::checkMotionDecoupled(const ob::State *s1, const ob::State *s2, int active_chain, int ik_sol, int nd_out)
{
	// We assume motion starts and ends in a valid configuration - due to projection
	bool result = true;
	int nd = nd_out;

	// initialize the queue of test positions
	std::queue< std::pair<int, int> > pos;
	if (nd >= 2)
	{
		pos.push(std::make_pair(1, nd - 1));

		// temporary storage for the checked state
		ob::State *test = mysi_->allocState();

		// repeatedly subdivide the path segment in the middle (and check the middle)
		while (!pos.empty())
		{
			std::pair<int, int> x = pos.front();

			int mid = (x.first + x.second) / 2;
			stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);

			if (!isValid(test, active_chain, ik_sol))
			{
				result = false;
				break;
			}

			pos.pop();

			if (x.first < mid)
				pos.push(std::make_pair(x.first, mid - 1));
			if (x.second > mid)
				pos.push(std::make_pair(mid + 1, x.second));
		}

		mysi_->freeState(test);
	}

	return result;
}

double StateValidityChecker::normDistance(Vector a1, Vector a2) {
	double sum = 0;
	for (int i=0; i < a1.size(); i++)
		sum += pow(a1[i]-a2[i], 2);
	return sqrt(sum);
}

bool StateValidityChecker::check_neighbors(ob::State *dstate, ob::State *nstate, int active_chain) {

	Vector a(6), q1n(6), q2n(6), q1(6), q2(6);

	retrieveStateVector(dstate, a, q1, q2);
	retrieveStateVector(nstate, a, q1n, q2n);

	double lim = 2.0;
	if (!active_chain) {
		if (q2n[3]*q2[3] < 0)
			if (fabs(q2n[3]-q2[3]) > lim)
				return false;
		if (q2n[5]*q2[5] < 0)
			if (fabs(q2n[5]-q2[5]) > lim)
				return false;
		if (q2n[0]*q2[0] < 0)
			if (fabs(q2n[0]-q2[0]) > lim)
				return false;
	}
	else {
		if (q1n[3]*q1[3] < 0)
			if (fabs(q1n[3]-q1[3]) > lim)
				return false;
		if (q1n[5]*q1[5] < 0)
			if (fabs(q1n[5]-q1[5]) > lim)
				return false;
		if (q1n[0]*q1[0] < 0)
			if (fabs(q1n[0]-q1[0]) > lim)
				return false;
	}


	/*if (!active_chain) {
		for (int i=0; i < q2n.size(); i++) {
			if (fabs(q2[i]-q2n[i]) > 1)
				return false;
		}
	}
	else {
		for (int i=0; i < q1n.size(); i++) {
			if (fabs(q1[i]-q1n[i]) > 1)
				return false;
		}
	}*/

	return true;
}


