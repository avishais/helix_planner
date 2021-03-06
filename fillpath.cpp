/*
 * fillpath.cpp
 *
 *  Created on: Jan 5, 2017
 *      Author: avishai
 */

#include "StateValidityChecker.h"
//#include "Rod_ODE_class.h"

// Standard libraries
#include <iostream>
#include <fstream>

// Prototypes
class fillpath : public StateValidityChecker
{
public:

	fillpath(double q1_6, double q2_6);

	void copyfiles(string from, string to);
	void copyfiles_withNum(string from, string to, int);

	void read_robot_paths(Matrix &path);

	void retrieve(Vector c, Vector &a, Vector &q1, Vector &q2);

	double distance(Vector a, Vector b);

	Vector identify_state_ik(Vector c);

	void determine_active_ik(Vector, Vector, int);

	int active_chain;
	int ik_sol;
	int ik_sol_alter;

};

void fillpath::copyfiles(string from, string to) {
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

void fillpath::copyfiles_withNum(string from, string to, int num) {
	ifstream a;
	ofstream b;
	char ch;
	a.open(from); //The file from which the content will be copied
	b.open(to); //The file to which the content will be copied
	b << num << endl;

	while (!a.eof())
	{
		a.get(ch); //reading from file object 'a'
		b<<ch; //writing to file 'b'
	}
	a.close();
	b.close();
}

void fillpath::read_robot_paths(Matrix &path) {
	std::ifstream fq,fa;
	fq.open("robot_paths.txt");
	fa.open("afile.txt");

	char ch;
	Vector c_temp(18);

	int lines;
	fq >> lines; // Number of conf.

	for (int i = 0; i < lines; i++) {
		for (int j=0; j < 6; j++)
			fa >> c_temp[j];
		for (int j=6; j < 18; j++) {
			fq >> c_temp[j];
			fq >> ch;
			//cout << c_temp[j] << " ";
		}
		//cout << endl;
		path.push_back(c_temp);
	}
	fq.close();

	//printMatrix(path);
}

void fillpath::retrieve(Vector c, Vector &a, Vector &q1, Vector &q2) {
	for (int i = 0; i < 6; i++) {
		a[i] = c[i];
		q1[i] = c[i+6];
		q2[i] = c[i+12];
	}
}

double fillpath::distance(Vector a, Vector b) {
	double sum = 0;
	for (int i = 0; i < a.size(); i++)
		sum += pow(a[i]-b[i],2);
	return sqrt(sum);
}

Vector fillpath::identify_state_ik(Vector C) {

	Vector a(6), q1(6), q2(6), q_temp(6), ik(2);
	retrieve(C, a, q1, q2);

	ik = {-1, -1};

	if (!isRodFeasible(a)) {
		cout << "a not feasible." << endl;
		return ik;
	}
	Matrix Q = getT(get_Points_on_Rod()-1);

	// q1 is the active chain
	FKsolve_rob(q1, 1);
	//printMatrix(get_FK_solution_T1());
	Matrix T2 = MatricesMult(get_FK_solution_T1(), Q); // Returns the opposing required matrix of the rods tip at robot 2
	T2 = MatricesMult(T2, {{-1, 0, 0, 0}, {0, -1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}); // Returns the REQUIRED matrix of the rods tip at robot 2
	int n = calc_all_IK_solutions_2(T2);
	//cout << "n: " << n << endl;
	if (n == 0) {
		cout << "Why?\n";
		return ik;
	}
	//cout << "q1: "; printVector(q1);
	//cout << "q2: "; printVector(q2);
	for (int i = 0; i < n; i++) {
		q_temp = get_all_IK_solutions_2(i);
		//cout << "q_temp: "; printVector(q_temp);
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
		if (minD>0.85)
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
	cout << "q1: "; printVector(q1);
	for (int i = 0; i < n; i++) {
		q_temp = get_all_IK_solutions_1(i);
		cout << "q_temp: "; printVector(q_temp);
		if (fabs(q_temp[5]-PI-q1[5]))
				q_temp[5]=q1[5];
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

		if (minD>0.85)
			return ik;
		else
			ik[1] = ik_min;
	}

	return ik;
}

void fillpath::determine_active_ik(Vector st1, Vector st2, int inx) {

	Vector ik1(2), ik2(2);

	ik1 = identify_state_ik(st1);
	printVector(ik1);
	ik2 = identify_state_ik(st2);
	printVector(ik2);

	if (ik1[0]==-1 || ik1[1]==-1 || ik2[0]==-1 || ik2[1]==-1){
		cout << "Error!!!\n";
		exit(1);
	}

	ik_sol_alter = -1;

	int count = 0;
	if (ik1[0]==ik2[0]) {
		active_chain = 0;
		ik_sol = ik1[0];
		count++;
	}
	if (ik1[1]==ik2[1] && count==0) {
		active_chain = 1;
		ik_sol = ik1[1];
		return;
	}
	if (ik1[1]==ik2[1] && count==1) {
		ik_sol_alter = ik1[1];
		return;
	}

	if (ik1[1]!=ik2[1] && count==0) {
		cout << "No common IK with nodes " << inx << " and " << inx-1 << ". Oh oh...\n";
		/*cout << ik1[0] << " " << ik1[1] << endl;
		cout << ik2[0] << " " << ik2[1] << endl;
		printVector(st1);
		printVector(st2);*/
		exit(1);
	}
}


fillpath::fillpath(double q1_6, double q2_6) : StateValidityChecker(q1_6, q2_6) {

	relax_joint_limits();

	Vector a(6), q1(6), q2(6), ik(2), ikp(2), C(18), temp(3);

	// Make a copy of the original files
	copyfiles("robot_paths.txt", "robot_paths_original.txt");
	copyfiles("rod_path.txt", "rod_path_original.txt");
	copyfiles("afile.txt", "afile_original.txt");

	//Read from robots_path.txt
	Matrix path;
	read_robot_paths(path);

	// Open a_path file
	std::ofstream qfile, pfile, afile;
	qfile.open("temp.txt", ios::out);
	pfile.open("rod_path_temp.txt", ios::out);
	afile.open("afile_temp.txt", ios::out);

	// First point
	retrieve(path[0], a, q1, q2);
	for (int j = 6; j < 18; j++) {
		qfile << path[0][j] << ",";
	}
	qfile << endl;

	rod_solve(a);
	// Log points on rod to file
	for (int k = 0; k < get_Points_on_Rod(); k++) {
		temp = getP(k);
		pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
	}
	pfile << endl;

	for (int j = 0; j<6; j++) {
		afile << a[j] << " ";
	}
	afile << endl;

	cout << "Original size: " << path.size() << endl;

	// Rest of points
	double dd = 0.5;
	int count = 1;
	for (int i = 1; i < path.size(); i++) {
		double d = distance(path[i], path[i-1]);
		int nd = ceil(d / dd);
		determine_active_ik(path[i], path[i-1], i);

		//printVector(path[i]);
		//printVector(path[i-1]);
		cout << d << " " << dd << " " << nd << " " << active_chain << " " << ik_sol_alter << endl;

		// First check which active chain is valid
		if (ik_sol_alter!=-1) {
			bool valid = true;
			for (int j = 1; j < nd; j++) {
				// interpolate
				for (int k = 0; k<18; k++)
					C[k] = path[i-1][k] + ((double)j / (nd-1)) * (path[i][k]-path[i-1][k]);

				retrieve(C, a, q1, q2);
				if (!isRodFeasible(a)) {
					valid = false;
					break;
				}

				if (!active_chain) {
					if (!calc_specific_IK_solution_R1(getT(get_Points_on_Rod()-1), q1, ik_sol)) {
						valid = false;
						break;
					}
					else
						q2 = get_IK_solution_q2();
				}
				else {
					if (!calc_specific_IK_solution_R2(getT(get_Points_on_Rod()-1), q2, ik_sol)) {
						valid = false;
						break;
					}
					else
						q1 = get_IK_solution_q1();
				}

				if (collision_state(getPMatrix(), q1, q2)) {
					valid = false;
					break;
				}
			}

			if (!valid) {
				active_chain = !active_chain;
				ik_sol = ik_sol_alter;

				if (!active_chain) {
					if (!calc_specific_IK_solution_R1(getT(get_Points_on_Rod()-1), q1, ik_sol)) {
						cout << "Second IK failed." << endl;
						exit(1);
					}
					else
						q2 = get_IK_solution_q2();
				}
				else {
					if (!calc_specific_IK_solution_R2(getT(get_Points_on_Rod()-1), q2, ik_sol)) {
						cout << "Second IK failed." << endl;
						exit(1);
					}
					else
						q1 = get_IK_solution_q1();
				}

				if (collision_state(getPMatrix(), q1, q2)) {
					cout << "Second IK failed." << endl;
					exit(1);
				}
			}
		}

		// Now write to file
		for (int j = 1; j < nd; j++) {
			// interpolate
			for (int k = 0; k<18; k++)
				C[k] = path[i-1][k] + ((double)j / (nd-1)) * (path[i][k]-path[i-1][k]);

			retrieve(C, a, q1, q2);
			isRodFeasible(a);
			if (!active_chain) {
				calc_specific_IK_solution_R1(getT(get_Points_on_Rod()-1), q1, ik_sol);
				q2 = get_IK_solution_q2();
			}
			else {
				calc_specific_IK_solution_R2(getT(get_Points_on_Rod()-1), q2, ik_sol);
				q1 = get_IK_solution_q1();
			}

			for (int j = 0; j<6; j++) {
				qfile << q1[j] << ",";
			}
			for (int j = 0; j<6; j++) {
				qfile << q2[j] << ",";
			}
			qfile << endl;

			rod_solve(a);
			// Log points on rod to file
			for (int k = 0; k < get_Points_on_Rod(); k++) {
				temp = getP(k);
				pfile << temp[0] << " " << temp[1] << " "  << temp[2] << endl;
			}
			pfile << endl;

			for (int j = 0; j<6; j++) {
				afile << a[j] << " ";
			}
			afile << endl;

			count++;
		}
	}

	qfile.close();
	pfile.close();

	// Update file with number of conf.
	copyfiles_withNum("temp.txt", "robot_paths.txt", count);
	copyfiles_withNum("rod_path_temp.txt", "rod_path.txt", count*(get_Points_on_Rod()+1));
	copyfiles("afile_temp.txt", "afile.txt");

	std::remove("temp.txt");
	std::remove("rod_path_temp.txt");
	std::remove("afile_temp.txt");
}


int main() {
	double q1_6, q2_6;

	std::ifstream File;
	File.open("init_6.txt", std::fstream::in);
	if (!File.is_open()) {
		q1_6 = 0;
		q2_6 = 0;
	}
	else {
		File >> q1_6;
		File >> q2_6;
	}

	File.close();

	fillpath fp(q1_6, q2_6);

	return 0;
}
