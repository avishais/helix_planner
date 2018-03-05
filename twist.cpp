/*
 * twist.cpp
 *
 *  Created on: Feb 7, 2017
 *      Author: avishai
 */
//#include "StateValidityChecker.h"

// Standard libraries
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

typedef vector<vector< double > > Matrix;
typedef vector< double > Vector;



// Prototypes
class twist //: public StateValidityChecker
{
public:

	twist(int);

	void read_robot_paths(Matrix &path);
	void save_robot_paths(Matrix path);

};

void twist::read_robot_paths(Matrix &path) {
	std::ifstream fq,fa;
	fq.open("robot_paths.txt");

	char ch;
	Vector c_temp(12);

	int lines;
	fq >> lines; // Number of conf.

	for (int i = 0; i < lines; i++) {
		for (int j=0; j < 12; j++) {
			fq >> c_temp[j];
			fq >> ch;
		}
		path.push_back(c_temp);
	}
	fq.close();
}

void twist::save_robot_paths(Matrix path) {
	// Open a_path file
	std::ofstream myfile;
	myfile.open("robot_path_twist.txt");

	for (int i = 0; i < path.size(); i++) {
		for (int j = 0; j<12; j++)
			myfile << path[i][j] << " ";
		myfile << endl;
	}
	myfile.close();
}

twist::twist(int N) {
	int sgn;
	const double dq = 2 * 3.14/180;
	const double lim = 400 * 3.14/180;

	//Read from robots_path.txt
	Matrix path;
	read_robot_paths(path);

	int n = path.size();

	if (path[n-1][5] > 0 && path[n-1][11] > 0)
		sgn = -1;
	else
		sgn = 1;

	double total_twist = 0;

	Vector tw = path[n-1];
	while (fabs(tw[5])+2*dq < lim && fabs(tw[11])+2*dq < lim) {
		tw[5] += sgn * dq;
		tw[11] += sgn * dq;
		path.push_back(tw);
		total_twist += 2*dq;
	}
	if (fabs(tw[5])+2*dq > lim) {
		tw[5] -= sgn * dq;
		while (fabs(tw[11])+2*dq < lim) {
			tw[11] += sgn * dq;
			path.push_back(tw);
			total_twist += dq;
		}
	}
	else if (fabs(tw[11])+2*dq > lim) {
		tw[11] -= sgn * dq;
		while (fabs(tw[5])+2*dq < lim) {
			tw[5] += sgn * dq;
			path.push_back(tw);
			total_twist += dq;
		}
	}

	/*for (int i = 0; i < N && fabs(tw[5]) < lim && fabs(tw[11]) < lim; i++) {
		tw[5] += sgn * dq;
		tw[11] += sgn * dq;
		path.push_back(tw);
	}*/

	cout << "Total twist added: " << total_twist*180/3.14 << "deg.\n";
	save_robot_paths(path);
}


//int main(int argc, char **argv) {
int main() {


	/*int N;
	if (argc > 1)
		N = atof(argv[1]);
	else
		N = 100;*/

	twist t(1);

	return 0;
}



