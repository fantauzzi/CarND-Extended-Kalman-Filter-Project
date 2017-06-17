#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

VectorXd calculateRMSE(const vector<VectorXd> &estimations,
		const vector<VectorXd> &groundTruth) {
	VectorXd rmse(4);
	rmse.setZero();
	// Check the validity of the input parameters
	if (estimations.size() != groundTruth.size() || estimations.size() == 0) {
		cout
				<< "ERROR calculateRMSE() - Invalid estimation or groundTruth data."
				<< endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - groundTruth[i];
		residual = residual.array().pow(2);
		rmse += residual;
	}

	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd calculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3, 4);
	// Break down the state into its components
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	//Pre-compute a set of terms to avoid repeated calculation
	double sum_of_squares = px * px + py * py;
	double sq_root = sqrt(sum_of_squares);
	double cube_sq_root = (sum_of_squares * sq_root);

	//Check division by zero
	if (fabs(sum_of_squares) < threshold) {
		cout << "WARNING calculateJacobian() - Division by near-zero" << endl;
	}

	//Compute the Jacobian
	Hj << px / sq_root, py / sq_root, 0, 0, -py / sum_of_squares, px
			/ sum_of_squares, 0, 0, py * (vx * py - vy * px) / cube_sq_root, px
			* (px * vy - py * vx) / cube_sq_root, px / sq_root, py / sq_root;

	return Hj;
}

double normalizeAngle(const double angle) {
	// Perhaps not the most efficient way to do it, but it is neat!
	auto x = cos(angle);
	auto y = sin(angle);
	double norm = atan2(y, x);
	return norm;
}

