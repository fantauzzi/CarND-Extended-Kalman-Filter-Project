#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter(): x {VectorXd(4)}, P {MatrixXd(4,4)}, I {MatrixXd(4,4)} {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::init(const VectorXd &x_in, const MatrixXd &P_in) {
	x = x_in;
	P = P_in;
	I = MatrixXd::Identity(x.size(), x.size());
}

VectorXd KalmanFilter::getState() const {
	return x;
}
MatrixXd KalmanFilter::getStateCovariance() const {
	return P;
}


void KalmanFilter::predict(const MatrixXd& F, const MatrixXd& Q) {
	/**
	 * predict the state
	 */
	std::cout << "Predict" << std::endl;
	std::cout << "x in=" << std::endl << x;
	std::cout << "F in=" << std::endl << F;

	x = F * x;
	MatrixXd Ft = F.transpose();
	P = F * P * Ft + Q;

}

void KalmanFilter::basicUpdate(const VectorXd &y, const MatrixXd & H, const MatrixXd & R) {
	MatrixXd Ht = H.transpose();
	MatrixXd S = H * P * Ht + R;
	MatrixXd Si = S.inverse();
	MatrixXd K = P * Ht * Si;

	//new estimate
	std::cout << "K=" << std::endl << K << std::endl;
	x = x + (K * y);
	P = (I - K * H) * P;

}

void KalmanFilter::update(const VectorXd &z, const MatrixXd & H, const MatrixXd & R) {
	/**
	 * update the state by using Kalman Filter equations
	 */
	VectorXd z_pred = H * x;
	VectorXd y = z - z_pred;
	basicUpdate(y, H, R);
}


void KalmanFilter::updateEKF(const VectorXd &z, const MatrixXd & H, const MatrixXd & R) {
	auto pX = x(0);
	auto pY = x(1);
	auto vX = x(2);
	auto vY = x(3);
	auto rho = hypot(pX, pY);
	auto theta = atan2(pY, pX);
	auto rhoDot = (pX * vX + pY * vY) / rho;
	assert(rho > 0.0001);

	std::cout << "Update" << std::endl;
	std::cout << "z in=" << std::endl << z << std::endl;
	std::cout << "H in=" << std::endl << H << std::endl;
	std::cout << "x in=" << std::endl << x << std::endl;
	std::cout << "P in=" << std::endl << P << std::endl;

	VectorXd z_pred(3);
	z_pred << rho, theta, rhoDot;
	VectorXd y = z - z_pred;
	y(1) = normalizeAngle(y(1));
	basicUpdate(y, H, R);
	std::cout << "y=" << std::endl << y << std::endl;

}
