#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
	x_ = VectorXd(4);
	x_ << .0, .0, .0, .0;
}

KalmanFilter::KalmanFilter(VectorXd& xInit, MatrixXd& P_Init) {
	x_ = xInit;
	P_ = P_Init;
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::init(VectorXd &x_in, MatrixXd &P_in) {
	x_ = x_in;
	P_ = P_in;
}

void KalmanFilter::Predict(MatrixXd& F, MatrixXd& Q) {
	/**
	 * predict the state
	 */
	std::cout << "Predict" << std::endl;
	std::cout << "x in=" << std::endl << x_;
	std::cout << "F in=" << std::endl << F;

	x_ = F * x_;
	MatrixXd Ft = F.transpose();
	P_ = F * P_ * Ft + Q;

}

void KalmanFilter::basicUpdate(const VectorXd &y, const MatrixXd & H, const MatrixXd & R) {
	MatrixXd Ht = H.transpose();
	MatrixXd S = H * P_ * Ht + R;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	std::cout << "K=" << std::endl << K << std::endl;
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H) * P_;

}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd & H, const MatrixXd & R) {
	/**
	 * update the state by using Kalman Filter equations
	 */
	VectorXd z_pred = H * x_;
	VectorXd y = z - z_pred;
	basicUpdate(y, H, R);
}


void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd & H, const MatrixXd & R) {
	/**
	 * update the state by using Extended Kalman Filter equations
	 */
	auto pX = x_(0);
	auto pY = x_(1);
	auto vX = x_(2);
	auto vY = x_(3);
	auto rho = hypot(pX, pY);
	auto theta = atan2(pY, pX);
	auto rhoDot = (pX * vX + pY * vY) / rho;
	assert(rho > 0.0001);

	std::cout << "Update" << std::endl;
	std::cout << "z in=" << std::endl << z << std::endl;
	std::cout << "H in=" << std::endl << H << std::endl;
	std::cout << "x in=" << std::endl << x_ << std::endl;
	std::cout << "P in=" << std::endl << P_ << std::endl;

	VectorXd z_pred(3);
	z_pred << rho, theta, rhoDot;
	VectorXd y = z - z_pred;
	y(1) = NormalizeAngle(y(1));
	basicUpdate(y, H, R);
	std::cout << "y=" << std::endl << y << std::endl;

}
