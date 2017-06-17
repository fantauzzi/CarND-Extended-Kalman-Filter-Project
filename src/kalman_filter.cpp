#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

KalmanFilter::KalmanFilter() :
		x { VectorXd(4) }, P { MatrixXd(4, 4) } {
	I = MatrixXd::Identity(x.size(), x.size()); // Need to be sure x has been constructed first, to get its size()
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::init(const VectorXd &x_in, const MatrixXd &Pin) {
	x = x_in;
	P = Pin;
}

VectorXd KalmanFilter::getState() const {
	return x;
}

MatrixXd KalmanFilter::getStateCovariance() const {
	return P;
}

void KalmanFilter::predict(const MatrixXd& F, const MatrixXd& Q) {
	x = F * x;
	P = F * P * F.transpose() + Q;
}

void KalmanFilter::basicUpdate(const VectorXd &y, const MatrixXd & H,
		const MatrixXd & R) {
	MatrixXd Ht = H.transpose();
	MatrixXd S = H * P * Ht + R;
	MatrixXd Si = S.inverse();
	MatrixXd K = P * Ht * Si;

	x = x + (K * y);
	P = (I - K * H) * P;
}

void KalmanFilter::update(const VectorXd &z, const MatrixXd & H,
		const MatrixXd & R) {
	VectorXd z_pred = H * x;
	VectorXd y = z - z_pred;
	basicUpdate(y, H, R);
}

void KalmanFilter::updateEKF(const VectorXd &z, const MatrixXd & H,
		const MatrixXd & R) {
	auto pX = x(0);
	auto pY = x(1);
	auto vX = x(2);
	auto vY = x(3);
	auto rho = hypot(pX, pY);
	auto theta = atan2(pY, pX);
	auto rhoDot = (pX * vX + pY * vY) / rho;
	if (rho < threshold) {  // check division by 0
		cout << "WARNING KalmanFilter::updateEKF() - division by near zero"
				<< endl;
		rho = threshold;
	}

	VectorXd z_pred(3);
	z_pred << rho, theta, rhoDot;
	VectorXd y = z - z_pred;
	y(1) = normalizeAngle(y(1));
	basicUpdate(y, H, R);
}
