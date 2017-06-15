#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
		MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict() {
	/**
	 * predict the state
	 */
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	 * update the state by using Kalman Filter equations
	 */
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}

double NormalizeAngle(double angle) {
    if (angle > M_PI) {
        double temp = fmod((angle - M_PI), (2 * M_PI)); // -= 2. * M_PI;
        angle = temp - M_PI;
    } // phi normalization
    if (angle < -M_PI) {
        double temp = fmod((angle + M_PI) ,(2 * M_PI));
        angle = temp + M_PI;
    }
    return angle;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	 * update the state by using Extended Kalman Filter equations
	 */
	auto const pi = 3.1415926535;
	auto pX = x_(0);
	auto pY = x_(1);
	auto vX = x_(2);
	auto vY = x_(3);
	auto rho = sqrt(pow(pX, 2) + pow(pY, 2));
	auto theta = atan2(pY, pX);
	auto rhoDot = (pX * vX + pY * vY) / rho;
	VectorXd z_pred(3);
	z_pred << rho, theta, rhoDot;
	VectorXd y = z - z_pred;
	/*if (y(1) <= -pi)
		y(1) = 2 * pi + y(1);
	else if (y(1) >= pi)
		y(1) = -2 * pi + y(1);*/
	y(1) = NormalizeAngle(y(1));
	MatrixXd Hjt = H_.transpose();
	MatrixXd S = H_ * P_ * Hjt + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHjt = P_ * Hjt;
	MatrixXd K = PHjt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}
