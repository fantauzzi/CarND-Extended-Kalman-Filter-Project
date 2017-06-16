#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
	x_ = VectorXd(4);
	x_ << .0, .0, .0, .0;
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
	std::cout << "Predict" << std::endl;
	std::cout << "F" << std::endl << F_;
	std::cout << "x in=" << std::endl << x_;
	std::cout << "F in=" << std::endl << F_;

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
	std::cout << "K=" << std::endl << K<<std::endl;
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}

/*double NormalizeAngle(double angle) {
    if (angle > M_PI) {
        double temp = fmod((angle - M_PI), (2 * M_PI)); // -= 2. * M_PI;
        angle = temp - M_PI;
    } // phi normalization
    if (angle < -M_PI) {
        double temp = fmod((angle + M_PI) ,(2 * M_PI));
        angle = temp + M_PI;
    }
    return angle;
}*/

double NormalizeAngle(double angle) {
	auto x=cos(angle);
	auto y=sin(angle);
	double norm=atan2(y, x);
	return norm;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	 * update the state by using Extended Kalman Filter equations
	 */
	auto pX = x_(0);
	auto pY = x_(1);
	auto vX = x_(2);
	auto vY = x_(3);
	auto rho = sqrt(pow(pX, 2) + pow(pY, 2));
	auto theta = atan2(pY, pX);
	auto rhoDot = (pX * vX + pY * vY) / rho;
	assert(rho >0.0001);

	std::cout << "Update" << std::endl;
	std::cout << "z in=" << std::endl << z << std::endl;
	std::cout << "H in="<< std::endl << H_ << std::endl;
	std::cout << "x in="<< std::endl << x_ << std::endl;
	std::cout << "P in="<< std::endl << P_ << std::endl;


	VectorXd z_pred(3);
	z_pred << rho, theta, rhoDot;
	VectorXd y = z - z_pred;
	y(1) = NormalizeAngle(y(1));
	assert(y(1) >=-M_PI && y(1) <=M_PI);
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

	/*std::cout << "H=" << std::endl << H_ << std::endl;
	std::cout << "S="<< std::endl << S << std::endl;
	std::cout << "P="<< std::endl << P_ << std::endl;
	std::cout << "K="<< std::endl << K << std::endl;*/
	std::cout << "y="<< std::endl << y << std::endl;

}
