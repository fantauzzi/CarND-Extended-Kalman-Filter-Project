#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0, 0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

	/**
	 * Finish initializing the FusionEKF.
	 * Set the process and measurement noises
	 */
	noise_ax = 5.;
	noise_ay = 5.;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

	/*****************************************************************************
	 *  Initialization
	 ****************************************************************************/
	if (!is_initialized_) {
		/**
		 * Initialize the state ekf_.x_ with the first measurement.
		 * Create the covariance matrix.
		 * Remember: you'll need to convert radar from polar to cartesian coordinates.
		 */
		// first measurement
		prev_timestamp = measurement_pack.timestamp_;
		//cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);
		// ekf_.x_ << 1, 1, 1, 1;
		ekf_.P_ = MatrixXd(4, 4);
		ekf_.P_ << 100, 0, 0, 0, 0, 100, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;

		ekf_.F_ = MatrixXd(4, 4);
		ekf_.F_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			 Convert radar from polar to cartesian coordinates and initialize state.
			 */
			auto rho = measurement_pack.raw_measurements_[0];
			auto theta = measurement_pack.raw_measurements_[1];
			auto rhoDot = measurement_pack.raw_measurements_[2];
			auto px = rho * cos(theta);
			auto py = rho * sin(theta);
			auto xDot = rhoDot * cos(theta);
			auto yDot = rhoDot * sin(theta);
			ekf_.x_ << px, py, xDot, yDot;
		} else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			auto px = measurement_pack.raw_measurements_[0];
			auto py = measurement_pack.raw_measurements_[1];
			ekf_.x_ << px, py, 0, 0;
			/**
			 Initialize state.
			 */
		}

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/

	/**
	 * Update the state:transition matrix F according to the new elapsed time.
	 - Time is measured in seconds.
	 * Update the process noise covariance matrix.
	 * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	 */

	auto deltaT = (measurement_pack.timestamp_ - prev_timestamp) / 1000000.0;
	ekf_.F_(0, 2) = deltaT;
	ekf_.F_(1, 3) = deltaT;

	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_.setZero();
	ekf_.Q_(0, 0) = std::pow(deltaT, 4) / 4 * noise_ax;
	ekf_.Q_(2, 0) = std::pow(deltaT, 3) / 2 * noise_ax;
	ekf_.Q_(1, 1) = std::pow(deltaT, 4) / 4 * noise_ay;
	ekf_.Q_(3, 1) = std::pow(deltaT, 3) / 2 * noise_ay;
	ekf_.Q_(0, 2) = ekf_.Q_(2, 0);
	ekf_.Q_(2, 2) = std::pow(deltaT, 2) * noise_ax;
	ekf_.Q_(1, 3) = ekf_.Q_(3, 1);
	ekf_.Q_(3, 3) = std::pow(deltaT, 2) * noise_ay;

	ekf_.Predict();

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/

	/**
	 * Use the sensor type to perform the update step.
	 * Update the state and covariance matrices.
	 */

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		ekf_.R_ =R_radar_;
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	} else {
		// Laser updates
		ekf_.H_ = MatrixXd(2, 4);
		ekf_.H_ << 1, 0, 0, 0, 0, 1, 0, 0;

		ekf_.R_ =R_laser_;
		ekf_.Update(measurement_pack.raw_measurements_);

	}

	// print the output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
