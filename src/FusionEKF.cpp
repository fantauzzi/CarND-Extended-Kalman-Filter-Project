#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() :
		isInitialized { false }, previousTimestamp { 0 }, R_Lidar { MatrixXd(2,
				2) }, R_Radar { MatrixXd(3, 3) }, noiseAx { 9. }, noiseAy { 9. } {

	//measurement covariance matrix - laser
	R_Lidar << 0.0225, 0, 0, 0.0225;

	//measurement covariance matrix - radar
	R_Radar << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;
}

FusionEKF::~FusionEKF() {
}

VectorXd FusionEKF::getState() const {
	return ekf.getState();
}

MatrixXd FusionEKF::getStateCovariance() const{
	return ekf.getStateCovariance();
}

void FusionEKF::processMeasurement(const MeasurementPackage &measurement_pack) {

	/*****************************************************************************
	 *  Initialization
	 ****************************************************************************/
	if (!isInitialized) {
		/**
		 * Initialize the state of ekf with the first measurement.
		 * Create the covariance matrix.
		 */
		// first measurement
		previousTimestamp = measurement_pack.timestamp;
		auto x = VectorXd(4);
		auto P = MatrixXd(4, 4);
		P << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;

		if (measurement_pack.sensorType == MeasurementPackage::RADAR) {
			/**
			 Convert radar from polar to cartesian coordinates and initialize state.
			 */
			auto rho = measurement_pack.rawMeasurements[0];
			auto theta = measurement_pack.rawMeasurements[1];
			auto rhoDot = measurement_pack.rawMeasurements[2];
			auto px = rho * cos(theta);
			auto py = rho * sin(theta);
			auto xDot = rhoDot * cos(theta);
			auto yDot = rhoDot * sin(theta);
			x << px, py, xDot, yDot;
		} else if (measurement_pack.sensorType == MeasurementPackage::LASER) {
			auto px = measurement_pack.rawMeasurements[0];
			auto py = measurement_pack.rawMeasurements[1];
			x << px, py, 1, 1;
		}

		ekf.init(x, P);
		// done initializing, no need to predict or update
		isInitialized = true;
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

	auto deltaT = (measurement_pack.timestamp - previousTimestamp) / 1000000.0;
	previousTimestamp = measurement_pack.timestamp;
	auto F = MatrixXd(4, 4);
	F << 1, 0, deltaT, 0, 0, 1, 0, deltaT, 0, 0, 1, 0, 0, 0, 0, 1;

	auto Q = MatrixXd(4, 4);
	Q.setZero();
	Q(0, 0) = std::pow(deltaT, 4) / 4 * noiseAx;
	Q(2, 0) = std::pow(deltaT, 3) / 2 * noiseAx;
	Q(1, 1) = std::pow(deltaT, 4) / 4 * noiseAy;
	Q(3, 1) = std::pow(deltaT, 3) / 2 * noiseAy;
	Q(0, 2) = Q(2, 0);
	Q(2, 2) = std::pow(deltaT, 2) * noiseAx;
	Q(1, 3) = Q(3, 1);
	Q(3, 3) = std::pow(deltaT, 2) * noiseAy;

	ekf.predict(F, Q);

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/

	/**
	 * Use the sensor type to perform the update step.
	 * Update the state and covariance matrices.
	 */

	if (measurement_pack.sensorType == MeasurementPackage::RADAR) {
		auto rho = measurement_pack.rawMeasurements(0);
		auto theta = measurement_pack.rawMeasurements(1);
		auto rhoDot = measurement_pack.rawMeasurements(2);
		VectorXd g_of_z(4);  // g(z)
		g_of_z << rho * cos(theta), rho * sin(theta), rhoDot * cos(theta), rhoDot
				* sin(theta);
		auto H = calculateJacobian(g_of_z);
		ekf.updateEKF(measurement_pack.rawMeasurements, H, R_Radar);
	} else {
		auto H = MatrixXd(2, 4);
		H << 1, 0, 0, 0, 0, 1, 0, 0;

		ekf.update(measurement_pack.rawMeasurements, H, R_Lidar);

	}

	// print the output
	if (measurement_pack.sensorType == MeasurementPackage::RADAR)
		cout << "RADAR\n";
	else
		cout << "LIDAR\n";
	cout << "x_ = " << ekf.getState() << endl;
	cout << "P_ = " << ekf.getStateCovariance() << endl;
}
