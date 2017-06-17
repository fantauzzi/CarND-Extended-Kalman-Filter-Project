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
		initialised { false }, previousTimestamp { 0 }, R_Lidar { MatrixXd(2,
				2) }, R_Radar { MatrixXd(3, 3) }, noiseAx { 9. }, noiseAy { 9. } {

	//Fill in the sensore noise covariance matrices
	R_Lidar << 0.0225, 0, 0, 0.0225;
	R_Radar << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;
}

FusionEKF::~FusionEKF() {
}

VectorXd FusionEKF::getState() const {
	return ekf.getState();
}

MatrixXd FusionEKF::getStateCovariance() const {
	return ekf.getStateCovariance();
}

void FusionEKF::processMeasurement(const MeasurementPackage &measurement_pack) {

	/*****************************************************************************
	 *  Initialisation
	 ****************************************************************************/

	// If it is the first measurement to be processed...
	if (!initialised) {
		// ... initialise the KF setting its initial state estimate x and covariance matrix P
		previousTimestamp = measurement_pack.timestamp;
		auto x = VectorXd(4);
		auto P = MatrixXd(4, 4);
		P << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;

		if (measurement_pack.sensorType == MeasurementPackage::RADAR) {
			// Radar measurement needs to be converted to estimated state
			auto rho = measurement_pack.rawMeasurements[0];
			auto theta = measurement_pack.rawMeasurements[1];
			auto rhoDot = measurement_pack.rawMeasurements[2];
			auto px = rho * cos(theta);
			auto py = rho * sin(theta);
			auto xDot = rhoDot * cos(theta);
			auto yDot = rhoDot * sin(theta);
			x << px, py, xDot, yDot;
		} else if (measurement_pack.sensorType == MeasurementPackage::LASER) {
			// Lidar measurement provides initial px and py, but no velocity information
			auto px = measurement_pack.rawMeasurements[0];
			auto py = measurement_pack.rawMeasurements[1];
			x << px, py, 1, 1;
		}

		ekf.init(x, P);
		initialised = true;
		return;
	}

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/

	auto deltaT = (measurement_pack.timestamp - previousTimestamp) / 1000000.0; // deltaT is in seconds
	previousTimestamp = measurement_pack.timestamp;

	// Set F and Q based on delta T, noiseAx and noiseAy

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

	if (measurement_pack.sensorType == MeasurementPackage::RADAR) {
		/* If mesurements come from RADAR, set H to the Jacobian in the state
		 * estimated after prediction.
		 */
		auto H = calculateJacobian(ekf.getState());
		ekf.updateEKF(measurement_pack.rawMeasurements, H, R_Radar);
	} else {
		// Set H for measurements coming from LIDAR
		auto H = MatrixXd(2, 4);
		H << 1, 0, 0, 0, 0, 1, 0, 0;
		ekf.update(measurement_pack.rawMeasurements, H, R_Lidar);

	}

	// print the output, left here for debugging
	/*
	 if (measurement_pack.sensorType == MeasurementPackage::RADAR)
	 cout << "RADAR\n";
	 else
	 cout << "LIDAR\n";
	 cout << "x_ = " << ekf.getState() << endl;
	 cout << "P_ = " << ekf.getStateCovariance() << endl; */
}
