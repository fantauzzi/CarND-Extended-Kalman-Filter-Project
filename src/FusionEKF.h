#pragma once
#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
	FusionEKF();

	virtual ~FusionEKF();


	/* When called for the first time on the instantiated FusionEKF, it initialises the state estimate and
	 * co-variance matrix estimate based on the given measurements. When subsequently called, it updates the
	 * state and co-variance estimates based on the given measurements.
	 */
	void processMeasurement(const MeasurementPackage &measurement_pack);

	/* Returns the current state estimate; if processMeasurement() hasn't been called yet on the object,
	 * then it return an un-initialised vector.
	 */
	VectorXd getState() const;

	/* Returns the current co-variance estimate; if processMeasurement() hasn't been called yet on the object,
	 * then it return an un-initialised matrix.
	 */
	MatrixXd getStateCovariance() const;

private:
	// True iff the EKF has been initialised, i.e. processMeasurement() has been called at least once on the object.
	bool isInitialized;

	// Timestamp of the latest processed measurements, in microseconds from an epoch.
	long long previousTimestamp;

	// Covariance of the measurements noise distribution, it is a characteristic of the sensor.
	Eigen::MatrixXd R_Lidar;
	Eigen::MatrixXd R_Radar;

	// Process noise variance along x and y
	double noiseAx;
	double noiseAy;

	// Where the magic happens
	KalmanFilter ekf;
};

