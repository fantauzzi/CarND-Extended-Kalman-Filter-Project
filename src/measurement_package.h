#pragma once
#include "Eigen/Dense"

class MeasurementPackage {
public:
	long long timestamp;

	enum SensorType {
		LASER, RADAR
	} sensorType;

	Eigen::VectorXd rawMeasurements;
};

