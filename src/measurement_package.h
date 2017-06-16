#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
  long long timestamp;

  enum SensorType{
    LASER,
    RADAR
  } sensorType;

  Eigen::VectorXd rawMeasurements;
};

#endif /* MEASUREMENT_PACKAGE_H_ */
