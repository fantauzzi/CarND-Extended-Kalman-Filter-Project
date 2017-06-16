#pragma once
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
MatrixXd calculateJacobian(const VectorXd& x_state);

// Ensure the angle is between -pi and pi. Perhaps not the most efficient way to do it, but it is neat!
double NormalizeAngle(double angle);

