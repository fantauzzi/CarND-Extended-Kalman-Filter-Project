#pragma once
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/* Calculates and returns the RMSE between the given vector of estimations and the given vector of exact values (ground truth).
 * RMSE is returned by component of the state estimates, therefore estimations and ground_truth must have the same number of elements;
 * also, each element in either, which is a vector, must have the same number of elements.
 */
VectorXd calculateRMSE(const vector<VectorXd> &estimations,
		const vector<VectorXd> &ground_truth);

// Calculates and returns the Jacobian of the given state, which must be a 4 components vector.
MatrixXd calculateJacobian(const VectorXd& x_state);

// Returns the given angle after normalization, ensuring it is between -pi and pi.
double normalizeAngle(const double angle);

// Threshold below which the absolute value of a denominator is considered too close to 0
const double threshold = 0.00001;

