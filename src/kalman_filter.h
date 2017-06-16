#pragma once
#include "Eigen/Dense"

using namespace Eigen;

class KalmanFilter {
public:

	KalmanFilter();

	virtual ~KalmanFilter();

	// Initialises the EKF to its initial state, based on the given state and state covariance estimates.
	void init(const VectorXd& xInit, const MatrixXd& P_Init);

	// Returns the current state estimate, or an uninitialised vector if state hasn't been initialised yet (init() hasn't been called).
	VectorXd getState() const;

	// Returns the current state variance estimate, or an uninitialised amtrix if state hasn't been initialised yet (init() hasn't been called).
	MatrixXd getStateCovariance() const;

	/* Runs the prediction step of the KF, based on the current state and state covariance estimates, and using
	 * the given state transition matrix F and process covariance Q.
	 */
	void predict(const MatrixXd& F, const MatrixXd& Q);

	/* Runs the update step of the KF, based on the current state and covariance estimates.
	 * @param z the sensor measurements.
	 * @param H the matrix giving the linear relationship between state and measurements.
	 * @param R the sensors noise covariance matrix.
	 */
	void update(const VectorXd &z, const MatrixXd & H, const MatrixXd & R);

	/* Runs the update step of the Extended KF, based on the current state and covariance estimates.
	 * @param z the sensor measurements.
	 * @param H the Jacobian used in the linear approximation of the relationship between state and measurements.
	 * @param R the sensors noise covariance matrix.
	 */
	void updateEKF(const VectorXd &z, const MatrixXd & H, const MatrixXd & R);

private:

	// The current state estimate
	Eigen::VectorXd x;

	// The current covariance estimate
	Eigen::MatrixXd P;

	// Identity matrix, just not to fill it in again every time it is needed.
	Eigen::MatrixXd I;

	/* Runs the part of the algorithm common between KF and EKF. Parameter `y` is the difference between the last
	 * measurements and what the measurements should be if the estimated state was the actual one; it has 4 components
	 * for a KF, and 3 for an EKF.
	 */
	void basicUpdate(const VectorXd &y, const MatrixXd & H, const MatrixXd & R);

};
