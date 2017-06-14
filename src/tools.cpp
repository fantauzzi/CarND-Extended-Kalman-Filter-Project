#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse.setZero();

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size()
       || estimations.size() == 0){
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array().pow(2);
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3, 4);
    auto px = x_state(0);
    auto py = x_state(1);
    auto vx = x_state(2);
    auto vy = x_state(3);

    Hj.setZero();
    auto sq_sum = pow(px, 2) + pow(py, 2);
    if (sq_sum==0) {
            cout << "Division by zero in Jacobian." << endl;
            return Hj;
    }

    auto sq_rt_sum = sqrt(sq_sum);

    Hj(0, 0) = px / sq_rt_sum;
    Hj(0, 1) = py / sq_rt_sum;
    Hj(1,0)=-py/sq_sum;
    Hj(1,1)=px/sq_sum;
    Hj(2,0)= py*(vx*py-vy*px)/pow(sq_rt_sum, 3);
    Hj(2,1)= px*(vy*px-vx*py)/pow(sq_rt_sum, 3);
    Hj(2,2)= px/sq_rt_sum;
    Hj(2,3) = py/sq_rt_sum;

    return Hj;

}
