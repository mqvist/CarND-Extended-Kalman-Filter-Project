#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Check preconditions
  assert(estimations.size());
  assert(estimations.size() == ground_truth.size());
  assert(estimations[0].size() == 4);

  VectorXd rmse = VectorXd::Zero(4);
  // Check for invalid input values
  // Calculate the sum of squared residuals
  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  // Calculate mean
  rmse /= estimations.size();
  // Calculate square root of the mean and return
  rmse = rmse.array().sqrt();
  // Check postconditions
  assert(rmse.size() == 4);
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);

  float px = x_state[0];
  float py = x_state[1];
  float vx = x_state[2];
  float vy = x_state[3];
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = c1 * c2;

  if (c1 == 0) {
    std::cout << "CalculateJacobian: px and py are both zero" << std::endl;
    return Hj;
  }

  Hj << px / c2, py / c2, 0, 0,
        -py / c1, px / c1, 0, 0,
        py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;
  return Hj;
}

Eigen::Vector4d Tools::Radar2State(const Eigen::Vector3d &radar_measurement) {
  float range = radar_measurement[0];
  float angle = radar_measurement[1];
  float range_rate = radar_measurement[2];
  return Eigen::Vector4d(range * cos(angle), range * sin(angle), range_rate * cos(angle), range_rate * sin(angle));
}

