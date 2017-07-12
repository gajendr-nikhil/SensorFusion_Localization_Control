#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd res(4);
  res << 0, 0, 0, 0;

  if (estimations.size() != ground_truth.size() ||
    estimations.size() == 0) {
    cout << "Invalid estimations or ground_truth data\n";
    return res;
  }
  for (int i = 0; i < estimations.size(); i++) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    res += residual;
  }
  res /= estimations.size();
    res = res.array().sqrt();
  return res;
}
