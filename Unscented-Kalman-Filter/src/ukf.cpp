#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0.0);

  // state dimension
  n_x_ = 5;

  // augmented state dimension
  n_aug_ = 7;

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // lambda
  lambda_ = 3 - n_aug_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.2;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / float(lambda_ + n_aug_);
  for(int i=1; i<weights_.size(); ++i) weights_(i) = (1 / float(2 * (lambda_ + n_aug_)));

  // Predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  // NIS for radar
  NIS_radar_ = 0.0;

  // NIS for LASER
  NIS_laser_ = 0.0;
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  if (!is_initialized_) {
      if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
      }
      else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        double rho, phi;
        rho = meas_package.raw_measurements_[0];
        phi = meas_package.raw_measurements_[1];
        x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
      }
      previous_timestamp_ = meas_package.timestamp_;
      previous_measurement_ = meas_package;
      is_initialized_ = true;
      return;
  }
  else {
      double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
      previous_timestamp_ = meas_package.timestamp_;
      Prediction(dt);
      if (meas_package.sensor_type_ == MeasurementPackage::LASER) UpdateLidar(meas_package);
      else UpdateRadar(meas_package);
      previous_measurement_ = meas_package;
  }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */

void UKF::Prediction(double delta_t) {
  // AugmentedSigmaPoints

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

}
