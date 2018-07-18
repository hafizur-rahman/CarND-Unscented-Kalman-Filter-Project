#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (
    (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) ||
    (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  ) {
    if (!is_initialized_) {
      time_us_ = meas_package.timestamp_;

      P_ << 0.15,    0, 0, 0, 0,
               0, 0.15, 0, 0, 0,
               0,    0, 1, 0, 0,
               0,    0, 0, 1, 0,
               0,    0, 0, 0, 1;

      if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
       float ro =  meas_package.raw_measurements_(0);
       float phi =  meas_package.raw_measurements_(1);
       float ro_dot =  meas_package.raw_measurements_(2);

       x_(0) = ro * cos(phi);
       x_(1) = ro * sin(phi);
      } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
       x_(0) = meas_package.raw_measurements_(0);
       x_(1) = meas_package.raw_measurements_(1);
      }

      is_initialized_ = true;
      return;
    }

    // Predict first
    double delta_t = (meas_package.timestamp_ - time_us_)/1000000.0;
    Prediction(delta_t);

    // Update second
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      UpdateLidar(meas_package);
    }
  }  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //create sigma point matrix
  MatrixXd Xsig_ = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P_
  MatrixXd A_ = P_.llt().matrixL();

  Xsig_.col(0) = x_;  
  for (int i = 0; i < n_x_; i++) {
      Xsig_.col(i + 1) = x_ + sqrt(lambda_ + n_x_)*A_.col(i);
      Xsig_.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_)*A_.col(i);
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
