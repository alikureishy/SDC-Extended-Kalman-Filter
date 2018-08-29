#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  cout << "F.1" << endl;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  VectorXd z = measurement_pack.raw_measurements_;
  cout << "F.2" << endl;

  if (!is_initialized_) {
    cout << "F.2.1" << endl;
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */

    // Initialize the matrices:
    ekf_.x_ = VectorXd(4);
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.Q_ = MatrixXd(4, 4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "F.2.1.1" << endl;
      double rho = z(0);
      double theta = z(1);
      double rho_dot = z(2);

      double px = rho * cos(theta);
      double py = rho * sin(theta);
      double vx = rho_dot * cos(theta);
      double vy = rho_dot * sin(theta);
      ekf_.x_ << px, py, vx, vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      cout << "F.2.1.2" << endl;
      /**
      Initialize state.
      */
  		ekf_.x_ << z(0), z(1), 0, 0;
    }

    // Initial state uncertainty
    cout << "F.2.2" << endl;
    ekf_.P_ <<  1000, 0, 0, 0,
                0, 1000, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

    // Initialize prediction function
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // Update F for elapsed time
	ekf_.F_(0, 2) *= dt;
	ekf_.F_(1, 3) *= dt;
  cout << "F.3_" << endl;

  // Update Q with elapsed time
  int noise_ax = 9;
  int noise_ay = 9;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  cout << "F.3" << endl;
  ekf_.Q_ << (dt_4*noise_ax/4), 0, (dt_3*noise_ax/2), 0,
              0, (dt_4*noise_ay/4), 0, (dt_3*noise_ay/2),
             (dt_3*noise_ax/2), 0, (dt_2*noise_ax), 0,
              0, (dt_3*noise_ay/2), 0, (dt_2*noise_ay);	
  cout << "F.4" << endl;

  ekf_.Predict();

  cout << "F.5" << endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    MatrixXd Hj = Tools().CalculateJacobian(ekf_.x_);
    cout << "F.5.1" << endl;
    ekf_.R_ = R_radar_;
    ekf_.H_ = Hj;
    ekf_.UpdateEKF(z);
    cout << "F.5.2" << endl;
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(z);
    cout << "F.5.3" << endl;
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
