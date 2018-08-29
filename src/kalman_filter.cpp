#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::PrettyPrint() const {
  cout << "\tx -> " << endl << x_ << endl;
  cout << "\tP -> " << endl << P_ << endl;
  cout << "\tF -> " << endl << F_ << endl;
  cout << "\tQ -> " << endl << Q_ << endl;
  cout << "\tH -> " << endl << H_ << endl;
  cout << "\tR -> " << endl << R_ << endl;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  cout << "------" << endl;
  cout << "PREDICT::" << endl;
  cout << "Start..." << endl;
  PrettyPrint();
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
  cout << "...End" << endl;
  PrettyPrint();
  cout << endl;
}

// For LIDAR measurements
void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  cout << "------" << endl;
  cout << "UPDATE:: (" << z << ")" << endl;
  cout << "Start..." << endl;
  PrettyPrint();
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd Pi = P_.inverse();
  MatrixXd K = P_ * Ht * Si; //???Pi * Ht * Si;
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(K.rows(), H_.cols());
  P_ = (I - K * H_) * P_; //???(I - K * H_) * Pi;
  cout << "...End" << endl;
  PrettyPrint();
  cout << endl;
}

// For RADAR measurements
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  cout << "------" << endl;
  cout << "UPDATE-EKF:: (" << z << ")" << endl;
  cout << "Start..." << endl;
  PrettyPrint();
  VectorXd y = z - h(x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si; //???Pi * Ht * Si;
  x_ = x_ + K * y;
  MatrixXd I = MatrixXd::Identity(K.rows(), H_.cols());
  P_ = (I - K * H_) * P_; //??? (I - K * H_) * Pi;
  cout << "...End" << endl;
  PrettyPrint();
  cout << endl;
}

// Convert cartesian coordinates to polar coordinates
VectorXd KalmanFilter::h(const VectorXd& point) const {
  VectorXd t = VectorXd(3);
  double r = sqrt(pow(x_(0),2) + pow(x_(1),2));
  t(0) = r;
  t(1) = atan2(x_(1) + M_1_PI, x_(0));
  t(2) = (x_(0)*x_(2) + x_(1)*x_(3)) / max(1e-6, r) ;
  return t;
}
