#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /** TODO: predict the state */
	x_ = F_*x_; //x_ or x ??
	MatrixXd Ft = F_.transpose(); //section 9.5
	P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /** TODO: update the state by using Kalman Filter equations  */

    // KF Measurement update step
	MatrixXd I = MatrixXd::Identity(4, 4);
	VectorXd y = z - H_*x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S_ = H_*P_*Ht + R_;
	MatrixXd K = P_*Ht*S_.inverse();
	x_ = x_ + K*y;
	P_ = (I - (K*H_))*P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /** TODO: update the state by using Extended Kalman Filter equations */

  //recover state parameters
	float x = x_(0);
	float y = x_(1);
	float vx = x_(2);
	float vy = x_(3);
	
	
	float rho = sqrt(x*x + y*y);
	float theta = atan2(y , x);
	float rho_dot = ( x*vx + y*vy) / rho;

	if (fabs(rho) < 0.0001) {
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return;
	}

	VectorXd z_pred = VectorXd(3);
	z_pred << rho, theta, rho_dot;

	VectorXd ym = z - z_pred;

	MatrixXd I = MatrixXd::Identity(4, 4);
	MatrixXd Ht = H_.transpose();
	MatrixXd S_ = H_*P_*Ht + R_;
	MatrixXd K = P_*Ht*S_.inverse();
	x_ = x_ + K*ym;
	P_ = (I - (K*H_))*P_;
}
