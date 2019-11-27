#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
						MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	// NOTE: This function is unused, but kept here for completeness,
	// since it was included in original project code
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict() {
	/**
	TODO:
		* predict the state
	*/
	x_ = F_ * x_;	// + u;	// ignoring noise
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	TODO:
		* update the state by using Kalman Filter equations
	*/
	VectorXd y = z - H_ * x_;
	KalmanFilter::UpdateCore(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z_in) {
	/**
	TODO:
		* update the state by using Extended Kalman Filter equations
	*/
	const float PI = 3.1415927;

	// Make "deep copy" of z_in vector, since I will modify it internally
	VectorXd z(3);
	z << z_in(0), z_in(1), z_in(2);

	// Make measured phi fall between [-pi, pi]
	if (z(1) > PI) {
		z(1) = z(1) - 2*PI;
	}
	else if (z(1) < -PI) {
		z(1) = 2*PI + z(1);
	}

	// Calculate h(x')
	float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	float phi = atan2(x_(1), x_(0));  // returns value in range [-pi, pi]
	float rho_dot = 0.0;
	if (fabs(rho) > 1e-6) {	// prevent divide by 0
		rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
	}
	VectorXd hx(3);	// h(x')
	hx << rho, phi, rho_dot;

	// Calculate y vector and normalize y(1), i.e. make -pi < y(1) < pi
	VectorXd y = z - hx;
	if (y(1) > PI) {
		y(1) = y(1) - 2*PI;
	}
	else if (y(1) < -PI) {
		y(1) = 2*PI + y(1);
	}

	KalmanFilter::UpdateCore(y);
}

void KalmanFilter::UpdateCore(const Eigen::VectorXd &y) {
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =	P_ * Ht * Si;

	MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

	//new state
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
}
