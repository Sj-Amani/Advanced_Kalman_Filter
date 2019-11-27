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

	// measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
				0, 0.0225;

	// measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
				0, 0.0009, 0,
				0, 0, 0.09;

	/**
	TODO:
		* Finish initializing the FusionEKF.
		* Set the process and measurement noises
	*/
	// measurement matrix for lidar
	H_laser_ << 1, 0, 0, 0,
				0, 1, 0, 0;

	// state covariance matrix P
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1000, 0,
				0, 0, 0, 1000;

	// state transition matrix
	ekf_.F_ = MatrixXd(4, 4);

	Tools tools;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


	/*******************************
	 *	INITIALIZATION
	 *******************************/
	if (!is_initialized_) {
		/**
		TODO:
			* Initialize the state ekf_.x_ with the first measurement.
			* Create the covariance matrix.
			* Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/
		// first measurement
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);
		ekf_.x_ << 0, 0, 0, 0;  // initial guess: object is at origin and stationary

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			float rho = measurement_pack.raw_measurements_(0);
			float phi = measurement_pack.raw_measurements_(1);
			float rho_dot = measurement_pack.raw_measurements_(2);

			ekf_.x_(0) = rho * cos(phi);
			ekf_.x_(1) = rho * sin(phi);
			ekf_.x_(2) = rho_dot * cos(phi);
			ekf_.x_(3) = rho_dot * sin(phi);
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			/**
			Initialize state.
			*/
			ekf_.x_(0) = measurement_pack.raw_measurements_(0);
			ekf_.x_(1) = measurement_pack.raw_measurements_(1);
		}

		previous_timestamp_ = measurement_pack.timestamp_;

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*******************************
	 *	PREDICTION
	 *******************************/

	/**
	 TODO:
		 * Update the state transition matrix F according to the new elapsed time.
			- Time is measured in seconds.
		 * Update the process noise covariance matrix.
		 * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	 */
	// calculate elapsed time
	float dt = (measurement_pack.timestamp_ - previous_timestamp_);  // time in microseconds
	dt /= 1e6;	// convert to seconds

	// update internal timestamp
	previous_timestamp_ = measurement_pack.timestamp_;

	// update F matrix
	ekf_.F_ << 1, 0, dt, 0,
				0, 1, 0, dt,
				0, 0, 1, 0,
				0, 0, 0, 1;

	// process covariance noise matrix Q
	float noise_ax = 9;
	float noise_ay = 9;
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << pow(dt, 4) / 4 * noise_ax, 0, pow(dt, 3) / 2 * noise_ax, 0,
				0, pow(dt, 4) / 4 * noise_ay, 0, pow(dt, 3) / 2 * noise_ay,
				pow(dt, 3) / 2 * noise_ax, 0, pow(dt, 2) * noise_ax, 0,
				0, pow(dt, 3) / 2 * noise_ay, 0, pow(dt, 2) * noise_ay;

	ekf_.Predict();

	/*******************************
	 *	UPDATE
	 *******************************/
	/**
	 TODO:
		 * Use the sensor type to perform the update step.
		 * Update the state and covariance matrices.
	 */

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		Hj_ = tools.CalculateJacobian(ekf_.x_);
		ekf_.H_ = Hj_;
		ekf_.R_ = R_radar_;
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
	} else {
		// Laser updates
		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;
		ekf_.Update(measurement_pack.raw_measurements_);
	}

	// print the output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
