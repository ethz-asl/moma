#include <moma_sensor_tools/ft_calibration.h>
#include <ros/ros.h>

#include <Eigen/Dense>

using namespace moma_sensor_tools;

FTCalibration::FTCalibration() { n_meas_ = 0; }

void FTCalibration::addMeasurement(const geometry_msgs::Vector3Stamped &gravity,
                                   const geometry_msgs::WrenchStamped &ft_raw) {
  if (gravity.header.frame_id != ft_raw.header.frame_id) {
    ROS_ERROR(
        "Gravity vector and ft raw expressed in different frames (%s, %s)!",
        gravity.header.frame_id.c_str(), ft_raw.header.frame_id.c_str());
    return;
  }

  n_meas_++;

  Eigen::MatrixXd h = getMeasurementMatrix(gravity);
  Eigen::VectorXd z = Eigen::Matrix<double, 6, 1>::Zero();
  z(0) = ft_raw.wrench.force.x;
  z(1) = ft_raw.wrench.force.y;
  z(2) = ft_raw.wrench.force.z;

  z(3) = ft_raw.wrench.torque.x;
  z(4) = ft_raw.wrench.torque.y;
  z(5) = ft_raw.wrench.torque.z;

  if (n_meas_ == 1) {
    H_ = h;
    Z_ = z;
  }

  else {
    Eigen::MatrixXd H_temp = H_;
    Eigen::VectorXd Z_temp = Z_;

    H_.resize(n_meas_ * 6, 10);
    Z_.resize(n_meas_ * 6);

    H_.topRows((n_meas_ - 1) * 6) = H_temp;
    Z_.topRows((n_meas_ - 1) * 6) = Z_temp;

    H_.bottomRows(6) = h;
    Z_.bottomRows(6) = z;
  }
}

// Least squares to estimate the FT sensor parameters
Eigen::VectorXd FTCalibration::getCalib() {
  Eigen::VectorXd ft_calib_params(10);

  ft_calib_params =
      H_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Z_);

  return ft_calib_params;
}

Eigen::MatrixXd FTCalibration::getMeasurementMatrix(
    const geometry_msgs::Vector3Stamped &gravity) {
  Eigen::Vector3d w;
  Eigen::Vector3d alpha;
  Eigen::Vector3d a;

  Eigen::Vector3d g(gravity.vector.x, gravity.vector.y, gravity.vector.z);

  Eigen::MatrixXd H;
  H = Eigen::Matrix<double, 6, 10>::Zero();

  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 4; j < 10; j++) {
      if (i == j - 4) {
        H(i, j) = 1.0;
      } else {
        H(i, j) = 0.0;
      }
    }
  }

  for (unsigned int i = 3; i < 6; i++) {
    H(i, 0) = 0.0;
  }

  H(3, 1) = 0.0;
  H(4, 2) = 0.0;
  H(5, 3) = 0.0;

  for (unsigned int i = 0; i < 3; i++) {
    H(i, 0) = a(i) - g(i);
  }

  H(0, 1) = -w(1) * w(1) - w(2) * w(2);
  H(0, 2) = w(0) * w(1) - alpha(2);
  H(0, 3) = w(0) * w(2) + alpha(1);

  H(1, 1) = w(0) * w(1) + alpha(2);
  H(1, 2) = -w(0) * w(0) - w(2) * w(2);
  H(1, 3) = w(1) * w(2) - alpha(0);

  H(2, 1) = w(0) * w(2) - alpha(1);
  H(2, 2) = w(1) * w(2) + alpha(0);
  H(2, 3) = -w(1) * w(1) - w(0) * w(0);

  H(3, 2) = a(2) - g(2);
  H(3, 3) = -a(1) + g(1);

  H(4, 1) = -a(2) + g(2);
  H(4, 3) = a(0) - g(0);

  H(5, 1) = a(1) - g(1);
  H(5, 2) = -a(0) + g(0);

  for (unsigned int i = 3; i < 6; i++) {
    for (unsigned int j = 4; j < 10; j++) {
      if (i == (j - 4)) {
        H(i, j) = 1.0;
      } else {
        H(i, j) = 0.0;
      }
    }
  }

  return H;
}
