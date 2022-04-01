/*
 *  ft_calib.h
 *
 *  Least squares calibration of:
 *  - Bias of F/T sensor
 *  - Mass of attached gripper
 *  - Location of the center of mass of the gripper
 *
 */

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <Eigen/Core>

// Least Squares calibration of bias of FT sensor and the mass and location of
// the COM of the gripper
namespace moma_sensor_tools {

class FTCalibration {
 public:
  FTCalibration();
  ~FTCalibration() = default;

  // adds a F/T measurement and gravity which
  // is assumed to be expressed in the F/T sensor frame
  void addMeasurement(const geometry_msgs::Vector3Stamped &gravity,
                      const geometry_msgs::WrenchStamped &ft_raw);

  // Least squares to estimate the F/T sensor parameters
  // The estimated parameters are :
  // [m m*cx m*cy m*cz FBx FBy FBz TBx TBy TBz]
  // m: mass of the tool
  // [cx, cy, cz] are the coordinates of the center of mass of the gripper
  // FB : force bias
  // TB: torque bias
  // All expressed in the FT sensor frame
  Eigen::VectorXd getCalib();

  // measurement matrix based on "On-line Rigid Object Recognition and Pose
  // Estimation
  //  Based on Inertial Parameters", D. Kubus, T. Kroger, F. Wahl, IROS 2008
  Eigen::MatrixXd getMeasurementMatrix(const Eigen::Vector3d &gravity);

 protected:
  Eigen::MatrixXd H_;    // stacked measurement matrices
  Eigen::VectorXd Z_;    // stacked F/T measurements
  unsigned int n_meas_;  // number of stacked measurements;
};
}  // namespace moma_sensor_tools
