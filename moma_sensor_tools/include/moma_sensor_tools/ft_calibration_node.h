#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <moma_sensor_tools/ft_calibration.h>
#include <Eigen/Core>

#include <actionlib/client/simple_action_client.h>
#include <moma_msgs/JointAction.h>

namespace moma_sensor_tools{

class FTCalibrationNode {
public:
    FTCalibrationNode(const ros::NodeHandle &n);
    ~FTCalibrationNode() = default;

    bool init();
    bool init_ros();

    bool moveNextPose();
    void saveCalibData();
    void saveMeasurements(geometry_msgs::Vector3Stamped gravity, geometry_msgs::WrenchStamped ft_meas);
    bool finished() const;
    void ft_raw_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void addMeasurement();
    void getCalib(double &mass, Eigen::Vector3d &COM_pos, Eigen::Vector3d &f_bias, Eigen::Vector3d &t_bias);
    void averageFTMeas();

public:
    ros::NodeHandle n_;
    ros::Subscriber ft_raw_subscriber_;
    ros::Publisher gravity_publisher_;

private:

    int num_poses_;
    unsigned int pose_counter_;
    unsigned int ft_counter_;

    bool debug_;
    bool finished_;
    bool received_ft_;

    bool external_wrench_is_positive_;
    double wrench_sign_;

    FTCalibration ft_calib_;

    geometry_msgs::WrenchStamped ft_raw_;
    geometry_msgs::WrenchStamped ft_avg_;

    // TF
    std::string gravity_aligned_frame_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::Buffer tf2_buffer_;

    // calibration file name
    std::string calib_file_name_;

    // name of dir where to save calibration file
    std::string calib_file_dir_;

    // name of file with recorded gravity and F/T measurements
    std::string meas_file_name_;

    // name of directory for saving gravity and F/T measurements
    std::string meas_file_dir_;

    // joint configurations
    std::vector<Eigen::VectorXd> configurations_;

    // joint action client
    std::unique_ptr<actionlib::SimpleActionClient<moma_msgs::JointAction>> action_client_;;
};

}
