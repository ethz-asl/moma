//
// Created by giuseppe on 13.07.21.
// Node that publishes a fake wrench message given mass, com and bias of the
// fake gripper and force sensor. Used to test the functionality of the
// calibration suite.
//

#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char **argv) {
  ros::init(argc, argv, "dummy_wrench");
  ros::NodeHandle nh("~");

  double mass;
  Eigen::Vector3d payload;
  Eigen::Vector3d torque;
  Eigen::Vector3d com;
  Eigen::Matrix<double, 6, 1> bias;
  Eigen::Vector3d gravity_eigen;

  // init ros params
  std::string wrench_topic;
  if (!nh.param<std::string>("wrench_topic", wrench_topic, "")) {
    ROS_ERROR("Failed to get wrench_topic or invalid parameter.");
    return 0;
  }

  std::string wrench_frame;
  if (!nh.param<std::string>("wrench_frame", wrench_frame, "")) {
    ROS_ERROR("Failed to get wrench_frame or invalid parameter.");
    return 0;
  }

  std::string gravity_aligned_frame;
  if (!nh.param<std::string>("gravity_aligned_frame", gravity_aligned_frame, "")) {
    ROS_ERROR("Failed to get gravity_aligned_frame or invalid parameter.");
    return 0;
  }

  if (!nh.param<double>("mass", mass, 1.0) || mass <= 0) {
    ROS_ERROR("Failed to get mass or invalid parameter.");
    return 0;
  }

  std::vector<double> com_vec;
  if (!nh.param<std::vector<double>>("com", com_vec, {}) || com_vec.size() != 3) {
    ROS_ERROR("Failed to get com or invalid parameter.");
    return 0;
  }

  std::vector<double> bias_vec;
  if (!nh.param<std::vector<double>>("bias", bias_vec, {}) || bias_vec.size() != 6) {
    ROS_ERROR("Failed to get bias or invalid parameter.");
    return 0;
  }

  for (int i = 0; i < 3; i++) {
    com[i] = com_vec[i];
    bias[i] = bias_vec[i];
    bias[i + 3] = bias_vec[i + 3];
  }
  ROS_INFO_STREAM("Dummy wrench: mass=" << mass << ", com=" << com.transpose()
                                        << ", bias=" << bias.transpose());

  ros::Publisher wrench_publisher = nh.advertise<geometry_msgs::WrenchStamped>(wrench_topic, 1);
  ros::Rate rate(100);
  tf2_ros::Buffer tf2_buffer;
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.frame_id = wrench_frame;

  while (ros::ok()) {
    // express gravity vector in F/T sensor frame
    geometry_msgs::Vector3Stamped gravity;
    gravity.header.stamp = ros::Time();
    gravity.header.frame_id = gravity_aligned_frame;
    gravity.vector.x = 0.0;
    gravity.vector.y = 0.0;
    gravity.vector.z = -9.81;

    geometry_msgs::Vector3Stamped gravity_ft_frame;

    try {
      // target_frame, source_frame ...
      geometry_msgs::TransformStamped transform = tf2_buffer.lookupTransform(
          wrench_frame, gravity_aligned_frame, ros::Time(0), ros::Duration(3.0));
      tf2::doTransform(gravity, gravity_ft_frame, transform);
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("Error transforming gravity aligned frame to the F/T sensor frame");
      ROS_ERROR("%s.", ex.what());
    }
    gravity_eigen.x() = gravity_ft_frame.vector.x;
    gravity_eigen.y() = gravity_ft_frame.vector.y, gravity_eigen.z() = gravity_ft_frame.vector.z;

    payload = mass * gravity_eigen;
    torque = com.cross(payload);

    wrench_msg.header.stamp = ros::Time::now();
    wrench_msg.wrench.force.x = payload.x() + bias(0);
    wrench_msg.wrench.force.y = payload.y() + bias(1);
    wrench_msg.wrench.force.z = payload.z() + bias(2);
    wrench_msg.wrench.torque.x = torque.x() + bias(3);
    wrench_msg.wrench.torque.y = torque.y() + bias(4);
    wrench_msg.wrench.torque.z = torque.z() + bias(5);
    wrench_publisher.publish(wrench_msg);
    rate.sleep();
  }
  return 0;
}
