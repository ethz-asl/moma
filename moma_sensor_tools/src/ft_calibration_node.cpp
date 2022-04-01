#include <geometry_msgs/TwistStamped.h>
#include <moma_sensor_tools/ft_calibration_node.h>

using namespace moma_sensor_tools;

FTCalibrationNode::FTCalibrationNode(const ros::NodeHandle &n) : n_(n), tf2_listener_(tf2_buffer_) {
  pose_counter_ = 0;
  ft_counter_ = 0;
  received_ft_ = false;
  finished_ = false;
}

bool FTCalibrationNode::init() {
  if (!init_ros()) return false;
  return true;
}

bool FTCalibrationNode::init_ros() {
  n_.param<bool>("debug", debug_, false);
  if (debug_) {
    gravity_publisher_ = n_.advertise<geometry_msgs::TwistStamped>("gravity_ft", 1);
  }

  if (!n_.param<std::string>("gravity_aligned_frame", gravity_aligned_frame_, "")) {
    ROS_ERROR("Failed to get gravity_aligned_frame from param server.");
    return false;
  }

  std::string ft_raw_topic;
  if (!n_.param<std::string>("wrench_topic", ft_raw_topic, "")) {
    ROS_ERROR("Failed to get wrench topic from param server.");
    return false;
  }

  ft_raw_subscriber_ = n_.subscribe(ft_raw_topic, 1, &FTCalibrationNode::ft_raw_callback, this);

  std::string joint_action_server_name;
  if (!n_.param<std::string>("joint_action_server_name", joint_action_server_name, "")) {
    ROS_ERROR("Failed to get joint_action_server_name from param server.");
    return false;
  }

  action_client_ = std::make_unique<actionlib::SimpleActionClient<moma_msgs::JointAction>>(
      joint_action_server_name, true);

  if (!n_.param<std::string>("calib_file_name", calib_file_name_, "ft_calib_data.yaml")) {
    ROS_WARN("No calib_file_name parameter, setting to default 'ft_calib.yaml'");
  }

  if (!n_.param<bool>("external_wrench_is_positive", external_wrench_is_positive_, true)) {
    ROS_WARN("No external_wrench_is_positive parameter, setting to default: true");
  }
  wrench_sign_ = (external_wrench_is_positive_) ? 1.0 : -1.0;

  if (!n_.param<std::string>("calib_file_dir", calib_file_dir_, "~/.ros/ft_calib")) {
    ROS_WARN("No calib_file_dir parameter, setting to default '~/.ros/ft_calib' ");
  }

  if (!n_.param<std::string>("meas_file_name", meas_file_name_, "ft_calib_meas.txt")) {
    ROS_WARN("No meas_file_name parameter, setting to default 'ft_calib_meas.txt'");
  }

  if (!n_.param<std::string>("meas_file_dir", meas_file_dir_, "~/.ros/ft_calib")) {
    ROS_WARN("No meas_file_dir parameter, setting to default '~/.ros/ft_calib' ");
  }

  if (!n_.param<int>("num_poses", num_poses_, 1) || num_poses_ < 1) {
    ROS_ERROR("No num_poses parameter or invalid parameter (must be > 1)");
    return false;
  }

  int pose_vector_size;
  for (int i = 0; i < num_poses_; i++) {
    std::string pose_name = "pose";
    pose_name += std::to_string(i);
    std::vector<double> pose_vector;
    if (!n_.param<std::vector<double>>(pose_name, pose_vector, {})) {
      ROS_ERROR_STREAM("No " << pose_name << " parameter or invalid parameter");
      return false;
    }
    if (i > 0 && pose_vector_size != pose_vector.size()) {
      ROS_ERROR("Pose vectors have different size!");
      return false;
    }
    pose_vector_size = pose_vector.size();
    Eigen::VectorXd pose_eigen = Eigen::VectorXd::Zero(pose_vector_size);
    for (int j = 0; j < pose_vector_size; j++) {
      pose_eigen[j] = pose_vector[j];
    }
    configurations_.push_back(pose_eigen);
    ROS_INFO_STREAM(pose_name << " = [" << pose_eigen.transpose() << "]");
  }

  // initialize the file with gravity and F/T measurements
  // expand the path
  if (!meas_file_dir_.empty() && meas_file_dir_[0] == '~') {
    assert(meas_file_dir_.size() == 1 or meas_file_dir_[1] == '/');  // or other error handling
    char const *home = getenv("HOME");
    if (home or (home = getenv("USERPROFILE"))) {
      meas_file_dir_.replace(0, 1, home);
    } else {
      char const *hdrive = getenv("HOMEDRIVE"), *hm_meas_file_dir = getenv("HOMEPATH");
      assert(hdrive);  // or other error handling
      assert(hm_meas_file_dir);
      meas_file_dir_.replace(0, 1, std::string(hdrive) + hm_meas_file_dir);
    }
  }

  std::ofstream meas_file;
  meas_file.open((meas_file_dir_ + "/" + meas_file_name_).c_str(), std::ios::out);

  std::stringstream meas_file_header;

  meas_file_header << "\% gravity , f/t measurements all expressed in F/T sensor frame\n";
  meas_file_header << "\% [gx, gy, gz, fx, fy, fz, tx, ty, tz]\n";
  meas_file << meas_file_header.str();
  meas_file.close();

  return true;
}

// Calibrates the FT sensor by putting the arm in several different positions
bool FTCalibrationNode::moveNextPose() {
  if (pose_counter_ > (configurations_.size() - 1)) {
    ROS_INFO("No more poses left.");
    finished_ = true;
    return true;
  }

  ROS_INFO_STREAM("Moving to pose " << pose_counter_ << " = ["
                                    << configurations_[pose_counter_].transpose() << "]");

  moma_msgs::JointGoal goal;
  goal.position.resize(configurations_[pose_counter_].size());
  for (int i = 0; i < configurations_[pose_counter_].size(); i++) {
    goal.position[i] = configurations_[pose_counter_][i];
  }
  action_client_->sendGoal(goal);
  bool finished_before_timeout = action_client_->waitForResult(ros::Duration(30.0));
  if (!finished_before_timeout) {
    ROS_WARN("Goal not achieved within timeout");
    return false;
  }

  ROS_INFO_STREAM("Finished executing pose " << pose_counter_);
  pose_counter_++;
  return true;
}

// prints out the pose (3-D positions) of the calibration frame at each of the
// positions of the left arm
void FTCalibrationNode::saveCalibData() {
  double mass;
  Eigen::Vector3d COM_pos;
  Eigen::Vector3d f_bias;
  Eigen::Vector3d t_bias;

  getCalib(mass, COM_pos, f_bias, t_bias);

  XmlRpc::XmlRpcValue bias;
  bias.setSize(6);
  for (unsigned int i = 0; i < 3; i++) bias[i] = (double)f_bias(i);

  for (unsigned int i = 0; i < 3; i++) bias[i + 3] = (double)t_bias(i);

  XmlRpc::XmlRpcValue COM_pose;
  COM_pose.setSize(6);
  for (unsigned int i = 0; i < 3; i++) COM_pose[i] = (double)COM_pos(i);

  for (unsigned int i = 0; i < 3; i++) COM_pose[i + 3] = 0.0;

  // set the parameters in the parameter server
  n_.setParam("/ft_calib/bias", bias);
  n_.setParam("/ft_calib/gripper_mass", mass);
  n_.setParam("/ft_calib/gripper_com_frame_id", ft_raw_.header.frame_id.c_str());
  n_.setParam("/ft_calib/gripper_com_pose", COM_pose);

  // dump the parameters to YAML file
  std::string file = calib_file_dir_ + std::string("/") + calib_file_name_;

  // first create the directory
  std::string command = std::string("mkdir -p ") + calib_file_dir_;
  std::system(command.c_str());

  // now dump the yaml file
  command.clear();
  command = std::string("rosparam dump ") + file + std::string(" /ft_calib");
  std::system(command.c_str());
}

// saves the gravity and force-torque measurements to a file for postprocessing
void FTCalibrationNode::saveMeasurements(geometry_msgs::Vector3Stamped gravity,
                                         geometry_msgs::WrenchStamped ft_meas) {
  std::ofstream meas_file;
  meas_file.open((meas_file_dir_ + "/" + meas_file_name_).c_str(), std::ios::out | std::ios::app);

  std::stringstream meas_file_text;

  meas_file_text << gravity.vector.x << " " << gravity.vector.y << " " << gravity.vector.z << " ";
  meas_file_text << ft_meas.wrench.force.x << " " << ft_meas.wrench.force.y << " "
                 << ft_meas.wrench.force.z << " ";
  meas_file_text << ft_meas.wrench.torque.x << " " << ft_meas.wrench.torque.y << " "
                 << ft_meas.wrench.torque.z << "\n";

  meas_file << meas_file_text.str();

  meas_file.close();
}

// finished moving the arm through the poses set in the config file
bool FTCalibrationNode::finished() const { return finished_; }

void FTCalibrationNode::ft_raw_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
  ft_raw_ = *msg;
  received_ft_ = true;
}

void FTCalibrationNode::addMeasurement() {
  ft_avg_.wrench.force.x = wrench_sign_ * ft_avg_.wrench.force.x / (double)ft_counter_;
  ft_avg_.wrench.force.y = wrench_sign_ * ft_avg_.wrench.force.y / (double)ft_counter_;
  ft_avg_.wrench.force.z = wrench_sign_ * ft_avg_.wrench.force.z / (double)ft_counter_;

  ft_avg_.wrench.torque.x = wrench_sign_ * ft_avg_.wrench.torque.x / (double)ft_counter_;
  ft_avg_.wrench.torque.y = wrench_sign_ * ft_avg_.wrench.torque.y / (double)ft_counter_;
  ft_avg_.wrench.torque.z = wrench_sign_ * ft_avg_.wrench.torque.z / (double)ft_counter_;

  ft_counter_ = 0;

  if (!received_ft_) {
    ROS_ERROR("Haven't received F/T sensor measurements");
    return;
  }

  // express gravity vector in F/T sensor frame
  geometry_msgs::Vector3Stamped gravity;
  gravity.header.stamp = ros::Time();
  gravity.header.frame_id = gravity_aligned_frame_;
  gravity.vector.x = 0.0;
  gravity.vector.y = 0.0;
  gravity.vector.z = -9.81;

  geometry_msgs::Vector3Stamped gravity_ft_frame;

  try {
    // target_frame, source_frame ...
    geometry_msgs::TransformStamped transform =
        tf2_buffer_.lookupTransform(ft_raw_.header.frame_id, gravity_aligned_frame_, ros::Time(0));
    tf2::doTransform(gravity, gravity_ft_frame, transform);
    gravity_ft_frame.header.frame_id = ft_raw_.header.frame_id;
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Error transforming gravity aligned frame to the F/T sensor frame");
    ROS_ERROR("%s.", ex.what());
    return;
  }

  if (debug_) {
    geometry_msgs::TwistStamped gravity;
    gravity.header.frame_id = ft_raw_.header.frame_id;
    gravity.twist.linear.x = gravity_ft_frame.vector.x;
    gravity.twist.linear.y = gravity_ft_frame.vector.y;
    gravity.twist.linear.z = gravity_ft_frame.vector.z;
    gravity_publisher_.publish(gravity);
  }

  ft_calib_.addMeasurement(gravity_ft_frame, ft_avg_);
  saveMeasurements(gravity_ft_frame, ft_avg_);
}

void FTCalibrationNode::getCalib(double &mass, Eigen::Vector3d &COM_pos, Eigen::Vector3d &f_bias,
                                 Eigen::Vector3d &t_bias) {
  Eigen::VectorXd ft_calib = ft_calib_.getCalib();

  mass = ft_calib(0);
  if (mass <= 0.0) {
    ROS_ERROR("Error in estimated mass (<= 0)");
    //		return;
  }

  Eigen::Vector3d center_mass_position(ft_calib(1) / mass, ft_calib(2) / mass, ft_calib(3) / mass);

  COM_pos = center_mass_position;

  f_bias(0) = ft_calib(4);
  f_bias(1) = ft_calib(5);
  f_bias(2) = ft_calib(6);
  t_bias(0) = ft_calib(7);
  t_bias(1) = ft_calib(8);
  t_bias(2) = ft_calib(9);
}

void FTCalibrationNode::averageFTMeas() {
  if (ft_counter_ == 0) {
    ft_avg_ = ft_raw_;
  } else {
    ft_avg_.wrench.force.x = ft_avg_.wrench.force.x + ft_raw_.wrench.force.x;
    ft_avg_.wrench.force.y = ft_avg_.wrench.force.y + ft_raw_.wrench.force.y;
    ft_avg_.wrench.force.z = ft_avg_.wrench.force.z + ft_raw_.wrench.force.z;
    ft_avg_.wrench.torque.x = ft_avg_.wrench.torque.x + ft_raw_.wrench.torque.x;
    ft_avg_.wrench.torque.y = ft_avg_.wrench.torque.y + ft_raw_.wrench.torque.y;
    ft_avg_.wrench.torque.z = ft_avg_.wrench.torque.z + ft_raw_.wrench.torque.z;
  }
  ft_counter_++;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ft_calib_node");
  ros::NodeHandle nh("~");

  FTCalibrationNode ft_calib_node(nh);
  if (!ft_calib_node.init()) {
    ROS_ERROR("Error initializing FT calibration node.");
    return 0;
  }

  /// main loop
  double loop_rate_;
  nh.param("loop_rate", loop_rate_, 650.0);
  ros::Rate loop_rate(loop_rate_);

  // waiting time after end of each pose to take F/T measurements
  double wait_time;
  nh.param("wait_time", wait_time, 4.0);

  bool ret = false;
  unsigned int n_measurements = 0;

  ros::Time t_end_move_arm = ros::Time::now();

  while (ros::ok() && !ft_calib_node.finished()) {
    // Move the arm, then calibrate sensor
    if (!ret) {
      ret = ft_calib_node.moveNextPose();
      t_end_move_arm = ros::Time::now();
    }

    // average 100 measurements to calibrate the sensor in each position
    else if ((ros::Time::now() - t_end_move_arm).toSec() > wait_time) {
      n_measurements++;
      ft_calib_node.averageFTMeas();  // average over 100 measurements;

      if (n_measurements == 100) {
        ret = false;
        n_measurements = 0;

        ft_calib_node.addMeasurement();  // stacks up measurement matrices and
                                         // FT measurements
        double mass;
        Eigen::Vector3d COM_pos;
        Eigen::Vector3d f_bias;
        Eigen::Vector3d t_bias;

        ft_calib_node.getCalib(mass, COM_pos, f_bias, t_bias);
        // clang-format off
        std::cout << "-----------------------------------------------------" << std::endl;
        std::cout << "Current calibration estimate:" << std::endl << std::endl;
        std::cout << "Mass: " << mass << std::endl << std::endl;
        std::cout << "Center of mass position (relative to FT sensor frame):" << std::endl;
        std::cout << "[" << COM_pos(0) << ", " << COM_pos(1) << ", " << COM_pos(2) << "]" << std::endl;
        std::cout << "FT bias: " << std::endl;
        std::cout << "[" << f_bias(0) << ", " << f_bias(1) << ", " << f_bias(2) << ", ";
        std::cout << t_bias(0) << ", " << t_bias(1) << ", " << t_bias(2) << "]";
        std::cout << std::endl << std::endl;
        std::cout << "-------------------------------------------------------------" << std::endl;
        // clang-format on
        ft_calib_node.saveCalibData();
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ft_calib_node.saveCalibData();
  ros::shutdown();
  return 0;
}
