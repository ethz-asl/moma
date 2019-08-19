#ifndef BASIC_ARM_MOVEMENT_H__
#define BASIC_ARM_MOVEMENT_H__

// C++ headers
#include <cmath>

// 3rd party headers
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

// ROS headers
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// parameters
#define NUM_JOINTS 7
#define CONST_HOME_STATE 0.0, -M_PI/4.0, 0.0, -3.0*M_PI/4.0, 0.0, M_PI/2.0, M_PI/4.0

typedef Eigen::Matrix<double, 7, 1> JointStateVector;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;

class BasicArmMovementSkills {
public:
    BasicArmMovementSkills(ros::NodeHandle& nh_);
    int getJointStates(std::vector<double>& states);
    int goToJointSetpoint(const std::vector<double>& joint_setpoints, double duration);
    int goHome();

private:
    // Variables
    const std::vector<double> home_state{CONST_HOME_STATE};
    arm_control_client_Ptr ArmClient;
    ros::NodeHandle& nh;
    control_msgs::FollowJointTrajectoryGoal traj;

    // Methods
    void createArmClient();
};


#endif
