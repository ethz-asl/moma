#include "BasicArmMovement.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "utilities.h"


BasicArmMovementSkills::BasicArmMovementSkills(ros::NodeHandle& nh_):
    nh(nh_)
{
    // Create action client
    createArmClient();

    // Set up trajectory message
    std::stringstream ss;
    for (size_t i = 1; i <= NUM_JOINTS; i++) {
        ss.str(std::string());      // Initialize with empty string.
        ss << "panda_joint" << i;
        // ROS_INFO_STREAM(ss.str());
        traj.trajectory.joint_names.push_back(ss.str());
    }
}

void BasicArmMovementSkills::createArmClient() {
    ROS_INFO("Creating action client to arm controller ...");
    ArmClient.reset( new arm_control_client("/franka/position_joint_trajectory_controller/follow_joint_trajectory") );
    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while( !ArmClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
    {
        ROS_DEBUG("Waiting for the arm_controller_action server to come up");
        ++iterations;
    }

    if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}

int BasicArmMovementSkills::getJointStates(std::vector<double>& states) {
    ROS_ASSERT(states.size() == NUM_JOINTS);

    boost::shared_ptr<sensor_msgs::JointState const> current_joint_state_ptr;
    int counter = 0;
    while (!current_joint_state_ptr && counter < 10) {
        current_joint_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh, ros::Duration(0.2));
        ++counter;
    }

    if (!current_joint_state_ptr) {
        ROS_WARN("Could not read joint state message.");
        return EXIT_FAILURE;
    }

    // Find the panda joints and save their states
    std::stringstream ss;
    std::vector<std::string> name_copy = current_joint_state_ptr->name;
    for (size_t i = 1; i <= NUM_JOINTS; i++) {
        ss.str(std::string());      // Initialize with empty string.
        ss << "panda_joint" << i;
        std::vector<std::string>::iterator it = std::find(name_copy.begin(), name_copy.end(), ss.str());
        if (it == name_copy.end()) {
            ROS_ERROR_STREAM("Joint state message does not contain value for link " << ss.str());
            return EXIT_FAILURE;
        } else {
            int index = std::distance(name_copy.begin(), it);
            states[i-1] = current_joint_state_ptr->position[index];
        }
    }
    return EXIT_SUCCESS;
}

int BasicArmMovementSkills::goToJointSetpoint(const std::vector<double>& joint_setpoints, double duration) {
    ROS_INFO("Hi from go to setpoint");
    ROS_ASSERT(joint_setpoints.size() == NUM_JOINTS);

    traj.trajectory.points.resize(2);
    std::vector<double> zeros(NUM_JOINTS, 0.0);

    std::vector<double> current_joint_states;
    current_joint_states.resize(NUM_JOINTS);
    int res = getJointStates(current_joint_states);
    if (res == EXIT_FAILURE) {
        ROS_WARN("Couldn't get current joint state. Abort going to setpoint.");
        return EXIT_FAILURE;
    }

    ROS_INFO_STREAM("Start point: " << util::stdVecToStr(current_joint_states));
    ROS_INFO_STREAM("End point: " << util::stdVecToStr(joint_setpoints));

    int index = 0;
    traj.trajectory.points[index].positions = current_joint_states;
    traj.trajectory.points[index].velocities = zeros;
    traj.trajectory.points[index].time_from_start = ros::Duration(0.0);

    index += 1;
    traj.trajectory.points[index].positions = joint_setpoints;
    traj.trajectory.points[index].velocities = zeros;
    traj.trajectory.points[index].time_from_start = ros::Duration(duration);

    traj.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);

    ROS_INFO("Sending trajectory command now ...");
    ArmClient->sendGoalAndWait(traj, ros::Duration(duration + 2.0), ros::Duration(0.0));

    // Check trajectory status
    auto state = ArmClient->getState();
    ROS_DEBUG_STREAM("Final action state: " << state.toString());
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN_STREAM("Action state response: " << state.toString());
        control_msgs::FollowJointTrajectoryResultConstPtr action_res = ArmClient->getResult();
        if (!(state == actionlib::SimpleClientGoalState::ABORTED && action_res->error_code == -5)) {
            ROS_WARN("Trajectory NOT successful!");
            ROS_WARN_STREAM("Result info: " << action_res->error_code);
            return EXIT_FAILURE;
        }
    }
    ROS_INFO("Executed trajectory successfully.");
    return EXIT_SUCCESS;
}

int BasicArmMovementSkills::goHome() {
    // TODO add logic to compute duration depending on starting position
    return goToJointSetpoint(home_state, 4.0);
}

int BasicArmMovementSkills::goAway() {
    // TODO add logic to compute duration depending on starting position
    std::vector<double> away_state{0.2,0.0,0.0,-1.5,-1.57,0.47,0.5};
    return goToJointSetpoint(away_state, 4.0);
}
