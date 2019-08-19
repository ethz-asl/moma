
// Based on the ROS tutorial: http://wiki.ros.org/Robots/TIAGo/Tutorials/trajectory_controller

// C++ headers
#include <cmath>
#include <algorithm> // Needed for std::find
#include <iterator> // Needed for std::distance
#include <sstream>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/topic.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>


// parameters
#define NUM_JOINTS 7


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;


void createArmClient(arm_control_client_Ptr& actionClient)
{
    ROS_INFO("Creating action client to arm controller ...");
    actionClient.reset( new arm_control_client("/franka/position_joint_trajectory_controller/follow_joint_trajectory") );
    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
    {
        ROS_DEBUG("Waiting for the arm_controller_action server to come up");
        ++iterations;
    }

    if ( iterations == max_iterations )
    throw std::runtime_error("Error in createArmClient: arm controller action server not available");
}

// Generates a simple trajectory with two waypoints to move TIAGo's arm
void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
{
    std::vector<double> home_joint_states{0.0, -M_PI/4.0, 0.0, -3.0*M_PI/4.0, 0.0, M_PI/2.0, M_PI/4.0};

    // The joint names, which apply to all waypoints
    std::stringstream ss;
    for (size_t i = 1; i <= NUM_JOINTS; i++) {
        ss.str(std::string());      // Initialize with empty string.
        ss << "panda_joint" << i;
        // ROS_INFO_STREAM(ss.str());
        goal.trajectory.joint_names.push_back(ss.str());
    }

    // Two waypoints in this goal trajectory
    goal.trajectory.points.resize(3);

    // First trajectory point
    // Positions
    int index = 0;
    goal.trajectory.points[index].positions = home_joint_states;
    // Velocities
    goal.trajectory.points[index].velocities.resize(7);
    for (int j = 0; j < 7; ++j)
    {
        goal.trajectory.points[index].velocities[j] = 0.0;
    }
    // To be reached 2 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(0.0);

    // Second trajectory point
    // Positions
    index += 1;
    goal.trajectory.points[index].positions.resize(7);
    goal.trajectory.points[index].positions[0] = 0.2;
    goal.trajectory.points[index].positions[1] = 0.0;
    goal.trajectory.points[index].positions[2] = -1.5;
    goal.trajectory.points[index].positions[3] = 1.94;
    goal.trajectory.points[index].positions[4] = -1.57;
    goal.trajectory.points[index].positions[5] = 0.0;
    goal.trajectory.points[index].positions[6] = 0.0;
    // Velocities
    goal.trajectory.points[index].velocities.resize(7);
    for (int j = 0; j < 7; ++j)
    {
        goal.trajectory.points[index].velocities[j] = 1.0;
    }
    // To be reached 2 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(4.0);

    // Third trajectory point
    // Positions
    index += 1;
    goal.trajectory.points[index].positions = home_joint_states;
    // Velocities
    goal.trajectory.points[index].velocities.resize(7);
    for (int j = 0; j < 7; ++j)
    {
        goal.trajectory.points[index].velocities[j] = 0.0;
    }
    // To be reached 4 seconds after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(6.0);
}

int get_joint_states(std::vector<double>& states, ros::NodeHandle &nh) {
    ROS_ASSERT(states.size() == NUM_JOINTS);

    boost::shared_ptr<sensor_msgs::JointState const> current_joint_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh, ros::Duration(0.2));
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
        if (it == current_joint_state_ptr->name.end()) {
            ROS_ERROR_STREAM("Joint state message does not contain value for link " << ss.str());
            return EXIT_FAILURE;
        } else {
            int index = std::distance(name_copy.begin(), it);
            // ROS_INFO_STREAM("Joint state of joint " << ss.str() << " is at index " << index);
            states[i-1] = current_joint_state_ptr->position[index];
            // ROS_INFO_STREAM("State is " << states[i-1]);
        }
    }
    ss.str(std::string());
    for (const double& val: states) {
        ss << val << " ";
    }
    ROS_INFO_STREAM("Joint states: " << ss.str());
    return EXIT_SUCCESS;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "jf_traj_node");

    // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    //     ros::console::notifyLoggerLevelsChanged();
    // }

    ROS_INFO("Starting application ...");

    ros::NodeHandle nh;
    if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
    {
        ROS_FATAL("Timed-out waiting for valid time.");
        return EXIT_FAILURE;
    }
    ROS_INFO("Time in sync.");

    std::vector<double> v;
    v.resize(NUM_JOINTS);
    auto blabla = get_joint_states(v, nh);

    // Create action client
    arm_control_client_Ptr ArmClient;
    createArmClient(ArmClient);

    // Specify goal trajectory
    control_msgs::FollowJointTrajectoryGoal arm_goal;
    waypoints_arm_goal(arm_goal);

    // Sends the command to start the given trajectory 1s from now
    arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    ROS_INFO("Sending trajectory command now ...");
    ArmClient->sendGoal(arm_goal);

    // Wait for trajectory execution
    bool flag_done = false;
    auto state = ArmClient->getState();
    ROS_DEBUG_STREAM("Current action state: " << state.toString());
    while(!flag_done && ros::ok())
    {
        state = ArmClient->getState();
        ROS_DEBUG_STREAM("Current action state: " << state.toString());
        flag_done = state.isDone();
        ros::Duration(0.5).sleep(); // sleep for four seconds
        blabla = get_joint_states(v, nh);
    }

    // Check trajectory status
    state = ArmClient->getState();
    ROS_DEBUG_STREAM("Final action state: " << state.toString());

    for (int i = 0; i<20; i++) {
        blabla = get_joint_states(v, nh);
        ros::Duration(0.5).sleep();
    }

    return EXIT_SUCCESS;

}
