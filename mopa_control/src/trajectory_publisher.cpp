
// Based on the ROS tutorial: http://wiki.ros.org/Robots/TIAGo/Tutorials/trajectory_controller

// C++ headers
#include <cmath>
#include <algorithm> // Needed for std::find
#include <iterator> // Needed for std::distance
#include <sstream>

// Boost headers

// Eigen headers
#include <Eigen/Dense>

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/topic.h>


// Own headers
#include "BasicArmMovement.h"


// // Generates a simple trajectory with two waypoints to move arm
// void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
// {
//     std::vector<double> home_joint_states{0.0, -M_PI/4.0, 0.0, -3.0*M_PI/4.0, 0.0, M_PI/2.0, M_PI/4.0};
//
//     // The joint names, which apply to all waypoints
//     std::stringstream ss;
//     for (size_t i = 1; i <= NUM_JOINTS; i++) {
//         ss.str(std::string());      // Initialize with empty string.
//         ss << "panda_joint" << i;
//         // ROS_INFO_STREAM(ss.str());
//         goal.trajectory.joint_names.push_back(ss.str());
//     }
//
//     // Two waypoints in this goal trajectory
//     goal.trajectory.points.resize(3);
//
//     // First trajectory point
//     // Positions
//     int index = 0;
//     goal.trajectory.points[index].positions = home_joint_states;
//     // Velocities
//     goal.trajectory.points[index].velocities.resize(7);
//     for (int j = 0; j < 7; ++j)
//     {
//         goal.trajectory.points[index].velocities[j] = 0.0;
//     }
//     // To be reached 2 second after starting along the trajectory
//     goal.trajectory.points[index].time_from_start = ros::Duration(0.0);
//
//     // Second trajectory point
//     // Positions
//     index += 1;
//     goal.trajectory.points[index].positions.resize(7);
//     goal.trajectory.points[index].positions[0] = 0.2;
//     goal.trajectory.points[index].positions[1] = 0.0;
//     goal.trajectory.points[index].positions[2] = -1.5;
//     goal.trajectory.points[index].positions[3] = 1.94;
//     goal.trajectory.points[index].positions[4] = -1.57;
//     goal.trajectory.points[index].positions[5] = 0.0;
//     goal.trajectory.points[index].positions[6] = 0.0;
//     // Velocities
//     goal.trajectory.points[index].velocities.resize(7);
//     for (int j = 0; j < 7; ++j)
//     {
//         goal.trajectory.points[index].velocities[j] = 1.0;
//     }
//     // To be reached 2 second after starting along the trajectory
//     goal.trajectory.points[index].time_from_start = ros::Duration(4.0);
//
//     // Third trajectory point
//     // Positions
//     index += 1;
//     goal.trajectory.points[index].positions = home_joint_states;
//     // Velocities
//     goal.trajectory.points[index].velocities.resize(7);
//     for (int j = 0; j < 7; ++j)
//     {
//         goal.trajectory.points[index].velocities[j] = 0.0;
//     }
//     // To be reached 4 seconds after starting along the trajectory
//     goal.trajectory.points[index].time_from_start = ros::Duration(6.0);
// }
//



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


    BasicArmMovementSkills bam_skills(nh);


    bam_skills.goAway();
    ros::Duration(2.1).sleep();
    bam_skills.goHome();

    return EXIT_SUCCESS;
}
