#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

bool plan_exec_safe_cb(std_srvs::Empty::Request &req,
                       std_srvs::Empty::Response &res)
{
    static const std::string PLANNING_GROUP = "/panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group_interface(
        PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // moveit::core::RobotStatePtr current_state =
    // move_group_interface.getCurrentState();

    // const moveit::core::JointModelGroup* joint_model_group =
    //  move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    //  safe joint positions
    std::vector<double> safe_joint_positions = {0.087, -0.873, -0.070, -2.09,
                                                0.017, 1.326, 0.838};
    move_group_interface.setJointValueTarget(safe_joint_positions);
    // current_state->copyJointGroupPositions(joint_model_group,
    // safe_joint_positions);

    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROS_INFO("trying plan");

    bool success = (move_group_interface.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("plan success: %s", success ? "true" : "false");

    ros::spinOnce();

    if (success)
    {

        move_group_interface.execute(my_plan);
        ROS_INFO("Trajectory executed successfully. Base can now be moved.");
    }
    else
    {
        ROS_ERROR("Planning failed.");
        return false;
    }

    return true;
}

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // JOINT GOAL
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = -tau / 6; // -1/6 turn in radians
    move_group_interface.setJointValueTarget(joint_group_positions);

    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);

    success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    if (success){
        ROS_INFO("Plan worked, now executing");
        move_group_interface.execute(my_plan);
    }
    else{
        ROS_ERROR("Plan failed");
    }


    ros::shutdown();
    return 0;
}

// int main(int argc, char[] argv)
// {
//     ros::init(argc, argv, "giraffe_panda_safe");
//     ros::NodeHandle nh;
//     ros::NodeHandle nh_private("~");

//     ros::AsyncSpinner spinner(1); // for MoveIt
//     spinner.start();

//     ros::ServiceServer service =
//         nh.advertiseService("go_to_safe_pos", plan_exec_safe_cb);
//     ROS_INFO("Ready to plan and execute trajectory for panda safe position.");

//     ros::Rate rate(50);
//     while (ros::ok())
//     {
//         rate.sleep();
//     }
//     return 0;
// }
