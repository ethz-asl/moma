#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/String.h>
#include <moveit/robot_state/conversions.h>
#include <std_srvs/Empty.h>

bool plan_exec_safe_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Hardcoded joint positions
    std::vector<double> safe_joint_positions = {0.087, -0.873, -0.070, -2.09, 0.017, 1.326, 0.838};
    move_group.setJointValueTarget(safe_joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group.execute(my_plan);
        ROS_INFO("Trajectory executed successfully. Base can now be moved.");
    } else {
        ROS_ERROR("Planning failed.");
        return false;
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "giraffe_panda_safe");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ros::ServiceServer service = nh_private.advertiseService("go_to_safe_pos", plan_exec_safe_cb);
    ROS_INFO("Ready to plan and execute trajectory for panda safe position.");

    ros::spin();
    return 0;
}
