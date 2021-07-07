#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
from moma_moveit_utils import MoveItPlanner

def home(home_id="home", attempts=3):

    # Unpause the physics
    rospy.loginfo("Unpausing Gazebo...")
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resp = unpause_gazebo()
    rospy.loginfo("Unpaused Gazebo.")


    moveit_planner = MoveItPlanner()
    
    for i in range(attempts):
        success = moveit_planner.reach_named_position(home_id)
        if success:
            return rospy.loginfo('Homed the robot!')
        else:
            rospy.logerr('Failed to home the robot')
        rospy.loginfo("Trying to home the robot: retrying ...")
        rospy.sleep(2.0)
    rospy.logerr('Failed to home the robot')


if __name__ == '__main__':
  rospy.init_node('init_robot')
  home_id = rospy.get_param("~home_id", "home")
  main(home_id=home_id)
