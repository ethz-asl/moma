#!/usr/bin/env python
from __future__ import print_function
import rospy
from moma_utils.ros.moveit import MoveItClient
from move import MoveActionNode


class NavigationNode(object):
    def __init__(self):
        self.moveit = MoveItClient("panda_arm")
        self.index = 0
        self.waypoint_before_goal_list = []
  
        self.navgoals_final = [[[3.31, -2.87], 2.2048190464583186, [2.81, -2.19]], 
                               [[3.95, -2.31], 2.226491953036433, [3.55, -1.79]], 
                               [[2.41, 0.63], 1.5707963267948966, [2.41, 1.41]], 
                               [[1.61, 0.63], 1.5707963267948966, [1.61, 1.41]]]
        self.search_navgoals()
    
    def search_navgoals(self):
        self.objects_detected_list = []
        for goal in self.navgoals_final:
            rospy.loginfo("Go to navgoal %s to search!",str(self.navgoals_final.index(goal)))
            waypoint_before_goal = MoveActionNode(self.index,self.waypoint_before_goal_list,goal).move_to_object_cb()
            self.index +=1
            self.waypoint_before_goal_list.append(waypoint_before_goal)
    

def main():
    rospy.init_node("test_navigation_node")
    NavigationNode()
    rospy.spin()

if __name__ == "__main__":
    main()

 