#!/usr/bin/env python
import math
import numpy as np
from actionlib import SimpleActionClient
import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback,MoveBaseActionFeedback
from cmd_vel_controller import VelControllerNode


class MoveActionNode(object):
    """Moves ridgeback to desired pose.
    """
    def __init__(self,index_num,waypoint_before_goal_list,navgoals):
        self.move_client = SimpleActionClient('/mobile_base/move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move base server...")
        self.move_client.wait_for_server()
        rospy.loginfo("Move base server ready!")
        self.index_num = index_num
        self.waypoint_before_goal_list = waypoint_before_goal_list
        self.move_base_action_feedback_list = []
        self.navgoals = navgoals

    def move_to_object_cb(self):        
        goal_pose = np.array(self.navgoals[0])
        yaw = self.navgoals[1]
        object_position_msg_xy = np.array(self.navgoals[2])
        
        if goal_pose[0] == 100:
            return [100,100]
        else:
            waypoint_before_goal_pose = self.waypoint_before_goal(goal_pose,object_position_msg_xy)
            waypoint_before_goal = [waypoint_before_goal_pose,yaw]
            # waypoint_before_goal.append([waypoint_before_goal_pose,yaw])
            goal = [goal_pose,yaw]

            if self.index_num == 0:
                goal_list = [waypoint_before_goal,goal]
                self.use_move_base(goal_list[0])
                real_base_position = [self.real_base_position[0],self.real_base_position[1]]
                rospy.loginfo("Real base position in the map now is %s.",str([round(self.real_base_position[0],2),round(self.real_base_position[1],2)]))
                distance = round(np.linalg.norm(real_base_position - goal[0]),2)
                rospy.loginfo("The distance need to move forward by velocity controller is %s meters.",str(distance))
                VelControllerNode(distance,'true')
                
            else:
                waypoint_move_backward = self.waypoint_before_goal_list[-1]
                goal_list = [waypoint_move_backward,waypoint_before_goal,goal]
                VelControllerNode(1,'false')
                self.use_move_base(goal_list[1])
                real_base_position = [self.real_base_position[0],self.real_base_position[1]]
                rospy.loginfo("Real base position in the map now is %s.",str([round(self.real_base_position[0],2),round(self.real_base_position[1],2)]))
                distance = round(np.linalg.norm(real_base_position - goal[0]),2)
                rospy.loginfo("The distance need to move forward by velocity controller is %s meters.",str(distance))
                VelControllerNode(distance,'true')
            return waypoint_before_goal
        
    def use_move_base(self,pose_yaw):
        # for pose_yaw in goal_list:
        pose = pose_yaw[0]
        yaw = pose_yaw[1]
        quaternion = quaternion_from_euler(0, 0, yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  #### this is frame not topic
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = pose[0]
        goal.target_pose.pose.position.y = pose[1]
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        rospy.loginfo("Move base goal hasn't been sent...")
        self.move_client.send_goal(goal)
        rospy.loginfo("Move base goal has been sent!")
       
        rospy.Subscriber("/mobile_base/move_base/feedback", MoveBaseActionFeedback,self.feedback_cb)
        
        wait = self.move_client.wait_for_result()
        if not wait:
            rospy.logerr("Move base action server not available!")
            rospy.signal_shutdown("Move base action server not available!")
        else:
            result =  self.move_client.get_result()
            if result:
                rospy.loginfo("Move goal execution done!")

    def feedback_cb(self,msg):
        self.real_base_position = [msg.feedback.base_position.pose.position.x,msg.feedback.base_position.pose.position.y]
    
    def waypoint_before_goal(self,pose,object_position_msg_xy):
        x_target = pose[0]
        y_target = pose[1]
        waypoint_direction_vector = pose - object_position_msg_xy
        x = waypoint_direction_vector[0]
        y = waypoint_direction_vector[1]
        ori_radian = math.atan2(y,x)
        
        waypoint_x = round(x_target + math.cos(ori_radian),2)# backward or forward 1 meter
        waypoint_y = round(y_target + math.sin(ori_radian),2)
        
        rospy.loginfo("Waypoint position before final goal is %s.",str([waypoint_x,waypoint_y]))
        return [waypoint_x,waypoint_y]

def main():
    rospy.init_node("move_action_node")
    MoveActionNode()
    rospy.spin()

if __name__ == "__main__":
    main()
