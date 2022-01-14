#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np

from std_msgs.msg import String
from nav_msgs.msg import Odometry

from mavsdk_ros.msg import CommandLong, CommandAck, WaypointList, WaypointsAck, TextStatus, AlarmStatus
from mavsdk_ros.msg import HLActionItem
from mavsdk_ros.srv import Command, InspectionPlan, SetUploadAlarm, SetUploadWaypointList, UpdateSeqWaypointItem
from mavsdk_ros.srv import SetUploadHLAction, SetUploadHLActionRequest

class RCSBridge:
    """
    Bridge for communication with the gRCS
    It assumes that the mavsdk_ros is running. This manage the conversion from ros to MAVLINK.
    Therefore the bridge is not a direct client of the gRCS but rather sends info to mavsdk_ros, which is the effective client.
    
    """

    def __init__(self):
        """
        services: 
        /mavsdk_ros/set_upload_alarm --> upload alarm list
        /mavsdk_ros/set_upload_waypoint_list --> upload waypoints list
        /mavsdk_ros/set_upload_hl_action --> upload hl action list
        /mavsdk_ros/update_current_waypoint_item --> indicate to the gRCS the index of the waypoint that is currently executing
        /mavsdk_ros/update_reached_waypoint_item --> Indicate to the gRCS the index of the waypoint that has just finished being executed.
        
        topics:
        /mavsdk_ros/local_position --> set the current telemetry info
        /mavsdk_ros/text_status --> text information
        """

        # params 
        self.telemetry_odom_topic = rospy.get_param("~telemetry_odom_topic")
        
        # Service servers
        self.command_server = rospy.Service('/command', Command, self.command_server_cb)
        self.inspection_server = rospy.Service('/inspection', InspectionPlan, self.inspection_server_cb)

        # Service clients 
        self.upload_alarm_client = rospy.ServiceProxy("/mavsdk_ros/set_upload_alarm", SetUploadAlarm)
        self.upload_waypoint_list_client = rospy.ServiceProxy("/mavsdk_ros/set_upload_waypoint_list", SetUploadWaypointList)
        self.upload_hl_action_client = rospy.ServiceProxy("/mavsdk_ros/set_upload_hl_action", SetUploadHLAction)
        self.update_current_waypoint_item_client = rospy.ServiceProxy("/mavsdk_ros/update_current_waypoint_item", UpdateSeqWaypointItem)
        self.update_reached_waypoint_item_client = rospy.ServiceProxy("/mavsdk_ros/update_reached_waypoint_item", UpdateSeqWaypointItem)
        
        # Publishers
        self.telemetry_pub = rospy.Publisher("/mavsdk_ros/local_position", Odometry, queue_size=1)
        self.status_pub = rospy.Publisher("/mavsdk_ros/text_status", TextStatus, queue_size=1)
        self.alarm_pub = rospy.Publisher("/mavsdk_ros/alarm_status", AlarmStatus, queue_size=1)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber(self.telemetry_odom_topic, Odometry, self.update_telemetry, queue_size=1)

        # Messages
        self.status_msg = TextStatus()
        self.alarm_msg = AlarmStatus()
        
        self.upload_hl_action()

    def command_server_cb(self, req):
        rospy.loginfo("Received command request from gRCS.")
        
        # check that the command matches the one specified in the high level actions
        response = CommandAck()
        if (req.info.command == 1):
            response.result = 1
            response.progress = 2
        else:
            response.result = 2
            response.progress = 3
        return response
   
    def inspection_server_cb(self, req):
        rospy.loginfo("Received inspection request from gRCS.")
        response = WaypointsAck()
        response.data = 0
        waypoint_list = WaypointList()
        waypoint_list = req.info

        for i in range(len(waypoint_list.items)):
            # do something with these waypoints
            pass
        return response

    def upload_hl_action(self):
        hl_action = HLActionItem()
        hl_action.command = 0
        hl_action.description ="Manipulate valve for pressure regulation."
        hl_action.name = "VALVE_MANIPULATION"
        hl_action.index = 0
    
        req = SetUploadHLActionRequest()
        req.hl_actions.append(hl_action)
        self.upload_hl_action_client.call(req)
        
    def update_telemetry(self, msg):
        """ 
        Resend msg received over odom topic to gRCS telemetry topic (relay) 
        """
        self.telemetry_pub.publish(msg)

    def update_status(self):
        pass

    def update_alarm(self):
        pass

    
    def run(self):
        while not rospy.is_shutdown():
            self.telemetry_msg.header.frame_id = "world"
            self.telemetry_msg.child_frame_id = "superpanda"
            self.telemetry_msg.pose.pose.position.x = np.random.normal()
            self.telemetry_msg.pose.pose.position.y = np.random.normal()
            self.telemetry_msg.pose.pose.position.z = np.random.normal()
            
            self.telemetry_pub.publish(self.telemetry_msg)
            rospy.sleep(0.01)

if __name__ == "__main__":
    rospy.init_node("ethz_gcs_bridge")
    bridge = RCSBridge()
    bridge.run()

