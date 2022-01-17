#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from moma_mission.utils import ros
import rospy
import numpy as np

from std_msgs.msg import String
from nav_msgs.msg import Odometry

from mavsdk_ros.msg import CommandLong, CommandAck, WaypointList, WaypointsAck, TextStatus, AlarmStatus
from mavsdk_ros.msg import HLActionItem
from mavsdk_ros.srv import Command, CommandRequest, CommandResponse
from mavsdk_ros.srv import InspectionPlan, SetUploadAlarm, SetUploadWaypointList, UpdateSeqWaypointItem
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

        self.telemetry_odom_topic = None

        # Service servers
        self.command_server = None
        self.inspection_server = None

        # Service clients 
        self.upload_alarm_client = None
        self.upload_waypoint_list_client = None
        self.upload_hl_action_client = None
        self.update_current_waypoint_item_client = None
        self.update_reached_waypoint_item_client = None
        
        # Publishers
        self.telemetry_pub = None
        self.status_pub = None
        self.alarm_pub = None
        
        # Subscribers
        self.odom_sub = None

        # Messages
        self.status_msg = TextStatus()
        self.alarm_msg = AlarmStatus()

        # Flags
        self.current_hl_command_id = -1
        self.current_hl_command = np.array([])
        
    def read_params(self):
        self.telemetry_odom_topic = rospy.get_param("~telemetry_odom_topic")
        return True

    def init_ros(self):
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
        return True
        

    def get_current_hl_command(self):
        return self.current_hl_command_id, self.current_hl_command

    def command_server_cb(self, req):
        rospy.loginfo("Received command request from gRCS.")
        
        # check that the command matches the one specified in the high level actions
        response = CommandResponse()
        response.ack = CommandAck()
        req = CommandRequest()

        req.info.command

        if (req.info.command == 0):
            self.current_hl_command_id = req.info.command
            self.current_hl_command = np.array([req.info.param1, req.info.param2, req.info.param3, 
                                                req.info.param4, req.info.param5, req.info.param6, req.info.param7])
            response.ack.result = 0 # See MAV_CMD enum
            response.ack.progress = 0
        else:
            rospy.logerr("Unknown command id")
            response.result = 1
            response.progress = 0
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

        try:
            self.upload_hl_action_client.wait_for_service(timeout=10.0)
        except rospy.ROSException as exc:
            rospy.logerr(exc)
            rospy.logerr("Aborting.")
            return False

        self.upload_hl_action_client.call(req)
        return True

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
    bridge.read_params()
    bridge.init_ros()
    bridge.run()
