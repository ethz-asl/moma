#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from time import time
from typing import Text
from grpc import Status
import yaml

import rospy
import numpy as np
import argparse

from nav_msgs.msg import Odometry

from mavsdk_ros.msg import CommandLong, CommandAck, WaypointList, WaypointsAck
from mavsdk_ros.msg import TextStatus, AlarmStatus, AlarmItem, ChecklistItem
from mavsdk_ros.msg import HLActionItem, WaypointItem
from mavsdk_ros.srv import Command, CommandRequest, CommandResponse
from mavsdk_ros.srv import InspectionPlan
from mavsdk_ros.srv import SetUploadAlarm, SetUploadAlarmRequest
from mavsdk_ros.srv import SetUploadChecklist, SetUploadChecklistRequest
from mavsdk_ros.srv import UpdateSeqWaypointItem, UpdateSeqWaypointItemRequest
from mavsdk_ros.srv import SetUploadWaypointList, SetUploadWaypointListRequest
from mavsdk_ros.srv import SetUploadHLAction, SetUploadHLActionRequest
from moma_mission.utils import ros


class WaypointStatus:
    IN_PROGRESS = 0
    QUEUED = 1
    DONE = 2

    @staticmethod
    def to_string(status):
        if status == WaypointStatus.DONE:
            return "DONE"
        elif status == WaypointStatus.QUEUED:
            return "QUEUED"
        elif status == WaypointStatus.IN_PROGRESS:
            return "IN_PROGRESS"
        else:
            return "UNKNOWN"


class Waypoint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.task_uuid = None
        self.status = WaypointStatus.QUEUED

    def __str__(self):
        return f"x:{self.x}, y:{self.y}, angle:{self.orientation}, status:{WaypointStatus.to_string(self.status)}"


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
        self.upload_checklist_client = None
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
        self.telemetry_msg = Odometry()

        # Waypoints
        self.waypoint_current_id = 0
        self.waypoints = []

        # Waypoints from the inspection plan
        self.inspection_waypoints = None

        # Flags
        self.is_inspection_available = False
        self.current_hl_command_id = -1
        self.current_hl_command = np.array([])

    def read_params(self):
        self.telemetry_odom_topic = rospy.get_param(
            "~telemetry_odom_topic", "/base_odom")
        return True

    def init_ros(self):
        # Service servers
        self.command_server = rospy.Service(
            '/command', Command, self.command_server_cb)
        self.inspection_server = rospy.Service(
            '/inspection', InspectionPlan, self.inspection_server_cb)

        # Service clients
        self.upload_alarm_client = rospy.ServiceProxy(
            "/mavsdk_ros/set_upload_alarm", SetUploadAlarm)
        self.upload_waypoint_list_client = rospy.ServiceProxy(
            "/mavsdk_ros/set_upload_waypoint_list", SetUploadWaypointList)
        self.upload_hl_action_client = rospy.ServiceProxy(
            "/mavsdk_ros/set_upload_hl_action", SetUploadHLAction)
        self.upload_checklist_client = rospy.ServiceProxy(
            "/mavsdk_ros/set_upload_checklist", SetUploadChecklist)
        self.update_current_waypoint_item_client = rospy.ServiceProxy(
            "/mavsdk_ros/update_current_waypoint_item", UpdateSeqWaypointItem)
        self.update_reached_waypoint_item_client = rospy.ServiceProxy(
            "/mavsdk_ros/update_reached_waypoint_item", UpdateSeqWaypointItem)

        # Publishers
        self.telemetry_pub = rospy.Publisher(
            "/mavsdk_ros/local_position", Odometry, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/mavsdk_ros/text_status", TextStatus, queue_size=1)
        self.alarm_pub = rospy.Publisher(
            "/mavsdk_ros/alarm_status", AlarmStatus, queue_size=1)

        # Subscribers
        self.odom_sub = rospy.Subscriber(
            self.telemetry_odom_topic, Odometry, self.update_telemetry, queue_size=1)

        # Messages
        self.status_msg = TextStatus()
        self.alarm_msg = AlarmStatus()
        return True

    ################################
    # Checklist
    ###############################
    def upload_checklist(self):
        req = SetUploadChecklistRequest()
        check0 = ChecklistItem()
        check0.description = "Arm is active and routed to the system."
        check0.name = "ARM_ACTIVE"
        check0.index = 0

        check1 = ChecklistItem()
        check1.description = "Arm is an a home configuration and does not hit any part."
        check1.name = "ARM_HOME"
        check1.index = 1

        check2 = ChecklistItem()
        check2.description = "Wheels are inflated."
        check2.name = "WHEELS_CHECKS"
        check2.index = 2

        check3 = ChecklistItem()
        check3.description = "Imu sensor is running."
        check3.name = "IMU_OK"
        check3.index = 3

        check4 = ChecklistItem()
        check4.description = "Lidar sensor is running"
        check4.name = "LIDAR_OK"
        check4.index = 4

        check5 = ChecklistItem()
        check5.description = "Realsense Tracking camera is running and publishing odometry."
        check5.name = "TRACKING_CAM_OK"
        check5.index = 5

        req.checklist.append(check0)
        req.checklist.append(check1)
        req.checklist.append(check2)
        req.checklist.append(check3)
        req.checklist.append(check4)
        req.checklist.append(check5)


        rospy.loginfo("Waiting for checklist service to become available.")
        try:
            self.upload_checklist_client.wait_for_service(timeout=20.0)
        except rospy.ROSException as exc:
            rospy.logwarn(exc)
            rospy.logwarn("Failed to upload the checklist.")
            return False

        self.upload_checklist_client.call(req)
        return True

    ################################
    # Waypoints
    ###############################

    def read_waypoints_from_file(self, file):
        """
        Mainly a debug function to set waypoints from file, otherwise should be received from inspection plan
        See @inspection_server_cb
        """
        with open(file) as stream:
            rospy.loginfo(f"Loading waypoints from file {file}")
            waypoints_list = yaml.load(stream, Loader=yaml.FullLoader)
            if "waypoints" not in waypoints_list.keys():
                return False
            for waypoint in waypoints_list['waypoints']:
                wp = Waypoint()
                wp.x = waypoint['position'][0]
                wp.y = waypoint['position'][1]
                wp.orientation = waypoint['orientation']
                rospy.loginfo(f"Adding waypoint [{wp}]")
                self.waypoints.append(wp)
        return True

    def upload_waypoints(self):
        if len(self.waypoints) == 0:
            rospy.logwarn(
                "No waypoints file found. Skipping upload waypoints list.")
            return

        req = SetUploadWaypointListRequest()
        req.waypoint_list.plan_uuid = str(1)
        req.waypoint_list.sync_id = 0

        for waypoint in self.waypoints:
            item = WaypointItem()
            item.command = 0
            item.task_uuid = "test"
            item.autocontinue = True
            item.x = waypoint.x
            item.y = waypoint.y
            item.z = 0.0
            # there is no specialized field for orientation, we use param 1
            item.param1 = waypoint.orientation
            req.waypoint_list.items.append(item)

        try:
            self.upload_waypoint_list_client.wait_for_service(timeout=10.0)
        except rospy.ROSException as exc:
            rospy.logerr(exc)
            rospy.logerr("Aborting.")
            return False

        self.upload_waypoint_list_client.call(req)
        return True

    def next_waypoint(self):
        if self.waypoint_current_id == len(self.waypoints):
            return None

        current_status = self.waypoints[self.waypoint_current_id].status
        current_wp = self.waypoints[self.waypoint_current_id]

        if current_status == WaypointStatus.IN_PROGRESS:
            return current_wp
        elif current_status == WaypointStatus.QUEUED:
            # update gRCS that the waypoint is the current one
            self.waypoints[self.waypoint_current_id].status = WaypointStatus.IN_PROGRESS
            req = UpdateSeqWaypointItemRequest()
            req.item_seq = self.waypoint_current_id
            self.update_current_waypoint_item_client.call(req)
            return current_wp

    def set_waypoint_done(self):
        req = UpdateSeqWaypointItemRequest()
        req.item_seq = self.waypoint_current_id
        self.update_reached_waypoint_item_client.call(req)

        # set status to done
        self.waypoints[self.waypoint_current_id].status = WaypointStatus.DONE

        # advance to next
        self.waypoint_current_id += 1

    ################################
    # Commands
    ###############################
    def command_server_cb(self, req):
        rospy.loginfo("Received command request from gRCS.")

        # check that the command matches the one specified in the high level actions
        response = CommandResponse()
        response.ack = CommandAck()

        self.current_hl_command = np.array([req.info.param1, req.info.param2, req.info.param3,
                                            req.info.param4, req.info.param5, req.info.param6, req.info.param7])

        rospy.loginfo(
            f"Received command [ID: {req.info.command}], cmd: {self.current_hl_command}")

        if req.info.command in (22, 21):
            self.current_hl_command_id = req.info.command
            response.ack.result = 0  # See MAV_CMD enum
            response.ack.progress = 0
        else:
            rospy.logerr("Unknown command id")
            response.result = 1
            response.progress = 0
        return response

    ################################
    #  Inspection
    ###############################
    def inspection_server_cb(self, req):
        rospy.loginfo("\n\n\nReceived inspection request from gRCS.\n\n\n")
        response = WaypointsAck()
        response.data = 0
        waypoint_list = WaypointList()
        waypoint_list = req.info

        self.inspection_waypoints = waypoint_list
        for i in range(len(waypoint_list.items)):
            wp = Waypoint()
            wp.task_uuid = waypoint_list.items[i].task_uuid
            wp.x = waypoint_list.items[i].x
            wp.y = waypoint_list.items[i].x
            rospy.loginfo(f"Adding waypoint [{wp}]")
            self.waypoints.append(wp)

        self.is_inspection_available = True
        return response

    def print_inspection(self):
        if not self.is_inspection_available:
            rospy.logwarn("Cannot print inspection plan. Not available yet.")
            return

        from mavsdk_ros.msg import WaypointItem
        rospy.loginfo("Inspection plan")
        for i, waypoint in enumerate(self.inspection_waypoints.items):
            rospy.loginfo(
                f"Waypoint {i}: [{waypoint.x}, {waypoint.y}, {waypoint.z}]")

    ################################
    #  High Level Actions
    ###############################

    def upload_hl_action(self):
        hl_manipulation_action = HLActionItem()
        # !!! We need to use a valid enum from the mavlink library
        hl_manipulation_action.command = 22
        hl_manipulation_action.description = "Manipulate valve for pressure regulation."
        hl_manipulation_action.name = "VALVE_MANIPULATION"
        hl_manipulation_action.index = 0

        hl_navigation_action = HLActionItem()
        hl_navigation_action.command = 21
        hl_navigation_action.description = "Autonomous navigation via waypoint-following"
        hl_navigation_action.name = "WAYPOINT_NAVIGATION"
        hl_navigation_action.index = 1

        req = SetUploadHLActionRequest()
        req.hl_actions.append(hl_manipulation_action)
        req.hl_actions.append(hl_navigation_action)

        try:
            self.upload_hl_action_client.wait_for_service(timeout=10.0)
        except rospy.ROSException as exc:
            rospy.logerr(exc)
            rospy.logerr("Aborting.")
            return False

        self.upload_hl_action_client.call(req)
        return True

    def get_current_hl_command(self):
        return self.current_hl_command_id, self.current_hl_command

    ################################
    #  Telemetry
    ###############################
    def update_telemetry(self, msg):
        """ 
        Resend msg received over odom topic to gRCS telemetry topic (relay) 
        """
        self.telemetry_pub.publish(msg)

    ################################
    # Status
    ###############################
    def update_status(self):
        status_msg = TextStatus()
        status_msg.type = TextStatus.INFO
        status_msg.text = "This is an info text message"
        self.status_pub.publish(status_msg)

    ################################
    # Alarms
    ###############################
    def upload_alarms(self):
        req = SetUploadAlarmRequest()
        alarm_sensor = AlarmItem()
        alarm_sensor.index = 0
        alarm_sensor.name = "BAD_SENSOR"
        alarm_sensor.description = "A sensor failed"

        alarm_mission = AlarmItem()
        alarm_mission.index = 1
        alarm_mission.name = "BAD_MISSION"
        alarm_mission.description = "Mission failed to execute. Check logs."

        alarm_waypoint = AlarmItem()
        alarm_waypoint.index = 2
        alarm_waypoint.name = "BAD_WAYPOINT"
        alarm_waypoint.description = "Failed to reach waypoint"

        req.alarms.append(alarm_sensor)
        req.alarms.append(alarm_mission)
        req.alarms.append(alarm_waypoint)

        try:
            self.upload_alarm_client.wait_for_service(timeout=10)
        except rospy.ROSException:
            rospy.logerr("Service {} not available.".format(
                self.upload_alarm_client.resolved_name))

        self.upload_alarm_client.call(req)

    def update_alarm(self):
        alarm = AlarmStatus()
        alarm.status = AlarmStatus.WARNING
        self.alarm_pub.publish(alarm)

    def publish_fake_odometry(self, x=0, vx=0, angle=0, vangle=0):
        self.telemetry_msg.header.frame_id = "world"
        self.telemetry_msg.child_frame_id = "superpanda"
        self.telemetry_msg.pose.pose.position.x = x
        self.telemetry_msg.pose.pose.position.y = 0.5
        self.telemetry_msg.pose.pose.position.z = 0.2

        axis = np.array([1, 1, 1])
        axis = axis / np.linalg.norm(axis)

        self.telemetry_msg.pose.pose.orientation.x = np.sin(
            angle/2.0) * axis[0]
        self.telemetry_msg.pose.pose.orientation.y = np.sin(
            angle/2.0) * axis[1]
        self.telemetry_msg.pose.pose.orientation.z = np.sin(
            angle/2.0) * axis[2]
        self.telemetry_msg.pose.pose.orientation.w = np.cos(angle/2.0)
        self.telemetry_msg.twist.twist.linear.x = vx
        self.telemetry_msg.twist.twist.angular.z = vangle

        self.telemetry_pub.publish(self.telemetry_msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--telemetry", action='store_true',
                        help="run telementry test")
    parser.add_argument("--alarm", action='store_true',
                        help="run alarm test (upload and set)")
    parser.add_argument("--status", action='store_true',
                        help="run status test (set text status)")
    parser.add_argument("--hlaction", action='store_true',
                        help="run high level action test (upload)")
    parser.add_argument("--inspection", action='store_true',
                        help="run inspection test (receive and process)")
    parser.add_argument("--checklist", action='store_true',
                        help="run inspection test (receive and process)")
    

    # neglect ros input arguments
    args, unknown = parser.parse_known_args()

    rospy.init_node("ethz_gcs_bridge")
    bridge = RCSBridge()
    bridge.read_params()
    bridge.init_ros()

    # telemetry test
    if args.telemetry:
        rospy.loginfo("[telemetry test]: Sending telemetry for 100s.")
        start = rospy.get_rostime().to_sec()
        elapsed = 0
        while not rospy.is_shutdown() and elapsed < 10:
            elapsed = rospy.get_rostime().to_sec() - start
            bridge.publish_fake_odometry(
                x=elapsed * 0.05, vx=0.05, angle=elapsed*0.05, vangle=0.05)
            rospy.sleep(0.1)
        rospy.loginfo("[telemetry test]: Done")

    # alarm test
    if args.alarm:
        rospy.loginfo("[alarm test]: Uploading alarm")
        bridge.upload_alarms()
        rospy.sleep(10.0)

        rospy.loginfo("[alarm test]: Setting alarm")
        bridge.update_alarm()
        rospy.sleep(2.0)
        rospy.loginfo("[alarm test]: Done")

    # status test
    if args.status:
        rospy.loginfo("[status test]: Updating the status message")
        bridge.update_status()
        rospy.sleep(2.0)
        rospy.loginfo("[status test]: Done")

    # high level action test
    if args.hlaction:
        rospy.loginfo("[high level action test]: Uploading high level actions")
        if not bridge.upload_hl_action():
            rospy.logerr("Failed to upload high level actions.")
        rospy.loginfo(
            "[high level action test]: Waiting to receive a command... timeout after 30s.")
        start = rospy.get_rostime().to_sec()
        elapsed = 0
        while not rospy.is_shutdown() and elapsed < 1000:
            elapsed = rospy.get_rostime().to_sec() - start
            rospy.sleep(0.1)
        rospy.logwarn("[high level action test]: Timeout elapsed")
        rospy.logwarn("[high level action test]: Done.")

    # inspection test (includes also test of uploading and setting waypoints)
    if args.inspection:
        rospy.loginfo(
            "[insepction test]: Waiting to receive an inspection plan")
        while not rospy.is_shutdown() and not bridge.is_inspection_available:
            rospy.loginfo(
                "[inspection test]: Inspection plan not available yet ...")
            rospy.sleep(1.0)
        rospy.loginfo("[inspection test]: Insepction plan avaialbe!")
        bridge.print_inspection()

        rospy.loginfo("[inspection test]: Reading waypoints from file")
        from os import path
        import rospkg
        rospack = rospkg.RosPack()
        file_path = path.join(rospack.get_path(
            'moma_mission'), 'config', 'state_machine', 'piloting_waypoints.yaml')

        # We just use the waypoints we receive from the gRCS here
        # bridge.read_waypoints_from_file(file_path)
        # else:
        # the previous upload is not istantaneous
        rospy.sleep(5.0)
        rospy.loginfo("[inspection test]: Waypoint successfully loaded")
        waypoint = bridge.next_waypoint()
        while waypoint:
            rospy.loginfo(
                f"[inspection test]: Executing next waypoint at [{waypoint.x}, {waypoint.y}]")
            bridge.set_waypoint_done()
            rospy.loginfo("[inspection test]: set waypoint to DONE.")
            waypoint = bridge.next_waypoint()
            rospy.sleep(5.0)
        rospy.loginfo("[inspection test]: Done.")

    if args.checklist:
        rospy.loginfo("[checklist test]: Updating the checkllist message")
        if not bridge.upload_checklist():
            rospy.loginfo("[checklist test]: Failed to upload the checklist.")
        rospy.sleep(2.0)
        rospy.loginfo("[checklist test]: Done")        
        