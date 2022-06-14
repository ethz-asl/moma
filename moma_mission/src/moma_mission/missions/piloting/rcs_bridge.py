#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import yaml
import threading
import tf
import tf2_ros
import tf2_geometry_msgs

import rospy
import numpy as np
import argparse

from std_msgs.msg import String, Float64MultiArray, UInt32
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, PointCloud2, JointState, Image

from moma_mission.missions.piloting.frames import Frames
from moma_mission.states.waypoint_bridge import (
    TASK_TYPE_POSE_UUID,
    TASK_TYPE_ACTION_VISUAL_UUID,
)

from mavsdk_ros.msg import CommandLong, CommandAck, WaypointList, WaypointsAck
from mavsdk_ros.msg import TextStatus, AlarmStatus, AlarmItem, ChecklistItem
from mavsdk_ros.msg import HLActionItem, WaypointItem
from mavsdk_ros.srv import Command, CommandRequest, CommandResponse
from mavsdk_ros.srv import InspectionPlan, InspectionPlanRequest
from mavsdk_ros.srv import SetUploadAlarm, SetUploadAlarmRequest
from mavsdk_ros.srv import SetUploadChecklist, SetUploadChecklistRequest
from mavsdk_ros.srv import UpdateSeqWaypointItem, UpdateSeqWaypointItemRequest
from mavsdk_ros.srv import SetUploadWaypointList, SetUploadWaypointListRequest
from mavsdk_ros.srv import SetUploadHLAction, SetUploadHLActionRequest
from std_msgs.msg import UInt16


class AlarmWatchdog:
    def __init__(self, alarm_pub, index, topic, type, timeout=1.0):
        self.watchdog_sub = rospy.Subscriber(topic, type, self.update, queue_size=1)
        self.alarm_pub = alarm_pub
        self.index = index
        self.timeout = timeout
        self.timer = None
        self.status = AlarmStatus.OK
        self.warns_count = 0
        self.errors_count = 0
        self.reset()

    def reset(self):
        if self.timer is not None:
            self.timer.cancel()
        self.timer = threading.Timer(self.timeout, self.alarm)
        self.timer.start()

    def check(self, msg):
        return AlarmStatus.OK

    def update(self, msg):
        self.alarm(self.check(msg))

    def alarm(self, status=AlarmStatus.ERROR):
        if self.status == AlarmStatus.OK:
            if status == AlarmStatus.WARNING:
                self.warns_count += 1
            elif status == AlarmStatus.ERROR:
                self.errors_count += 1
        self.status = status

        alarm = AlarmStatus()
        alarm.index = self.index
        alarm.status = self.status
        alarm.warns_count = self.warns_count
        alarm.errors_count = self.errors_count
        self.alarm_pub.publish(alarm)
        self.reset()


class AlarmWatchdogBattery(AlarmWatchdog):
    def check(self, msg: BatteryState):
        if msg.percentage < 0.2:
            return AlarmStatus.ERROR
        elif msg.percentage < 0.3:
            return AlarmStatus.WARNING
        return AlarmStatus.OK


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
        /mavsdk_ros/inspection_set_current --> set to the current waypoint
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

        # Buffers
        self.tf_buffer = None
        self.tf_listener = None
        self.current_photo = None

        # Publishers
        self.telemetry_pub = None
        self.status_pub = None
        self.alarm_pub = None

        self.plan_uuid_pub = None
        self.task_uuid_pub = None
        self.sync_id_pub = None
        self.photos_pub = None

        # Subscribers
        self.odom_sub = None
        self.photos_sub = None

        # Messages
        self.status_msg = TextStatus()
        self.alarm_msg = AlarmStatus()
        self.telemetry_msg = Odometry()

        # Waypoints from the inspection plan
        self.waypoint_requested_id = 0
        self.waypoint_active_id = None
        self.waypoints: WaypointList = None

        # High-level actions
        self.hl_actions = []

        # Flags
        self.paused = True
        self.current_hl_command = None
        self.alarms_uploaded = False

        self.reset_current_hl_command()
        self.reset_waypoints()

    def init_all(self):
        if not self.read_params():
            rospy.logerr("Failed to read gRCS params")
            return False

        if not self.init_ros():
            rospy.logerr("Failed to initialize RCSBridge ros")
            return False

        # self.waypoints_file = self.get_scoped_param("waypoints_file_path")
        # if not gRCS.read_waypoints_from_file(self.waypoints_file):
        #     rospy.logerr("Failed to read waypoints from file")
        #     return "Failure"

        if not self.upload_waypoints():
            rospy.logerr("Failed to upload waypoints.")
            return False

        if not self.upload_hl_actions():
            rospy.logerr("Failed to upload high level actions")
            return False

        if not self.upload_checklist():
            rospy.logerr("Failed to upload checklist")
            return False

        if not self.upload_alarms():
            rospy.logerr("Failed to upload alarms")
            return False

        return True

    def read_params(self):
        self.telemetry_odom_topic = rospy.get_param(
            "~telemetry_odom_topic", "/base_odom"
        )
        return True

    def init_ros(self):
        # Buffers
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Service servers
        self.command_server = rospy.Service("/command", Command, self.command_server_cb)
        self.inspection_server = rospy.Service(
            "/inspection", InspectionPlan, self.download_waypoints_cb
        )

        # Service clients
        self.upload_alarm_client = rospy.ServiceProxy(
            "/mavsdk_ros/set_upload_alarm", SetUploadAlarm
        )
        self.upload_waypoint_list_client = rospy.ServiceProxy(
            "/mavsdk_ros/set_upload_waypoint_list", SetUploadWaypointList
        )
        self.upload_hl_action_client = rospy.ServiceProxy(
            "/mavsdk_ros/set_upload_hl_action", SetUploadHLAction
        )
        self.upload_checklist_client = rospy.ServiceProxy(
            "/mavsdk_ros/set_upload_checklist", SetUploadChecklist
        )
        self.update_current_waypoint_item_client = rospy.ServiceProxy(
            "/mavsdk_ros/update_current_waypoint_item", UpdateSeqWaypointItem
        )
        self.update_reached_waypoint_item_client = rospy.ServiceProxy(
            "/mavsdk_ros/update_reached_waypoint_item", UpdateSeqWaypointItem
        )

        # Publishers
        self.telemetry_pub = rospy.Publisher(
            "/mavsdk_ros/local_position", Odometry, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            "/mavsdk_ros/text_status", TextStatus, queue_size=1
        )
        self.alarm_pub = rospy.Publisher(
            "/mavsdk_ros/alarm_status", AlarmStatus, queue_size=1
        )
        self.plan_uuid_pub = rospy.Publisher(
            "/plan_uuid", String, queue_size=1, latch=True
        )
        self.task_uuid_pub = rospy.Publisher(
            "/task_uuid", String, queue_size=1, latch=True
        )
        self.sync_id_pub = rospy.Publisher("/sync_id", UInt32, queue_size=1, latch=True)
        self.photos_pub = rospy.Publisher(
            "/photos_taken",
            Image,
            queue_size=1,
        )

        # Subscribers
        self.odom_sub = rospy.Subscriber(
            self.telemetry_odom_topic, Odometry, self.update_telemetry, queue_size=1
        )
        self.current_waypoint_sub = rospy.Subscriber(
            "/mavsdk_ros/inspection_set_current",
            UInt16,
            self.download_current_waypoint_cb,
            queue_size=1,
        )
        self.photos_sub = rospy.Subscriber(
            "/hand_eye/color/image_raw",
            Image,
            self._photo_msg,
            queue_size=1,
        )

        # Messages
        self.status_msg = TextStatus()
        self.alarm_msg = AlarmStatus()
        return True

    ################################
    # Checklist
    ###############################
    def upload_checklist(self):
        """
        Set of checklist items that are set on the gRCS to help the user to remember what should be checked before the
        beginning of a mission. It make sense (but it is not enforced) that each checklist item could be verified
        looking at the status of the alarm list.
        Example:
        - One checklist item is ARM_ACTIVE -> make sure the arm is active
        - We might have an alarm that based on the ping of the arm ip sets the level of alarm ARM_OK
        and so forth...
        """
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
        check5.description = (
            "Realsense Tracking camera is running and publishing odometry."
        )
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
    # Inspection / Waypoints
    ###############################
    @property
    def is_waypoints_available(self):
        return self.waypoints is not None and len(self.waypoints.items) > 0

    @property
    def is_continuable(self):
        return (
            self.is_waypoints_available
            and not self.paused
            and self.waypoint_requested_id < len(self.waypoints.items)
        )

    @property
    def waypoint_requested(self) -> WaypointItem:
        return self.waypoints.items[self.waypoint_requested_id]

    @property
    def waypoint_active(self) -> WaypointItem:
        return self.waypoints.items[self.waypoint_active_id]

    def read_waypoints_from_file(self, file):
        """
        Mainly a debug function to set waypoints from file, otherwise should be received from inspection plan
        See @inspection_server_cb
        """
        with open(file) as stream:
            rospy.loginfo(f"Loading waypoints from file {file}")
            self.waypoints = []
            waypoints_list = yaml.load(stream, Loader=yaml.FullLoader)
            if "waypoints" not in waypoints_list.keys():
                return False
            for waypoint in waypoints_list["waypoints"]:
                wp = Waypoint()
                wp.x = waypoint["position"][0]
                wp.y = waypoint["position"][1]
                wp.orientation = waypoint["orientation"]
                rospy.loginfo(f"Adding waypoint [{wp}]")
                self.waypoints.append(wp)
        return True

    def download_waypoints_cb(self, req: InspectionPlan):
        """
        Workflow:
        1. We receive an inspection plan from the gRCS. This has associated plan, task and sync ids.
        2. We save it locally in the list of waypoints to follow.
        3. Note that to each waypoint is associated two types of commands
            - MAV_CMD_NAV_WAYPOINT_QUATERNION: this tell us the waypoint is a simple navigate-to waypoint
            - MAV_CMD_NAV_INSP_POINT_QUATERNION: this tell us the waypoint is a location for some sort of action (e.g manipulation)
        4. We save the type of waypoint so that we can use it in the logic of the state machine TODO
        5. We upload the waypoints list, sending it to mavsdk_ros (running locally) which will save it.
        N.B. Point 5 allows to recover from the following situation:
        - gRCS disconnect from the system
        - instead of resending in the inspection plan issues an upload waypoints list
        - since the mavsdk_ros saved the waypoints list from the previous run (thanks to point 5) it can send back the old one
        - gRCS is able to resume the previous inspection plan
        """

        rospy.loginfo("Received inspection request with waypoints from gRCS.")
        self.reset_waypoints()
        self.waypoints = req.info

        response = WaypointsAck()
        response.data = 0

        plan_uuid = String()
        plan_uuid.data = req.info.plan_uuid
        self.plan_uuid_pub.publish(plan_uuid)
        sync_id = UInt32()
        sync_id.data = req.info.sync_id
        self.sync_id_pub.publish(sync_id)

        self.fix_waypoints()
        self.upload_waypoints()
        return response

    def upload_waypoints(self):
        """
        Upload waypoints is a method to send a updated waypoints list to the mavsdk_ros client. This does not
        mean that the gRCS will immediately see a new plan. Instead it means, that IF the gRCS issues a
        update_waypoint_list command, the mavsdk_ros will be able to send this new updated plan back.
        """
        if not self.is_waypoints_available:
            rospy.logwarn("No waypoints found. Skipping upload waypoints list.")
            return True

        req = SetUploadWaypointListRequest()
        req.waypoint_list = self.waypoints

        try:
            self.upload_waypoint_list_client.wait_for_service(timeout=10.0)
        except rospy.ROSException as exc:
            rospy.logerr(exc)
            rospy.logerr("Aborting while waiting for waypoint upload service.")
            return False

        rospy.loginfo(
            f"Uploading waypoints for plan {self.waypoints.plan_uuid} and sync id {self.waypoints.sync_id}"
        )
        self.upload_waypoint_list_client.call(req)
        return True

    def fix_waypoints(self):
        for waypoint in self.waypoints.items:
            rospy.loginfo(f"Received waypoint {waypoint}")

            # action vs navigation
            # wp.command of value MAV_CMD_NAV_WAYPOINT_QUATERNION or MAV_CMD_NAV_INSP_POINT_QUATERNION

            # Fix the pose waypoints for our robot
            if waypoint.task_type_uuid == TASK_TYPE_POSE_UUID:
                waypoint.z = 0

            # TODO Remove this hack as soon as we can adjust height of ACTION waypoints in gRCS
            if waypoint.task_type_uuid == TASK_TYPE_ACTION_VISUAL_UUID:
                rospy.logwarn("Applying hack on action waypoint height.")
                waypoint.z = 0.5

            explicit_quat = [
                waypoint.param2,
                waypoint.param3,
                waypoint.param4,
                waypoint.param1,
            ]
            roll_rad, pitch_rad, yaw_rad = tf.transformations.euler_from_quaternion(
                explicit_quat
            )
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_rad)
            waypoint.param2 = quaternion[0]  # x
            waypoint.param3 = quaternion[1]  # y
            waypoint.param4 = quaternion[2]  # z
            waypoint.param1 = quaternion[3]  # w

    def reset_waypoints(self):
        self.waypoint_requested_id = 0
        self.waypoint_active_id = None
        self.waypoints = None
        self.paused = True

    def download_current_waypoint_cb(self, msg):
        rospy.loginfo(f"Received request to set current waypoint to {msg.data}.")
        self.waypoint_requested_id = msg.data
        self.paused = False

    def upload_current_waypoint(self):
        """Send the requested waypoint and mark it as active internally"""
        req = UpdateSeqWaypointItemRequest()
        req.item_seq = self.waypoint_requested_id
        self.update_current_waypoint_item_client.call(req)
        self.waypoint_active_id = self.waypoint_requested_id

    def upload_reached_waypoint(self):
        """Mark the active waypoint as reached"""
        req = UpdateSeqWaypointItemRequest()
        req.item_seq = self.waypoint_active_id
        self.update_reached_waypoint_item_client.call(req)

    def get_next_waypoint(self):
        if not self.is_waypoints_available:
            rospy.logwarn("No waypoints found. Can't return next waypoint.")
            return None

        assert self.is_continuable

        task_uuid = String()
        task_uuid.data = self.waypoint_requested.task_uuid
        self.task_uuid_pub.publish(task_uuid)

        self.upload_current_waypoint()
        rospy.loginfo(f"Current waypoint is {self.waypoint_requested}.")
        return self.waypoint_requested

    def set_waypoint_reached(self):
        if self.waypoint_active_id is None:
            rospy.logerr(
                "Failed setting waypoint as reached. No waypoint is currently active."
            )
            return "Failure"

        rospy.loginfo("Successfully reached waypoint.")
        self.upload_reached_waypoint()

        if not self.waypoint_requested.autocontinue:
            self.paused = True
        paused = self.paused

        # advance to next if no other waypoint was requested in the meantime
        if self.waypoint_requested_id == self.waypoint_active_id:
            self.waypoint_requested_id += 1

        # No more active waypoint
        self.waypoint_active_id = None

        if self.waypoint_requested_id == len(self.waypoints.items):
            rospy.loginfo("All waypoints reached. Inspection completed.")
            # self.reset_waypoints()
            if not paused:
                return "Completed"

        return "Next"

    def print_inspection(self):
        if not self.is_waypoints_available:
            rospy.logwarn("Cannot print inspection plan. No waypoints available yet.")
            return

        rospy.loginfo("Inspection plan")
        for i, waypoint in enumerate(self.waypoints.items):
            rospy.loginfo(f"Waypoint {i}: [{waypoint.x}, {waypoint.y}, {waypoint.z}]")

    ################################
    # Commands
    ###############################
    def command_server_cb(self, req):
        """
        We receive commands and we check that this match with one of the available commands.
        """
        rospy.loginfo("Received command request from gRCS.")

        # check that the command matches the one specified in the high level actions
        response = CommandResponse()
        response.ack = CommandAck()

        params = [
            req.info.param1,
            req.info.param2,
            req.info.param3,
            req.info.param4,
            req.info.param5,
            req.info.param6,
            req.info.param7,
        ]

        rospy.loginfo(f"Received command [ID: {req.info.command}], params: {params}")

        response.ack.result = 0  # See MAV_CMD enum
        response.ack.progress = 0

        if any(req.info.command == hl_action.command for hl_action in self.hl_actions):
            command = self.get_hl_command_name(req.info)
            if command == "PAUSE_CONTINUE" and req.info.param1 == 0:
                rospy.loginfo("Pausing mission on request.")
                self.paused = True
            elif command == "PAUSE_CONTINUE" and req.info.param1 == 1:
                rospy.loginfo("Continuing mission on request.")
                self.paused = False
            elif command == "TAKE_PHOTO":
                rospy.loginfo("Taking a photo...")
                if self.current_photo is None:
                    rospy.logerr(
                        "Failure taking a photo, no image data was received yet."
                    )
                else:
                    self.photos_pub.publish(self.current_photo)
            else:
                # Let the state machine handle the command
                rospy.loginfo("Command will be handled by state machine later on.")
                self.current_hl_command = req.info
        elif req.info.command == 42001:  # HOME command
            # Overwrite the current waypoint list with the homing waypoint
            waypoint = WaypointItem()
            # Position
            waypoint.x = req.info.param5
            waypoint.y = req.info.param6
            waypoint.z = req.info.param7
            # Orientation
            # BUG in gRCS
            waypoint.param1 = req.info.param1  # w
            waypoint.param4 = req.info.param3  # z

            self.reset_waypoints()
            self.waypoints = WaypointList()
            self.waypoints.items.append(waypoint)

            self.fix_waypoints()

            # Uploading does not make sense since there is no plan and sync id for the homing operation
            # self.upload_waypoints()

            self.paused = False
        else:
            rospy.logerr("Unknown command id")
            response.result = 1
            response.progress = 0
        return response

    ################################
    #  High Level Actions
    ###############################
    def upload_hl_actions(self):
        """
        TODO need to use the proper MAV_CMD enums and give a corresponding meaning to their parameters
        """
        req = SetUploadHLActionRequest()

        hl_action = HLActionItem()
        # !!! We need to use a valid enum from the mavlink library
        # https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PAUSE_CONTINUE
        hl_action.command = 193
        hl_action.description = "Hold the current position or continue."
        hl_action.name = "PAUSE_CONTINUE"
        hl_action.index = 0
        req.hl_actions.append(hl_action)

        hl_action = HLActionItem()
        hl_action.command = 42004
        hl_action.description = "Take a photo at the current location."
        hl_action.name = "TAKE_PHOTO"
        hl_action.index = 1
        req.hl_actions.append(hl_action)

        hl_action = HLActionItem()
        hl_action.command = 42005
        hl_action.description = "Manipulate a valve."
        hl_action.name = "MANIPULATE_VALVE"
        hl_action.index = 2
        req.hl_actions.append(hl_action)

        hl_action = HLActionItem()
        hl_action.command = 42007
        hl_action.description = "Place an object."
        hl_action.name = "PLACE_OBJECT"
        hl_action.index = 3
        req.hl_actions.append(hl_action)

        # hl_action = HLActionItem()
        # hl_action.command = 42001
        # hl_action.description = "Go to home position."
        # hl_action.name = "HOME"
        # hl_action.index = 2
        # req.hl_actions.append(hl_action)

        try:
            self.upload_hl_action_client.wait_for_service(timeout=10.0)
        except rospy.ROSException as exc:
            rospy.logerr(exc)
            rospy.logerr("Aborting.")
            return False

        self.upload_hl_action_client.call(req)
        self.hl_actions = req.hl_actions
        return True

    def reset_current_hl_command(self):
        self.current_hl_command = None

    def get_hl_command_name(self, hl_command):
        return [
            hl_action.name
            for hl_action in self.hl_actions
            if hl_action.command == hl_command.command
        ][0]

    def get_current_hl_command(self):
        current_hl_command = self.current_hl_command
        if current_hl_command is None:
            return (None, None)
        self.reset_current_hl_command()
        return self.get_hl_command_name(current_hl_command), current_hl_command

    def photo_cb(self, msg: Image):
        self.current_photo = msg

    ################################
    #  Telemetry
    ###############################
    def update_telemetry(self, msg: Odometry):
        """
        Resend msg received over odom topic to gRCS telemetry topic (relay)
        """
        # Odometry parent frame_id needs to be map, mavlink does not lookup the transform internally
        try:
            map_frame = Frames.map_frame
            transform = self.tf_buffer.lookup_transform(
                map_frame,
                # source frame:
                msg.header.frame_id,
                # get the tf at the time the pose was valid
                # msg.header.stamp  # BUG not working properly
                rospy.Time(0),
            )  # But it is also OK, since the TF between map and tracking_camera_odom should be constant
            # Note that covariance is not properly transformed
            msg.header.frame_id = map_frame
            msg.pose.pose = tf2_geometry_msgs.do_transform_pose(
                msg.pose, transform
            ).pose
            twist_linear = Vector3Stamped()
            twist_linear.vector = msg.twist.twist.linear
            msg.twist.twist.linear = tf2_geometry_msgs.do_transform_vector3(
                twist_linear, transform
            ).vector
            twist_angular = Vector3Stamped()
            twist_angular.vector = msg.twist.twist.angular
            msg.twist.twist.angular = tf2_geometry_msgs.do_transform_vector3(
                twist_angular, transform
            ).vector
            self.telemetry_pub.publish(msg)
        except tf.ConnectivityException:
            rospy.logwarn_throttle(3, "Telemetry transform to map is not known yet.")
        except tf.ExtrapolationException:
            rospy.logwarn_throttle(
                3,
                "Telemetry could not extrapolate transform. Maybe the gRCS is slow and does not parse messages quickly enough for the tf_buffer to suffice.",
            )

    ################################
    # Status
    ###############################
    def update_status(self):
        """
        Whatever is published as status is printed to the consolve in the visualization portal. We
        can use this function as a logging method.
        """
        status_msg = TextStatus()
        status_msg.type = TextStatus.INFO
        status_msg.text = "Hello World!"
        self.status_pub.publish(status_msg)

    ################################
    # Alarms
    ###############################
    def upload_alarms(self):
        """
        Alarm should be set continuously and they are meant to be in a 'OK' state most of the time.
        This is executed only once, to inform the gRCS what is the list of possible alarms that can
        be triggered during the execution of a mission.
        """
        # Only allow once as the alarm watchdogs are registered here as well
        if self.alarms_uploaded:
            rospy.logwarn("Alarms were already uploaded before.")
            return True
        self.alarms_uploaded = True

        req = SetUploadAlarmRequest()
        alarm = AlarmItem()
        alarm.index = 0
        alarm.name = "BATTERY"
        alarm.description = "Battery condition."
        AlarmWatchdogBattery(
            self.alarm_pub, 0, "/smb/base_battery_state", BatteryState, 2.0
        )
        req.alarms.append(alarm)

        alarm = AlarmItem()
        alarm.index = 1
        alarm.name = "LIDAR"
        alarm.description = "LIDAR returning data."
        AlarmWatchdog(self.alarm_pub, 1, "/icp_node/registered_cloud", PointCloud2)
        req.alarms.append(alarm)

        alarm = AlarmItem()
        alarm.index = 2
        alarm.name = "IMU"
        alarm.description = "IMU returning data."
        AlarmWatchdog(self.alarm_pub, 2, "/versavis/imu", Imu)
        req.alarms.append(alarm)

        alarm = AlarmItem()
        alarm.index = 3
        alarm.name = "ODOMETRY"
        alarm.description = "Odometry returning data."
        AlarmWatchdog(self.alarm_pub, 3, "/smb/smb_diff_drive/odom", Odometry)
        req.alarms.append(alarm)

        alarm = AlarmItem()
        alarm.index = 4
        alarm.name = "TRACKING"
        alarm.description = "Tracking camera data."
        AlarmWatchdog(
            self.alarm_pub, 4, "/base_odom", Odometry
        )  # or /camera/odom/sample
        req.alarms.append(alarm)

        alarm = AlarmItem()
        alarm.index = 5
        alarm.name = "MOTORS"
        alarm.description = "Motor data."
        AlarmWatchdog(self.alarm_pub, 5, "/wheelSpeeds", Float64MultiArray)
        req.alarms.append(alarm)

        alarm = AlarmItem()
        alarm.index = 6
        alarm.name = "ARM"
        alarm.description = "Robot arm returning data."
        AlarmWatchdog(
            self.alarm_pub, 6, "/panda/franka_state_controller/joint_states", JointState
        )
        req.alarms.append(alarm)

        alarm = AlarmItem()
        alarm.index = 7
        alarm.name = "CAMERA"
        alarm.description = "Camera image data."
        AlarmWatchdog(self.alarm_pub, 7, "/hand_eye/color/image_raw", Image, 4.0)
        req.alarms.append(alarm)

        try:
            self.upload_alarm_client.wait_for_service(timeout=10)
        except rospy.ROSException:
            rospy.logerr(
                "Service {} not available.".format(
                    self.upload_alarm_client.resolved_name
                )
            )
            return False

        self.upload_alarm_client.call(req)
        return True

    def update_alarm(self):
        """
        Mainly to tell the state of the sensors of each sensor.  This should run
        in a separate thread to make sure that everything is running ok.
        Example implementation: we subscribe to lidar pcl and if none is received for more than dt seconds
        then we trigger the alarm. Note that we also need a static variable keeping track of the
        errors and warnings counts.
        """

        # TODO stub implementation for communication testing during gRCS integration week
        alarm = AlarmStatus()
        alarm.index = 0
        alarm.status = AlarmStatus.OK
        self.alarm_pub.publish(alarm)
        rospy.sleep(1.0)

        alarm = AlarmStatus()
        alarm.index = 1
        alarm.status = AlarmStatus.OK
        self.alarm_pub.publish(alarm)
        rospy.sleep(1.0)

        alarm = AlarmStatus()
        alarm.index = 2
        alarm.status = AlarmStatus.OK
        self.alarm_pub.publish(alarm)
        rospy.sleep(1.0)

    def test_alarm(self, idx):
        """Stub implementation to test alarm during gRCS integration week"""
        alarm = AlarmStatus()
        alarm.index = 0
        alarm.warns_count = idx
        alarm.status = AlarmStatus.WARNING
        self.alarm_pub.publish(alarm)
        rospy.sleep(1.0)

        alarm = AlarmStatus()
        alarm.index = 1
        alarm.errors_count = idx
        alarm.status = AlarmStatus.ERROR
        self.alarm_pub.publish(alarm)

    def publish_fake_odometry(self, x=0, vx=0, angle=0, vangle=0):
        """
        This should be in map frame: the same frame as the one used to
        visualize the global map (pcl) in the gRCS visualization portal
        """
        self.telemetry_msg.header.frame_id = "world"
        self.telemetry_msg.child_frame_id = "superpanda"
        self.telemetry_msg.pose.pose.position.x = x
        self.telemetry_msg.pose.pose.position.y = 0.5
        self.telemetry_msg.pose.pose.position.z = 0.2

        axis = np.array([1, 1, 1])
        axis = axis / np.linalg.norm(axis)

        self.telemetry_msg.pose.pose.orientation.x = np.sin(angle / 2.0) * axis[0]
        self.telemetry_msg.pose.pose.orientation.y = np.sin(angle / 2.0) * axis[1]
        self.telemetry_msg.pose.pose.orientation.z = np.sin(angle / 2.0) * axis[2]
        self.telemetry_msg.pose.pose.orientation.w = np.cos(angle / 2.0)
        self.telemetry_msg.twist.twist.linear.x = vx
        self.telemetry_msg.twist.twist.angular.x = vangle * axis[0]
        self.telemetry_msg.twist.twist.angular.y = vangle * axis[1]
        self.telemetry_msg.twist.twist.angular.z = vangle * axis[2]

        self.telemetry_pub.publish(self.telemetry_msg)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--telemetry", action="store_true", help="run telementry test")
    parser.add_argument(
        "--alarm", action="store_true", help="run alarm test (upload and set)"
    )
    parser.add_argument(
        "--status", action="store_true", help="run status test (set text status)"
    )
    parser.add_argument(
        "--hlaction", action="store_true", help="run high level action test (upload)"
    )
    parser.add_argument(
        "--inspection",
        action="store_true",
        help="run inspection test (receive and process)",
    )
    parser.add_argument(
        "--checklist",
        action="store_true",
        help="run inspection test (receive and process)",
    )

    # neglect ros input arguments
    args, unknown = parser.parse_known_args()

    rospy.init_node("ethz_grcs_bridge")
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
                x=elapsed * 0.05, vx=0.05, angle=elapsed * 0.05, vangle=0.05
            )
            rospy.sleep(0.1)
        rospy.loginfo("[telemetry test]: Done")

    # alarm test
    if args.alarm:
        rospy.loginfo("[alarm test]: Uploading alarm")
        bridge.upload_alarms()
        rospy.sleep(10.0)

        rospy.loginfo("[alarm test]: Setting alarm")
        for _ in range(10):
            bridge.update_alarm()
            rospy.sleep(1.0)
        bridge.test_alarm(1)
        rospy.logerr(2.0)
        bridge.test_alarm(2)
        rospy.logerr(2.0)

        rospy.loginfo("[alarm test]: Done")

    # status test
    if args.status:
        rospy.loginfo("[status test]: Updating the status message")
        # Wait for the mavsdk ros to start
        rospy.sleep(5.0)
        bridge.update_status()
        rospy.sleep(2.0)
        rospy.loginfo("[status test]: Done")

    # high level action test
    if args.hlaction:
        rospy.loginfo("[high level action test]: Uploading high level actions")
        if not bridge.upload_hl_actions():
            rospy.logerr("Failed to upload high level actions.")
        rospy.loginfo(
            "[high level action test]: Waiting to receive a command... timeout after 30s."
        )
        start = rospy.get_rostime().to_sec()
        elapsed = 0
        while not rospy.is_shutdown() and elapsed < 1000:
            elapsed = rospy.get_rostime().to_sec() - start
            rospy.sleep(0.1)
        rospy.logwarn("[high level action test]: Timeout elapsed")
        rospy.logwarn("[high level action test]: Done.")

    # inspection test (includes also test of uploading and setting waypoints)
    if args.inspection:

        rospy.loginfo("[insepction test]: Waiting to receive an inspection plan")
        while not rospy.is_shutdown() and not bridge.is_waypoints_available:
            rospy.loginfo("[inspection test]: Inspection plan not available yet ...")
            rospy.sleep(1.0)
        rospy.loginfo("[inspection test]: Insepction plan avaialbe!")
        bridge.print_inspection()

        rospy.loginfo("[inspection test]: Reading waypoints from file")
        from os import path
        import rospkg

        rospack = rospkg.RosPack()
        file_path = path.join(
            rospack.get_path("moma_mission"),
            "config",
            "state_machine",
            "piloting_waypoints.yaml",
        )

        # We just use the waypoints we receive from the gRCS here
        # bridge.read_waypoints_from_file(file_path)
        # else:
        # the previous upload is not istantaneous
        rospy.sleep(5.0)
        rospy.loginfo("[inspection test]: Waypoint successfully loaded")
        waypoint = bridge.next_waypoint()
        while waypoint:
            rospy.loginfo(
                f"[inspection test]: Executing next waypoint at [{waypoint.x}, {waypoint.y}]"
            )
            rospy.sleep(5.0)
            bridge.set_waypoint_done()
            rospy.loginfo("[inspection test]: set waypoint to DONE.")
            waypoint = bridge.next_waypoint()
            rospy.sleep(5.0)
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
        rospy.loginfo("[inspection test]: Done.")

    if args.checklist:
        rospy.loginfo("[checklist test]: Updating the checkllist message")
        if not bridge.upload_checklist():
            rospy.loginfo("[checklist test]: Failed to upload the checklist.")
        rospy.sleep(2.0)
        rospy.loginfo("[checklist test]: Done")
