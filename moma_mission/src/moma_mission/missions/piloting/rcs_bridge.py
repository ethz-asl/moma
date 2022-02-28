#!/usr/bin/env python
# -*- coding: utf-8 -*-
import yaml

import rospy
import numpy as np
import argparse

from std_msgs.msg import UInt16
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, PoseStamped

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

qfrom moma_mission.utils.rotation import CompatibleRotation

class WaypointStatus:
    """ A simple object to represent the current waypoint status """
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
    """ 
    A 2D waypoint navigation object. Each waypoint is associated with a 
    task uuid and an execution status @WaypointStatus
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.task_uuid = None
        self.status = WaypointStatus.QUEUED

    def __str__(self):
        return f"x:{self.x}, y:{self.y}, angle:{self.orientation}, status:{WaypointStatus.to_string(self.status)}"

    def to_pose_ros(self, frame):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = rospy.get_rostime()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        q = CompatibleRotation.from_euler('z', self.orientation, degrees=False).as_quat()
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
    
    @staticmethod
    def from_pose_ros(pose: Pose):
        wp = Waypoint()
        wp.x = pose.position.x
        wp.y = pose.position.y
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        ix = CompatibleRotation.from_quat(q).as_matrix() @ np.array([1.0, 0.0, 0.0]) # projection of the xaxis
        wp.orientation = np.arctan2(ix[1], ix[0])
        return wp

class ManualWaypointsSelector:
    """
    Utility class to retrieve waypoints from Rviz using the 2D Nav Goal button
    (note this requires a custom RVIz that publish nav_goal under /waypoint)
    """
    def __init__(self):
        self.add_pose_topic = "/waypoint"
        self.pose_array_topic = "/waypoints" 
        self.pose_array_publisher = rospy.Publisher(self.pose_array_topic, PoseArray, queue_size=1)
        self.path_reset_srv = rospy.Service("/path_reset", Trigger, self.path_reset_callback)
        self.path_point_sub = rospy.Subscriber(self.add_pose_topic, PoseStamped, self.path_pose_callback)
        self.waypoints = []
        self.waypoints_ros = []
        self.path_ready = False

    def path_reset_callback(self, req):
        rospy.loginfo('Received path RESET trigger')
        self.initialize_path_queue()

    def initialize_path_queue(self):
        self.waypoints = []  
        self.waypoints_ros = []
        self.path_ready = False
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"  # it does not matter, is just an empty path
        self.pose_array_publisher.publish(pose_array)

    def publish_pose_array(self, frame):
        pose_array = PoseArray()
        pose_array.header.frame_id = frame
        pose_array.header.stamp = rospy.get_rostime()
        pose_array.poses = self.waypoints_ros
        self.pose_array_publisher.publish(pose_array)
        rospy.sleep(1.0)

    def path_pose_callback(self, msg: PoseStamped):
        
        curr_wp = Waypoint.from_pose_ros(msg.pose)
        if len(self.waypoints) > 0:
            prev_wp = self.waypoints[-1]
            dist = (curr_wp.x - prev_wp.x)**2 + (curr_wp.y - prev_wp.y)**2
            if dist < 0.01:
                rospy.loginfo("Skipping waypoint... too close -> detecting that path is complete!")
                self.path_ready = True
                return

        rospy.loginfo("Received new waypoint")
        self.waypoints.append(curr_wp)
        self.waypoints_ros.append(msg.pose)
        self.publish_pose_array(frame=msg.header.frame_id)
        rospy.sleep(1.0)

        
    def get_waypoints(self):
        rospy.loginfo(f"Waiting to recieve waypoints via Pose msg on topic {self.add_pose_topic}")
        rospy.loginfo("To send a path, click on two close waypoints (last will be skipped). '")

        while not self.path_ready:
            rospy.logwarn_throttle(5.0, "Waiting for the next waypoint.")
            rospy.sleep(1.0)
        return self.waypoints

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
        self.waypoints_selector = None

        # Waypoints from the inspection plan
        self.inspection_waypoints = None

        # Flags
        self.is_inspection_available = False
        self.current_hl_command_id = 1
        self.current_hl_command = np.array([])


        self.plan_uuid = None
        self.sync_id = None

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
        self.current_waypoint = rospy.Subscriber(
            "/mavsdk_ros/inspection_set_current", UInt16, self.set_current, queue_size=1)

        # Utilities
        self.waypoints_selector = ManualWaypointsSelector()

        # Messages
        self.status_msg = TextStatus()
        self.alarm_msg = AlarmStatus()
        return True

    def set_current(self, msg):
        rospy.loginfo(f"Set command to {msg.data}")

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
        self.waypoints = []
        with open(file, 'r') as stream:
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
    
    def read_waypoints_from_user(self):
        self.waypoints = self.waypoints_selector.get_waypoints()

    def upload_waypoints(self):
        """
        Upload waypoints is a method to send a updated waypoints list to the mavsdk_ros client. This does not
        mean that the gRCS will immediately see a new plan. Instead it means, that IF the gRCS issues a 
        update_waypoint_list command, the mavsdk_ros will be able to send this new updated plan back. 
        """
        if len(self.waypoints) == 0:
            rospy.logwarn(
                "No waypoints file found. Skipping upload waypoints list.")
            return

        req = SetUploadWaypointListRequest()
        req.waypoint_list.plan_uuid = self.plan_uuid
        req.waypoint_list.sync_id = self.sync_id

        for waypoint in self.waypoints:
            item = WaypointItem()
            item.command = 0
            item.task_uuid = waypoint.task_uuid
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

        rospy.loginfo(f"Uploading wayponits for plan {self.plan_uuid} and sync id {self.sync_id}")
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
            
            try:
                self.update_current_waypoint_item_client.call(req)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                rospy.logwarn("Failed to set waypoint status")

            return current_wp

    def set_waypoint_done(self):
        req = UpdateSeqWaypointItemRequest()
        req.item_seq = self.waypoint_current_id

        try:        
            self.update_reached_waypoint_item_client.call(req)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("Failed to set waypoint status")

        # set status to done
        self.waypoints[self.waypoint_current_id].status = WaypointStatus.DONE

        # advance to next
        self.waypoint_current_id += 1

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
        
        # 1.
        rospy.loginfo("Received inspection request from gRCS.")
        self.plan_uuid = req.info.plan_uuid
        self.sync_id = req.info.sync_id

        response = WaypointsAck()
        response.data = 0
        waypoint_list = WaypointList()
        waypoint_list = req.info

        # 2.
        self.waypoints = []
        self.inspection_waypoints = waypoint_list
        for i in range(len(waypoint_list.items)):
            # TODO 3. and 4. 
            # TODO  the waypoint contains a command to the actual thing to be done at that waypoint
            # action vs navigation
            # wp = WaypointItem()
            # wp.command of value MAV_CMD_NAV_WAYPOINT_QUATERNION or  MAV_CMD_NAV_INSP_POINT_QUATERNION
            wp = Waypoint()
            wp.task_uuid = waypoint_list.items[i].task_uuid
            wp.x = waypoint_list.items[i].x
            wp.y = waypoint_list.items[i].y
            rospy.loginfo(f"Adding waypoint [{wp}]")
            self.waypoints.append(wp)
        
        # 5. 
        self.upload_waypoints()
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
        """
        TODO need to use the proper MAV_CMD enums and give a corresponding meaning to their parameters
        """
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
        # need to be map
        try: 
            self.telemetry_pub.publish(msg)
        except rospy.ROSException as exc:
            pass # when state mchine closes

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
        req = SetUploadAlarmRequest()
        alarm_sensor = AlarmItem()
        alarm_sensor.index = 0
        alarm_sensor.name = "LIDAR_OK"
        alarm_sensor.description = "A sensor failed"

        alarm_mission = AlarmItem()
        alarm_mission.index = 1
        alarm_mission.name = "MISSION_OK"
        alarm_mission.description = "Mission failed to execute. Check logs."

        alarm_waypoint = AlarmItem()
        alarm_waypoint.index = 2
        alarm_waypoint.name = "WAYPOINT_OK"
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
        """ 
        Mainly to tell the state of the sensors of each sensor.  This should run 
        in a separate thread to make sure that everything is running ok.
        Example implementation: we subscribe to lidar pcl and if none is received for more than dt seconds
        then we trigger the alarm. Note that we also need a static variable keeping track of the
        errors and warnings counts. 
        """
        
        # TODO stub implementation for communication testing during gRCS integration week
        alarm = AlarmStatus()
        alarm.index  = 0
        alarm.status = AlarmStatus.OK
        self.alarm_pub.publish(alarm)
        rospy.sleep(1.0)

        alarm = AlarmStatus()
        alarm.index  = 1
        alarm.status = AlarmStatus.OK
        self.alarm_pub.publish(alarm)
        rospy.sleep(1.0)
        
        alarm = AlarmStatus()
        alarm.index  = 2
        alarm.status = AlarmStatus.OK
        self.alarm_pub.publish(alarm)
        rospy.sleep(1.0)

    def test_alarm(self, idx):
        """ Stub implementation to test alarm during gRCS integration week """
        alarm = AlarmStatus()
        alarm.index  = 0
        alarm.warns_count = idx
        alarm.status = AlarmStatus.WARNING
        self.alarm_pub.publish(alarm)
        rospy.sleep(1.0)

        alarm = AlarmStatus()
        alarm.index  = 1
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

        self.telemetry_msg.pose.pose.orientation.x = np.sin(
            angle/2.0) * axis[0]
        self.telemetry_msg.pose.pose.orientation.y = np.sin(
            angle/2.0) * axis[1]
        self.telemetry_msg.pose.pose.orientation.z = np.sin(
            angle/2.0) * axis[2]
        self.telemetry_msg.pose.pose.orientation.w = np.cos(angle/2.0)
        self.telemetry_msg.twist.twist.linear.x = vx
        self.telemetry_msg.twist.twist.angular.x = vangle * axis[0]
        self.telemetry_msg.twist.twist.angular.y = vangle * axis[1]
        self.telemetry_msg.twist.twist.angular.z = vangle * axis[2]

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
