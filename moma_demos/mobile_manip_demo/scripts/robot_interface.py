"""Define ROS bindings for mobile manipulation task."""

from typing import Tuple
import rospy

import numpy as np
from numpy import linalg as LA
import math

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, SetBool, SetBoolRequest
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped

from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates

# Action lib stuff
from actionlib import SimpleActionClient
from actionlib import simple_action_client
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from moveit_msgs.msg import MoveItErrorCodes

moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == "_":
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


class MoveSkill:
    """Low level implementation of a Navigation skill."""

    def __init__(self) -> None:
        """Initialize ROS nodes."""
        self.move_client = SimpleActionClient("move_client", MoveBaseAction)
        self.move_client.wait_for_server(rospy.Duration(100))

    def initialize_navigation(self, goal_ID: str) -> None:
        """Move the robot to the target pose."""
        target_goal, ref_frame = get_goal_from_ID(goal_ID)
        target_pose, target_orientation = target_goal
        # command
        goal_ = MoveBaseGoal()
        goal_.target_pose.header.frame_id = ref_frame
        goal_.target_pose.header.stamp = rospy.Time.now()
        goal_.target_pose.pose.position.x = target_pose[0]
        goal_.target_pose.pose.position.y = target_pose[1]
        goal_.target_pose.pose.position.z = target_pose[2]
        goal_.target_pose.pose.orientation.x = target_orientation[0]
        goal_.target_pose.pose.orientation.y = target_orientation[1]
        goal_.target_pose.pose.orientation.z = target_orientation[2]
        goal_.target_pose.pose.orientation.w = target_orientation[3]

        # send the goal
        self.move_client.send_goal(goal_)

    def get_navigation_status(self) -> int:
        """
        Get result from navigation.

        uint8 PENDING         = 0   # The goal has yet to be processed by the action server
        uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
        uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                                    #   and has since completed its execution (Terminal State)
        uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
        uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                                    #    to some failure (Terminal State)
        uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                    #    because the goal was unattainable or invalid (Terminal State)
        uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                    #    and has not yet completed execution
        uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                                    #    but the action server has not yet confirmed that the goal is canceled
        uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                                    #    and was successfully cancelled (Terminal State)
        uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                                    #    sent over the wire by an action server
        """
        wait = self.move_client.wait_for_result(rospy.Duration(10))
        if not wait:
            rospy.logerr("Move Action server not available!")
            rospy.signal_shutdown("Move Action server not available!")
            return -1
        else:
            # Result of executing the action
            return self.move_client.get_state()

    def cancel_goal(self) -> None:
        """Cancel current navigation goal."""
        self.move_client.cancel_goal()


class RobotInterface:
    """Class to interface control policies with the environment."""

    def __init__(self, namespace: str):
        """Initialize ROS nodes."""

        self.ns = namespace

        rospy.Subscriber(
            self.ns + "/gazebo/model_states", ModelStates, self.cube_callback
        )
        # TODO: get this from config
        self.robot_name = "panda"
        rospy.Subscriber(
            self.ns + "/gazebo/model_states", ModelStates, self.robot_callback
        )
        rospy.Subscriber(
            self.ns + "/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_cb
        )

        # localization
        self.cmd_vel_top = "/key_vel"
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)
        # command
        self.move_msg = Twist()
        self.move_msg.linear.x = 0
        self.move_msg.angular.z = -1
        # subscriber
        rospy.Subscriber("/floating_base_pose_simulated", Odometry, self.odometry_cb)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_cb)
        # localisation services
        glb_loc_srv = rospy.get_param(
            "/robotics_intro/logic_state_machine/global_loc_srv", ""
        )
        rospy.wait_for_service(glb_loc_srv, timeout=30)
        self.amcl_loc_srv = rospy.ServiceProxy(glb_loc_srv, Empty)
        clr_cm_srv = rospy.get_param(
            "/robotics_intro/logic_state_machine/clear_costmaps_srv", ""
        )
        rospy.wait_for_service(clr_cm_srv, timeout=30)
        self.mb_clr_cm_srv = rospy.ServiceProxy(clr_cm_srv, Empty)

        # navigation services
        self.ref_frame = rospy.get_param(
            "/robotics_intro/logic_state_machine/move_base_frame", ""
        )
        # pick
        self.pick_pose_top = rospy.get_param(
            "/robotics_intro/logic_state_machine/pick_pose_topic", ""
        )
        self.pick_pose_received = False
        self.pick_pose_goal = PoseStamped()
        self.pick_subs = rospy.Subscriber(
            self.pick_pose_top, PoseStamped, self.pick_pose_cb
        )
        # place
        self.place_pose_top = rospy.get_param(
            "/robotics_intro/logic_state_machine/place_pose_topic", ""
        )
        self.place_pose_received = False
        self.place_pose_goal = PoseStamped()
        self.place_subs = rospy.Subscriber(
            self.place_pose_top, PoseStamped, self.place_pose_cb
        )
        # action - get_param returns move_base
        self.move_client = SimpleActionClient("move_client", MoveBaseAction)
        self.move_client.wait_for_server(rospy.Duration(100))

        # pick services
        pick_srv = rospy.get_param("/robotics_intro/logic_state_machine/pick_srv", "")
        rospy.wait_for_service(pick_srv, timeout=30)
        self.pick_srv = rospy.ServiceProxy(pick_srv, SetBool)
        # place services
        place_srv = rospy.get_param("/robotics_intro/logic_state_machine/place_srv", "")
        rospy.wait_for_service(place_srv, timeout=30)
        self.place_srv = rospy.ServiceProxy(place_srv, SetBool)

    # ROS related, e.g. callbacks
    def assign_poses(self, pick_pose_, place_pose_):
        # Pick pose
        self.pick_pose.position.x = float(pick_pose_[0])
        self.pick_pose.position.y = float(pick_pose_[1])
        self.pick_pose.position.z = float(pick_pose_[2])
        self.pick_pose.orientation.x = float(pick_pose_[3])
        self.pick_pose.orientation.y = float(pick_pose_[4])
        self.pick_pose.orientation.z = float(pick_pose_[5])
        self.pick_pose.orientation.w = float(pick_pose_[6])

        # Place pose
        self.place_pose.position.x = float(place_pose_[0])
        self.place_pose.position.y = float(place_pose_[1])
        self.place_pose.position.z = float(place_pose_[2])
        self.place_pose.orientation.x = float(place_pose_[3])
        self.place_pose.orientation.y = float(place_pose_[4])
        self.place_pose.orientation.z = float(place_pose_[5])
        self.place_pose.orientation.w = float(place_pose_[6])

    def robot_callback(self, msg):
        if self.robot_name in msg.name:
            # the index may vary (don't know why)
            idx = 0
            for idx in range(len(msg.name)):
                if msg.name[idx] == self.robot_name:
                    break
                else:
                    continue
            # in the same way, also robot configuration can be retrieved
            self.robot_pose.position.x = msg.pose[idx].position.x
            self.robot_pose.position.y = msg.pose[idx].position.y
            self.robot_pose.position.z = msg.pose[idx].position.z
            self.robot_pose.orientation.x = msg.pose[idx].orientation.x
            self.robot_pose.orientation.y = msg.pose[idx].orientation.y
            self.robot_pose.orientation.z = msg.pose[idx].orientation.z
            self.robot_pose.orientation.w = msg.pose[idx].orientation.w

            self.robot_pose_list = []
            self.robot_pose_list.append(msg.pose[idx].position.x)
            self.robot_pose_list.append(msg.pose[idx].position.y)
            self.robot_pose_list.append(msg.pose[idx].position.z)

            error = np.fabs(LA.norm(self.robot_pose_list) - LA.norm(self.amcl_pose))
            if error < 0.1:
                self.localised = True
            else:
                self.localised = False

        else:
            self.robot_pose.position.x = 100.0
            self.robot_pose.position.y = 100.0
            self.robot_pose.position.z = 0.0
            # orientation is not relevant
            self.robot_pose.orientation.w = 1.0

    def cube_callback(self, msg):
        if "aruco_cube" in msg.name:
            # the index may vary (don't know why)
            idx = 0
            for idx in range(len(msg.name)):
                if msg.name[idx] == "aruco_cube":
                    break
                else:
                    continue
            # in the same way, also robot configuration can be retrieved
            self.cube_pose.position.x = msg.pose[idx].position.x
            self.cube_pose.position.y = msg.pose[idx].position.y
            self.cube_pose.position.z = msg.pose[idx].position.z
            # orientation is not relevant
            self.cube_pose.orientation.w = 1.0

            self.cube_pose_list = []
            self.cube_pose_list.append(msg.pose[idx].position.x)
            self.cube_pose_list.append(msg.pose[idx].position.y)
            self.cube_pose_list.append(msg.pose[idx].position.z)

            error = np.fabs(
                LA.norm(self.robot_pose_list[0:2]) - LA.norm(self.cube_pose_list[0:2])
            )
            if error > 0.8:
                # rospy.sleep(2)
                self.have_cube = False

        else:
            self.cube_pose.position.x = -100.0
            self.cube_pose.position.y = -100.0
            self.cube_pose.position.z = 0.0
            # orientation is not relevant
            self.cube_pose.orientation.w = 1.0

    def amcl_pose_cb(self, msg):
        self.amcl_pose = []
        self.amcl_pose.append(msg.pose.pose.position.x)
        self.amcl_pose.append(msg.pose.pose.position.y)
        self.amcl_pose.append(msg.pose.pose.position.z)

    def odometry_cb(self, msg):
        self.odometry = []
        self.odometry.append(msg.pose.pose.position.x)
        self.odometry.append(msg.pose.pose.position.y)
        self.odometry.append(msg.pose.pose.position.z)

    def pick_pose_cb(self, pose_msg):
        self.pick_pose_goal = pose_msg
        self.pick_pose_received = True

    def place_pose_cb(self, pose_msg):
        self.place_pose_goal = pose_msg
        self.place_pose_received = True

    # auxiliary stuff
    def distance_pose(self, pose1, pose2):
        d = Pose()
        # orientation is not important
        # this can be also transformed into a vector3 for simplicity
        d.position.x = np.abs(pose1.position.x - pose2.position.x)
        d.position.y = np.abs(pose1.position.y - pose2.position.y)
        d.position.z = np.abs(pose1.position.z - pose2.position.z)
        d.orientation.w = 1.0

        return d

    def ready_to_pick(self):
        d = self.distance_pose(self.robot_pose, self.cube_pose)
        distance = math.sqrt(d.position.x**2 + d.position.y**2)
        return distance < 1.0

    def ready_to_place(self):
        d = self.distance_pose(self.robot_pose, self.cube_goal)
        distance = math.sqrt(d.position.x**2 + d.position.y**2)
        return distance < 1.0

    def pick(self):
        """
        Pickup the object.
        """
        success = False
        if self.ready_to_pick():
            # command
            pick_srv_req = self.pick_srv(True)
            done = False
            while not done:
                # if the request was succesful!
                if pick_srv_req.success:
                    done = True
                    success = True
                    self.arm_config = "pick"
                # if it failed
                elif not pick_srv_req.success:
                    # rospy.logwarn('PICK ACTION FAILED')
                    done = True
                    success = False
                # if it's still running
                else:
                    continue
        else:
            success = False

        self.have_cube = success

        return success

    def place(self):
        """
        Place the object.
        """
        success = False
        if self.ready_to_place():
            place_srv_req = self.place_srv(True)
            done = False
            while not done:
                # if the request was succesful!
                if place_srv_req.success:
                    done = True
                    success = True
                    self.arm_config = "place"
                # if it failed
                elif not place_srv_req.success:
                    # rospy.logwarn('PICK ACTION FAILED')
                    done = True
                    success = False
                # if it's still running
                else:
                    continue
        else:
            success = False

        self.have_cube = not success

        return success

    def localise(self):
        """
        Conjoins spinning and localisation into one behaviour,
        since the compounded approach failed without explanation...
        """
        amcl_loc_req = self.amcl_loc_srv()
        # counter
        i = 0
        n = 400
        # convergence checker
        converged = False
        while not converged and i <= n:
            # increment counter
            i += 1
            error = np.fabs(LA.norm(self.odometry) - LA.norm(self.amcl_pose))
            if error < 1e-2:
                converged = True
                mb_cc_req = self.mb_clr_cm_srv()
            # keep spinning if still trying
            else:
                rate = rospy.Rate(10)
                self.cmd_vel_pub.publish(self.move_msg)
                rate.sleep()

        self.localised = converged

        return converged
