import actionlib
import numpy as np
import rospy

from franka_gripper.msg import (
    GraspAction,
    GraspGoal,
    GraspEpsilon,
    StopAction,
    StopGoal,
    MoveAction,
    MoveGoal,
    HomingAction,
    HomingGoal,
)
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal, FrankaState
from sensor_msgs.msg import JointState


class PandaArmClient:
    def __init__(self):
        self._init_recovery()
        self._init_state_callbacks()
        rospy.loginfo("Panda arm ready")

    #### this should also be modified
    def get_state(self):
        q = np.asarray(self._joint_state_msg.position[:7])
        dq = np.asarray(self._joint_state_msg.velocity[:7])
        return q, dq

    def recover(self):
        msg = ErrorRecoveryActionGoal()
        self.recovery_client.send_goal(msg)
        self.recovery_client.wait_for_result(timeout=rospy.Duration(10.0))
        if not self._check_robot_state():
            rospy.sleep(rospy.Duration(3.0))
            if not self._check_robot_state():
                rospy.logerr("Arm error recovery failed")
                return
        self.has_error = False
        rospy.loginfo("Arm error recovered")

    def _check_robot_state(self):
        state = rospy.wait_for_message(
            "franka_state_controller/franka_states", FrankaState
        )
        if state.robot_mode == 4:
            return False
        else:
            return True

    def _init_state_callbacks(self):
        rospy.Subscriber("joint_states", JointState, self._joint_state_cb)
        rospy.wait_for_message("joint_states", JointState)
        rospy.Subscriber(
            "franka_state_controller/franka_states", FrankaState, self._robot_state_cb
        )

    def _init_recovery(self):
        self.has_error = False
        self.recovery_client = actionlib.SimpleActionClient(
            "franka_control/error_recovery", ErrorRecoveryAction
        )

    def _joint_state_cb(self, msg):
        self._joint_state_msg = msg

    def _robot_state_cb(self, msg):
        if msg.robot_mode == 4:
            self.has_error = True
        else:
            self.has_error = False
            # rospy.loginfo("Error detected")


class PandaGripperClient:
    def __init__(self):
        self._init_state_callback()
        self._init_action_clients()
        rospy.loginfo("Panda gripper ready")

    def home(self):
        msg = HomingGoal()
        self.homing_client.send_goal(msg)
        self.homing_client.wait_for_result(rospy.Duration.from_sec(20.0))
        rospy.loginfo("Panda gripper homed")

    def move(self, width, speed=0.1):
        msg = MoveGoal(width, speed)
        self.move_client.send_goal(msg)
        self.move_client.wait_for_result(rospy.Duration.from_sec(10.0))
        return self.move_client.get_result().success

    def grasp(self, width=0.0, e_inner=0.1, e_outer=0.1, speed=0.1, force=5.0):
        rospy.loginfo("Closing gripper")
        msg = GraspGoal(width, GraspEpsilon(e_inner, e_outer), speed, force)
        self.grasp_client.send_goal(msg)
        self.grasp_client.wait_for_result(rospy.Duration(10.0))
        return self.grasp_client.get_result().success

    def release(self, width=0.08):
        rospy.loginfo("Opening gripper")
        return self.move(width)

    def stop(self):
        msg = StopGoal()
        self.stop_client.send_goal(msg)
        self.stop_client.wait_for_result(timeout=rospy.Duration(2.0))

    def read(self):
        return self._joint_state_msg.position[-2] + self._joint_state_msg.position[-1]

    def _init_state_callback(self):
        rospy.Subscriber("joint_states", JointState, self._joint_state_cb)

    def _joint_state_cb(self, msg):
        self._joint_state_msg = msg

    def _init_action_clients(self, ns="/panda/franka_gripper/"):
        self.move_client = actionlib.SimpleActionClient(ns + "move", MoveAction)
        self.grasp_client = actionlib.SimpleActionClient(ns + "grasp", GraspAction)
        self.stop_client = actionlib.SimpleActionClient(ns + "stop", StopAction)
        self.homing_client = actionlib.SimpleActionClient(ns + "homing", HomingAction)
        rospy.loginfo("Waiting for franka_gripper")
        self.move_client.wait_for_server()
        self.grasp_client.wait_for_server()
        self.stop_client.wait_for_server()
        self.homing_client.wait_for_server()
        rospy.loginfo("Gripper connected")
