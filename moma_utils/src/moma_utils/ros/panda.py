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
from control_msgs.msg import GripperCommand, GripperCommandAction, GripperCommandGoal


class PandaArmClient:
    def __init__(self):
        self._init_recovery()
        self._init_state_callbacks()
        rospy.loginfo("Panda arm ready")

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
        if not self.has_error and msg.robot_mode == 4:
            self.has_error = True
            rospy.loginfo("Error detected")


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

    def move(self, width, speed=0.05):
        msg = MoveGoal(width, speed)
        self.move_client.send_goal(msg)
        self.move_client.wait_for_result(rospy.Duration.from_sec(2.0))

    def grasp(self, width=0.0, e_inner=0.1, e_outer=0.1, speed=0.1, force=10.0):
        rospy.loginfo("Closing gripper")
        self.move_gripper2(0.0)

    def grasp2(self, width=0.0, e_inner=0.1, e_outer=0.1, speed=0.1, force=10.0):
        rospy.loginfo("Closing gripper")
        msg = GraspGoal(width, GraspEpsilon(e_inner, e_outer), speed, force)
        self.grasp_client.send_goal(msg)
        self.grasp_client.wait_for_result(rospy.Duration(2.0))

    def release(self):
        # self.move(0.08)
        rospy.loginfo("Opening gripper")
        self.move_gripper1(0.035)

    def stop(self):
        msg = StopGoal()
        self.stop_client.send_goal(msg)
        self.stop_client.wait_for_result(timeout=rospy.Duration(5.0))

    def read(self):
        return self._joint_state_msg.position[7] + self._joint_state_msg.position[8]

    def _init_state_callback(self):
        rospy.Subscriber("joint_states", JointState, self._joint_state_cb)

    def _joint_state_cb(self, msg):
        self._joint_state_msg = msg

    def _init_action_clients(self, ns="franka_gripper/"):
        self.move_client = actionlib.SimpleActionClient(ns + "move", MoveAction)
        self.grasp_client = actionlib.SimpleActionClient(ns + "grasp", GraspAction)
        self.stop_client = actionlib.SimpleActionClient(ns + "stop", StopAction)
        rospy.loginfo("Waiting for franka_gripper/move")
        self.move_client.wait_for_server()

        name = ns + "gripper_action"
        self.gripper_client1 = actionlib.SimpleActionClient(name, GripperCommandAction)
        self.gripper_client1.wait_for_server()
        name = ns + "grasp"
        self.gripper_client2 = actionlib.SimpleActionClient(name, GraspAction)
        self.gripper_client2.wait_for_server()
        name = ns + "stop"
        self.gripper_stop_client = actionlib.SimpleActionClient(name, StopAction)
        self.gripper_stop_client.wait_for_server()
        name = ns + "homing"
        self.homing_client = actionlib.SimpleActionClient(name, HomingAction)
        self.homing_client.wait_for_server()
        rospy.loginfo("Gripper connected")

    def move_gripper1(self, width, max_effort=10):
        command = GripperCommand(width, max_effort)
        rospy.loginfo("width: {}".format(width))
        goal = GripperCommandGoal(command)
        self.gripper_client1.send_goal(goal)
        self.gripper_client1.wait_for_result(timeout=rospy.Duration(5.0))
        res = self.gripper_client1.get_result()
        rospy.loginfo("Gripper res: {}".format(res.reached_goal))

    def move_gripper2(self, width, max_effort=10):
        msg = GraspGoal()
        msg.width = width
        rospy.loginfo("Going to width {}".format(width))
        msg.speed = 0.1
        msg.force = max_effort
        msg.epsilon.inner = 10
        msg.epsilon.outer = 10
        self.gripper_client2.send_goal(msg)
        self.gripper_client2.wait_for_result(timeout=rospy.Duration(5.0))
        res = self.gripper_client2.get_result()
        if res is None:
            rospy.loginfo("Timeout")
            return
        rospy.loginfo("Gripper res: {}".format(res.success))
        if not res.success:
            rospy.logwarn("Gripper issued warning: {}".format(res.error))
