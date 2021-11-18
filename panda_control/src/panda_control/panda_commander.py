import actionlib
from control_msgs.msg import GripperCommand, GripperCommandAction, GripperCommandGoal
import franka_msgs.msg
from franka_gripper.msg import GraspAction, GraspGoal, StopAction, StopGoal
import moveit_commander
from moveit_commander.conversions import list_to_pose
from moveit_msgs.msg import MoveGroupAction
import rospy
import numpy as np


"""
THIS IS DEPRECATED. PLEASE DO NOT USE THIS.
Instead, use moma_utils/src/moma_utils/ros/panda.py to control the panda arm. 
"""


class PandaCommander(object):
    """
    TODO(mbreyer): write docstrings
    """

    def __init__(self):
        self._connect_to_move_group()
        self._setup_gripper_action_client()
        self._setup_robot_state_connection()
        rospy.loginfo("PandaCommander ready")

    def _connect_to_move_group(self):
        # wait for moveit to be available
        tmp = actionlib.SimpleActionClient("move_group", MoveGroupAction)
        tmp.wait_for_server()

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")

    def _setup_gripper_action_client(self):
        # name = "gripper_cmd" if rospy.get_param("use_sim_time") else "gripper_action"
        name = "gripper_action"
        name = "franka_gripper/" + name
        self.gripper_client1 = actionlib.SimpleActionClient(name, GripperCommandAction)
        self.gripper_client1.wait_for_server()

        name = "franka_gripper/grasp"
        self.gripper_client2 = actionlib.SimpleActionClient(name, GraspAction)
        self.gripper_client2.wait_for_server()
        name = "franka_gripper/stop"
        self.gripper_stop_client = actionlib.SimpleActionClient(name, StopAction)
        self.gripper_stop_client.wait_for_server()
        rospy.loginfo("Gripper connected")
        rospy.loginfo("Gripper connected")

    def _setup_robot_state_connection(self):
        self.has_error = False
        self.recover_pub = rospy.Publisher(
            "franka_control/error_recovery/goal",
            franka_msgs.msg.ErrorRecoveryActionGoal,
            queue_size=1,
        )
        rospy.Subscriber(
            "franka_state_controller/franka_states",
            franka_msgs.msg.FrankaState,
            self._robot_state_cb,
            queue_size=1,
        )

    def home(self):
        self.goto_joint_target([0, -0.785, 0, -2.356, 0, 1.57, 0.785], 0.4, 0.4)

    def recover(self):
        self.recover_pub.publish(franka_msgs.msg.ErrorRecoveryActionGoal())
        rospy.sleep(3.0)
        self.has_error = False
        rospy.loginfo("Panda recovered")

    def goto_joint_target(
        self, joints, max_velocity_scaling=0.1, max_acceleration_scaling=0.1
    ):
        self.move_group.set_max_velocity_scaling_factor(max_velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(max_acceleration_scaling)
        self.move_group.set_joint_value_target(joints)
        plan = self.move_group.plan()
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        return success

    def goto_pose_target(
        self, pose, max_velocity_scaling=0.1, max_acceleration_scaling=0.1
    ):
        pose_msg = list_to_pose(pose) if type(pose) is list else pose
        self.move_group.set_max_velocity_scaling_factor(max_velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(max_acceleration_scaling)
        self.move_group.set_pose_target(pose_msg)
        plan = self.move_group.plan()
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def follow_cartesian_waypoints(
        self, poses, velocity_scaling=0.1, acceleration_scaling=0.1
    ):
        assert type(poses) == list

        waypoints = []
        for pose in poses:
            pose_msg = list_to_pose(pose) if type(pose) is list else pose
            waypoints.append(pose_msg)

        plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        if fraction != 1.0:
            rospy.logerr("Unable to plan entire path!")
            return False

        plan = self.move_group.retime_trajectory(
            self.robot.get_current_state(), plan, velocity_scaling, acceleration_scaling
        )

        success = self.move_group.execute(plan, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()

        return success

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
        msg.speed = 0.065
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

    def grasp(self):
        rospy.loginfo("grasp inside")
        self.move_gripper2(0.0)

    def release(self):
        rospy.loginfo("release inside")
        self.move_gripper1(0.039)

    def stop(self):
        rospy.loginfo("stopping")
        msg = StopGoal()
        self.gripper_stop_client.send_goal(msg)
        self.gripper_stop_client.wait_for_result(timeout=rospy.Duration(5.0))

    def check_object_grasped(self):
        # raise NotImplementedError
        # TODO need to implement this
        return True

    def _robot_state_cb(self, msg):
        if not self.has_error and msg.robot_mode == 4:
            self.has_error = True
            rospy.loginfo("Error detected")
