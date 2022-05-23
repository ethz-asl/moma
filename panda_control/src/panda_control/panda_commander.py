import actionlib
import moveit_commander
import rospy
from control_msgs.msg import GripperCommand
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
from moveit_commander.conversions import list_to_pose
from moveit_msgs.msg import MoveGroupAction


class PandaCommander(object):
    """
    TODO(mbreyer): write docstrings
    """

    def __init__(self):
        self._connect_to_move_group()
        self._setup_gripper_action_client()
        rospy.loginfo("PandaCommander ready")

    def _connect_to_move_group(self):
        # wait for moveit to be available
        tmp = actionlib.SimpleActionClient("move_group", MoveGroupAction)
        tmp.wait_for_server()

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")

    def _setup_gripper_action_client(self):
        name = "gripper_cmd" if rospy.get_param("use_sim_time") else "gripper_action"
        name = "franka_gripper/" + name
        self.gripper_client = actionlib.SimpleActionClient(name, GripperCommandAction)
        self.gripper_client.wait_for_server()

    def home(self):
        self.goto_joint_target([0, -0.785, 0, -2.356, 0, 1.57, 0.785], 0.4, 0.4)

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

    def move_gripper(self, width, max_effort=10):
        command = GripperCommand(width, max_effort)
        goal = GripperCommandGoal(command)
        self.gripper_client.send_goal(goal)
        return self.gripper_client.wait_for_result(timeout=rospy.Duration(1.0))

    def grasp(self):
        self.move_gripper(0.0)

    def release(self):
        self.move_gripper(0.04)

    def check_object_grasped(self):
        # raise NotImplementedError
        # TODO need to implement this
        return True
