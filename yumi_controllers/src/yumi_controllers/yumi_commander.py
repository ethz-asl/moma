import actionlib
import moveit_commander
from moveit_commander.conversions import list_to_pose
import rospy


class YumiCommander(object):
    """
    TODO(mbreyer): write docstrings
    """

    def __init__(self):
        # Connect to the Yumi MoveGroup
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.left_arm = GroupCommander("left_arm")
        self.right_arm = GroupCommander("right_arm")


class GroupCommander(object):
    def __init__(self, group_name):
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    def goto_joint_target(self, joints, max_velocity_scaling=1.0):
        self.move_group.set_max_velocity_scaling_factor(max_velocity_scaling)
        self.move_group.set_joint_value_target(joints)
        plan = self.move_group.plan()
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        return success

    def goto_pose_target(self, pose, max_velocity_scaling=1.0):
        pose_msg = list_to_pose(pose) if type(pose) is list else pose
        self.move_group.set_max_velocity_scaling_factor(max_velocity_scaling)
        self.move_group.set_pose_target(pose_msg)
        plan = self.move_group.plan()
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success
