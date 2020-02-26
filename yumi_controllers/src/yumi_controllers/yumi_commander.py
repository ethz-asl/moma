import actionlib
import moveit_commander
from moveit_commander.conversions import list_to_pose
import rospy
from std_msgs.msg import Float64


class YumiCommander(object):
    """
    TODO(mbreyer): write docstrings
    """

    def __init__(self):
        # Connect to the Yumi MoveGroup
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.left_arm = ArmCommander("left_arm")
        self.right_arm = ArmCommander("right_arm")


class ArmCommander(object):
    def __init__(self, group_name):
        self.group_name = group_name
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_pose_reference_frame("yumi_body")

        self._gripper_force = 10
        self.grasp_pub = rospy.Publisher(
            "/yumi/gripper_{}_effort_cmd".format(group_name[0]), Float64, queue_size=1
        )

    def goto_joint_target(
        self, joints, max_velocity_scaling=1.0, max_acceleration_scaling=1.0
    ):
        if type(joints) == list:
            joints = {
                "yumi_joint_{}_{}".format(i + 1, self.group_name[0]): joints[i]
                for i in range(7)
            }

        self.move_group.set_max_velocity_scaling_factor(max_velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(max_acceleration_scaling)
        self.move_group.set_joint_value_target(joints)
        plan = self.move_group.plan()
        success = self.move_group.execute(plan, wait=True)
        self.move_group.stop()
        return success

    def goto_pose_target(
        self, pose, max_velocity_scaling=1.0, max_acceleration_scaling=1.0
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

    def grasp(self):
        self.grasp_pub.publish(Float64(data=self._gripper_force))
        rospy.sleep(2.0)

    def release(self):
        self.grasp_pub.publish(Float64(data=-self._gripper_force))
        rospy.sleep(2.0)
