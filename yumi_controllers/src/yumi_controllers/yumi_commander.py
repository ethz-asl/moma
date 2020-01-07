import actionlib
import moveit_commander
from moveit_commander.conversions import list_to_pose
import rospy
from yumi_hw.srv import YumiGrasp, YumiGraspRequest


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

        grasp_service_name = "/yumi/yumi_gripper/do_grasp"
        rospy.wait_for_service(grasp_service_name)
        self.grasp_client = rospy.ServiceProxy(grasp_service_name, YumiGrasp)

        release_service_name = "/yumi/yumi_gripper/release_grasp"
        rospy.wait_for_service(release_service_name)
        self.release_client = rospy.ServiceProxy(release_service_name, YumiGrasp)

    def goto_joint_target(
        self, joints, max_velocity_scaling=1.0, max_acceleration_scaling=1.0
    ):
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
        req = YumiGraspRequest(gripper_id=1 if self.group_name == "left_arm" else 2)
        self.grasp_client.call(req)

    def release(self):
        req = YumiGraspRequest(gripper_id=1 if self.group_name == "left_arm" else 2)
        self.release_client.call(req)
