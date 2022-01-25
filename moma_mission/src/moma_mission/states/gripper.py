import rospy
import actionlib
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

from moma_mission.core import StateRos, StateRosControl


class GripperPositionControlState(StateRos):
    """
    TODO
    """

    def __init__(self, ns):
        StateRos.__init__(self, ns=ns)
        command_topic_name = self.get_scoped_param("command_topic")
        self.command = self.get_scoped_param("command")
        self.command_publisher = rospy.Publisher(
            command_topic_name, Float64, queue_size=10)

    def run(self):
        # It should actually switch to the right controller but assuming that
        # the controller is already switched
        rospy.loginfo(
            "Sending target gripper position: {} %".format(self.command))
        cmd = Float64()
        cmd.data = self.command
        self.command_publisher.publish(cmd)

        rospy.loginfo("Sleeping 5.0s before returning.")
        rospy.sleep(5.0)

        return 'Completed'


class GripperUSB(StateRos):
    """
    TODO
    """

    def __init__(self, ns):
        StateRos.__init__(self, ns=ns)
        command_topic_name = self.get_scoped_param("command_topic")
        self.position = self.get_scoped_param("position")
        self.effort = self.get_scoped_param("effort")
        self.velocity = self.get_scoped_param("velocity")
        self.command_publisher = rospy.Publisher(
            command_topic_name, JointState, queue_size=10)

    def run(self):
        # It should actually switch to the right controller but assuming that
        # the controller is already switched
        rospy.loginfo("Target gripper position: {}, effort: {}".format(
            self.position, self.effort))
        cmd = JointState()
        cmd.position.append(self.position)
        cmd.velocity.append(self.velocity)
        cmd.effort.append(self.effort)
        self.command_publisher.publish(cmd)

        rospy.loginfo("Sleeping 3.0s before returning.")
        rospy.sleep(3.0)

        return 'Completed'


class GripperControl(StateRos):
    """
    This state controls the gripper through the GripperCommandAction
    """

    def __init__(self, ns=""):
        StateRos.__init__(self, ns=ns)

        self.position = self.get_scoped_param("position")
        self.max_effort = self.get_scoped_param("max_effort")
        self.tolerance = self.get_scoped_param("tolerance")
        self.server_timeout = self.get_scoped_param("timeout")

        self.gripper_cmd = GripperCommandGoal()

        self.gripper_cmd.command.position = self.position
        self.gripper_cmd.command.max_effort = self.max_effort

        self.gripper_action_name = self.get_scoped_param("gripper_action_name")
        self.gripper_client = actionlib.SimpleActionClient(
            self.gripper_action_name, GripperCommandAction)

    def run(self):
        if not self.gripper_client.wait_for_server(rospy.Duration(self.server_timeout)):
            rospy.logerr("Timeout exceeded while waiting for {} server".format(
                self.gripper_action_name))
            return 'Failure'

        # TODO(giuseppe) remove hack --> kinova takes time to switch to highlevel mode
        rospy.sleep(1.0)
        self.gripper_client.send_goal(self.gripper_cmd)
        if not self.gripper_client.wait_for_result(rospy.Duration(self.server_timeout)):
            rospy.logerr(
                "Timeout exceeded while waiting the gripper action to complete")
            return 'Aborted'

        rospy.loginfo(
            f"Sending gripper command: pos={self.gripper_cmd.command.position}, max_eff={self.gripper_cmd.command.max_effort}")
        result = self.gripper_client.get_result()
        error = abs(result.position - self.position)
        tolerance_met = error < self.tolerance

        success = True
        if result is None:
            rospy.logerr("None received from the gripper server")
            success = False
        elif result.stalled and not tolerance_met:
            rospy.logerr(
                "Gripper stalled and not moving, position error {} is larger than tolerance".format(error))
            success = False
        elif result.stalled and tolerance_met:
            rospy.logerr(
                "Gripper stalled and not moving, position error {} is smaller than tolerance".format(error))
            success = True
        elif result.reached_goal:
            rospy.loginfo("Gripper successfully reached the goal")
            success = True

        if success:
            rospy.sleep(2.0)  # just for safety
            return 'Completed'
        else:
            return 'Failure'
