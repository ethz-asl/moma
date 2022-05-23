import actionlib
import rospy
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
from control_msgs.msg import GripperCommandResult
from franka_gripper.msg import GraspAction
from franka_gripper.msg import GraspGoal
from franka_gripper.msg import GraspResult
from moma_mission.core import StateRos
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class GripperPositionControlState(StateRos):
    """
    TODO
    """

    def __init__(self, ns):
        StateRos.__init__(self, ns=ns)
        command_topic_name = self.get_scoped_param("command_topic")
        self.command = self.get_scoped_param("command")
        self.command_publisher = rospy.Publisher(
            command_topic_name, Float64, queue_size=10
        )

    def run(self):
        # It should actually switch to the right controller but assuming that
        # the controller is already switched
        rospy.loginfo("Sending target gripper position: {} %".format(self.command))
        cmd = Float64()
        cmd.data = self.command
        self.command_publisher.publish(cmd)

        rospy.loginfo("Sleeping 5.0s before returning.")
        rospy.sleep(5.0)

        return "Completed"


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
            command_topic_name, JointState, queue_size=10
        )

    def run(self):
        # It should actually switch to the right controller but assuming that
        # the controller is already switched
        rospy.loginfo(
            "Target gripper position: {}, effort: {}".format(self.position, self.effort)
        )
        cmd = JointState()
        cmd.position.append(self.position)
        cmd.velocity.append(self.velocity)
        cmd.effort.append(self.effort)
        self.command_publisher.publish(cmd)

        rospy.loginfo("Sleeping 3.0s before returning.")
        rospy.sleep(3.0)

        return "Completed"


class GripperAction(StateRos):
    """
    This state controls the gripper through the GripperCommandAction
    """

    def __init__(self, ns="", action_type=GripperCommandAction):
        StateRos.__init__(self, ns=ns)

        self.position = self.get_scoped_param("position")
        self.max_effort = self.get_scoped_param("max_effort")
        self.tolerance = self.get_scoped_param("tolerance")
        self.server_timeout = self.get_scoped_param("timeout", 35.0)

        self.gripper_goal = None
        self.gripper_action_name = self.get_scoped_param("gripper_action_name")
        self.gripper_client = actionlib.SimpleActionClient(
            self.gripper_action_name, action_type
        )
        self._set_goal()
        self.success = False

    def _set_goal(self):
        raise NotImplementedError()

    def _done_cb(self, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.success = True
        self._process_result(result)

    def _process_result(self, result):
        pass

    def run(self):
        if not self.gripper_client.wait_for_server(rospy.Duration(self.server_timeout)):
            rospy.logerr(
                "Timeout exceeded while waiting for {} server".format(
                    self.gripper_action_name
                )
            )
            return "Failure"

        self.gripper_client.send_goal(self.gripper_goal, done_cb=self._done_cb)
        if not self.gripper_client.wait_for_result(rospy.Duration(self.server_timeout)):
            rospy.logerr(
                "Timeout exceeded while waiting the gripper action to complete"
            )
            return "Failure"

        if self.success:
            return "Completed"
        else:
            return "Failure"


class GripperControl(GripperAction):
    """
    This state controls the gripper through the GripperCommandAction
    """

    def __init__(self, ns=""):
        GripperAction.__init__(self, ns=ns, action_type=GripperCommandAction)

    def _set_goal(self):
        self.gripper_goal = GripperCommandGoal()
        self.gripper_goal.command.position = self.position
        self.gripper_goal.command.max_effort = self.max_effort

    def _process_result(self, result: GripperCommandResult):
        if result.stalled:
            rospy.loginfo("Gripper stalled")
            self.success = False
        elif result.reached_goal:
            rospy.loginfo("Gripper reached goal")
            self.success = True
        elif abs(result.position - self.position) > self.tolerance:
            rospy.logwarn("Gripper did not meet tolerance")
            self.success = False


class GripperGrasp(GripperAction):
    """
    This state controls the gripper through the GripperCommandAction
    """

    def __init__(self, ns=""):
        GripperAction.__init__(self, ns=ns, action_type=GraspAction)

    def _set_goal(self):
        self.gripper_goal = GraspGoal()
        self.gripper_goal.epsilon.inner = (
            1.0  # how much fingers can close more than specified
        )
        self.gripper_goal.epsilon.outer = (
            1.0  # how much fingers can open more than specified
        )
        self.gripper_goal.speed = 0.01
        self.gripper_goal.width = self.position
        self.gripper_goal.force = self.max_effort

    def _process_result(self, result: GraspResult):
        if not result.success:
            rospy.logwarn(f"Grasp failed with error msg [{result.error}]")
