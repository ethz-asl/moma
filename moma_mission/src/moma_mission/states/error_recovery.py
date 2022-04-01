import rospy
from franka_msgs.msg import ErrorRecoveryActionGoal

from moma_mission.core import StateRosControl


class ErrorRecoveryState(StateRosControl):
    """
    Recover from robot errors.
    """

    def __init__(self, ns, outcomes=["Completed", "Failure"]):
        StateRosControl.__init__(self, ns=ns, outcomes=outcomes)

        self.robot_namespace = self.get_scoped_param("robot_namespace", "")
        self.error_recovery_pub = rospy.Publisher(
            f"/{self.robot_namespace}/franka_control/error_recovery/goal",
            ErrorRecoveryActionGoal,
            queue_size=1,
        )
        rospy.loginfo(f"/{self.robot_namespace}/franka_control/error_recovery/goal")

    def run(self):
        self.do_switch()
        goal = ErrorRecoveryActionGoal()
        self.error_recovery_pub.publish(goal)
        return "Completed"
