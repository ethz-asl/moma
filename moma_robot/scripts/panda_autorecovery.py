#!/usr/bin/env python3
import rospy
from franka_msgs.msg import ErrorRecoveryActionGoal
from franka_msgs.msg import FrankaState
from moma_mission.utils.ros import get_filtered_controllers
from moma_mission.utils.ros import switch_ros_controller


class PandaAutorecovery:
    def __init__(self) -> None:
        self.last_robot_mode = -1
        self.last_started_controllers = []
        self.dependent_controllers = rospy.get_param("~dependent_controllers", [])
        self.manager_namespace = rospy.get_param("~manager_namespace", "")

        self.error_recovery_pub = rospy.Publisher(
            "franka_control/error_recovery/goal", ErrorRecoveryActionGoal, queue_size=1
        )
        rospy.Subscriber(
            "franka_state_controller/franka_states",
            FrankaState,
            self.franka_state_callback,
            queue_size=1,
        )

    def franka_state_callback(self, msg: FrankaState):
        if msg.robot_mode != self.last_robot_mode:
            rospy.loginfo(f"Robot mode changed to {msg.robot_mode}")

            if (
                msg.robot_mode == FrankaState.ROBOT_MODE_REFLEX
                or msg.robot_mode == FrankaState.ROBOT_MODE_USER_STOPPED
            ):
                self.emergency_stop_callback()

            if msg.robot_mode == FrankaState.ROBOT_MODE_REFLEX or (
                msg.robot_mode == FrankaState.ROBOT_MODE_IDLE
                and self.last_robot_mode == FrankaState.ROBOT_MODE_USER_STOPPED
            ):
                self.error_recovery()

        self.last_robot_mode = msg.robot_mode

    def emergency_stop_callback(self):
        rospy.logwarn("Robot entered emergency stop state, stopping controllers")
        running_controllers = get_filtered_controllers(
            "running", self.manager_namespace
        )
        self.last_started_controllers = [
            c for c in self.dependent_controllers if c in running_controllers
        ]

        # switch_ros_controller(
        #    startlist=[],
        #    stoplist=self.dependent_controllers,
        #    manager_namespace=self.manager_namespace,
        # )

    def error_recovery(self):
        rospy.logwarn("Automatically recovering from error")
        goal = ErrorRecoveryActionGoal()
        self.error_recovery_pub.publish(goal)

        # Start previously started controllers again
        # switch_ros_controller(
        #    startlist=self.last_started_controllers,
        #    stoplist=[],
        #    manager_namespace=self.manager_namespace,
        # )
        self.last_started_controllers = []


if __name__ == "__main__":
    rospy.init_node("panda_autorecovery")
    PandaAutorecovery()
    rospy.spin()
