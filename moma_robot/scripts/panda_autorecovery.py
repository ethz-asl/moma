#!/usr/bin/env python3
import rospy
from franka_msgs.msg import ErrorRecoveryActionGoal, FrankaState

last_robot_mode = -1


def FrankaStateCallback(msg: FrankaState):
    global last_robot_mode
    if msg.robot_mode != last_robot_mode:
        rospy.loginfo(f"Robot mode changed to {msg.robot_mode}")
    last_robot_mode = msg.robot_mode

    if msg.robot_mode == FrankaState.ROBOT_MODE_REFLEX:
        rospy.logwarn("Automatically recovering from reflex error")
        goal = ErrorRecoveryActionGoal()
        error_recovery_pub.publish(goal)


# Init ros
rospy.init_node("panda_autorecovery")

error_recovery_pub = rospy.Publisher(
    "franka_control/error_recovery/goal", ErrorRecoveryActionGoal, queue_size=1
)
rospy.Subscriber(
    "franka_state_controller/franka_states",
    FrankaState,
    FrankaStateCallback,
    queue_size=1,
)

rospy.spin()
