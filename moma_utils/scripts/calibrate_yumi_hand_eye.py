"""Small script to follow a trajectory to collect a rosbag for hand-eye calibration
between the realsense and Yumi.
"""
import sys

import moveit_commander
import numpy as np
import rospy

from yumi_controllers.yumi_commander import YumiCommander


list_of_joints = [
    [
        0,
        -130,
        30,
        0,
        40,
        0,
        -135,
    ],
    [
        9,
        -109,
        13,
        20,
        61,
        21,
        -94,
    ],
    [54, -104, -8, -45, 58, 42, -57],
    [12, -81, 10, -18, 44, -10, -72],
    [0, -53, 18, -21, 52, 12, -57],
    [0, -16, -7, -26, 76, 100, -24],
    [-100, -37, 0, -2, 79, 100, 63],
    [-57, -43, 2, -21, 91, 72, 46],
]


def main():
    robot = YumiCommander()
    right_arm = robot.right_arm

    for joints in list_of_joints:
        joints = np.deg2rad(joints)

        right_arm.goto_joint_target(
            joints, max_velocity_scaling=0.1, max_acceleration_scaling=0.1
        )


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("hand_eye_trajectory")
    main()
