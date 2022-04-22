#!/usr/bin/env python
PKG = "piloting_demo"
import unittest
import rospy
import subprocess


class TestPilotingMission(unittest.TestCase):
    def test_state_machine(self):
        rospy.sleep(120)  # Wait for environment
        p = subprocess.Popen(
            ["roslaunch", "piloting_demo", "mission.launch", "sim:=true"],
            stderr=subprocess.PIPE,
        )
        err = "exit code" in str(p.stderr.read())
        self.assertEquals(err, False)
        # https://github.com/ros/ros_comm/issues/919
        self.assertEquals(p.returncode, None)


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, "test_piloting_mission", TestPilotingMission)
