#!/usr/bin/env python
PKG = "piloting_demo"
import unittest
import rospy


class TestPilotingMission(unittest.TestCase):
    def test_one_equals_one(self):
        rospy.sleep(300)  # Wait for mission to complete
        # TODO check for some topic /result or something
        self.assertEquals(1, 1, "1!=1")


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, "test_piloting_mission", TestPilotingMission)
