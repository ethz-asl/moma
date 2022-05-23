import unittest

import rospy
from panda_control.panda_commander import PandaCommander


class TestPandaCommander(unittest.TestCase):
    def setUp(self):
        rospy.init_node("test_panda_commander")
        self.commander = PandaCommander()

    def test_move_gripper(self):
        self.commander.move_gripper(0.0)
        self.commander.move_gripper(0.07)


if __name__ == "__main__":
    unittest.main()
