#!/usr/bin/env python3
PKG = "piloting_demo"
import os
import unittest
import rospy
import rospkg
import subprocess


class TestPilotingMission(unittest.TestCase):
    def test_state_machine(self):
        # moma_mission_path = rospkg.RosPack().get_path("moma_mission")
        # os.system(
        #     f"perl -i -0777 -pe 's/(SETUP:.*?default_outcome: ).*?\n/\\1Completed\n/s' {moma_mission_path}/config/state_machine/piloting.yaml"
        # )
        # os.system(
        #     f"perl -i -0777 -pe 's/(IDLE:.*?default_outcome: ).*?\n/\\1ExecuteDummyPlan\n/s' {moma_mission_path}/config/state_machine/piloting.yaml"
        # )
        rospy.sleep(120)  # Wait for environment
        p = subprocess.Popen(
            [
                "roslaunch",
                "piloting_demo",
                "mission.launch",
                "sim:=true",
                "standalone:=true",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        err = str(p.stderr.read())
        out = str(p.stdout.read())
        p.communicate()
        # Since the rostest framework doesn't handle stdout well (--text option)
        # we write it to file
        with open("/tmp/piloting_mission.log", "w") as file:
            file.write(out.replace("\\n", "\n").replace("\\t", "\t"))
        with open("/tmp/piloting_mission.err", "w") as file:
            file.write(err.replace("\\n", "\n").replace("\\t", "\t"))
        self.assertEquals("exit code" in err, False)
        # https://github.com/ros/ros_comm/issues/919
        self.assertEquals(p.returncode, 0)


if __name__ == "__main__":
    import rostest

    rostest.rosrun(PKG, "test_piloting_mission", TestPilotingMission)
