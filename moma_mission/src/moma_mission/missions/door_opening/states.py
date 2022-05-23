import numpy as np
import rospy
from moma_mission.core import StateRosControl
from moma_mission.missions.door_opening.door import Door
from nav_msgs.msg import Path


class DoorManipulation(StateRosControl):
    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)

        self.ee_frame = self.get_scoped_param("ee_frame")
        self.opening_angle = np.deg2rad(self.get_scoped_param("opening_angle_deg"))
        self.opening_speed = np.deg2rad(self.get_scoped_param("opening_speed_deg"))

        self.door_description_name = self.get_scoped_param("door_description_name")
        self.door = Door(self.door_description_name)

        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return "Failure"

        path = self.door.generate_door_opening_trajectory(
            self.opening_angle, self.opening_speed
        )
        self.path_publisher.publish(path)
        if not self.wait_until_reached(
            self.ee_frame, path.poses[-1], quiet=True, linear_tolerance=0.02
        ):
            return "Failure"

        return "Completed"
