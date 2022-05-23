#!/usr/bin/env python
import math

import rospy
from moma_mission.missions.door_opening.door import Door
from nav_msgs.msg import Path

# Need to have the door state publisher running as well
rospy.init_node("door_calibration")
door_description_name = "door_description"
door = Door(door_description_name)
door.publish_base_frame_from_hand(world_frame="world", ee_frame="tool_frame")
rospy.sleep(1.0)

# debug that the trajectory generation is correct
rospy.loginfo("Publishing opening trajectory")
path = door.generate_door_opening_trajectory(
    opening_angle=math.pi / 2.0, opening_speed=0.5
)

rospy.loginfo(f"Poses in the path: {len(path.poses)}")
path_publisher = rospy.Publisher("/trajectory", Path, queue_size=1)
while path_publisher.get_num_connections() == 0:
    rospy.sleep(1.0)

path_publisher.publish(path)
rospy.sleep(2.0)
