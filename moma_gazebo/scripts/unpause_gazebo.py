#!/usr/bin/env python3
import time

import rospy
from std_srvs.srv import Empty


def main(wait_time):

    rospy.loginfo("Waiting {}s before unpausing".format(wait_time))
    time.sleep(wait_time)

    # Unpause the physics
    rospy.loginfo("Unpausing Gazebo...")
    rospy.wait_for_service("/gazebo/unpause_physics")
    unpause_gazebo = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    resp = unpause_gazebo()
    rospy.loginfo("Unpaused Gazebo.")


if __name__ == "__main__":
    rospy.init_node("unpause_gazebo")
    wait_before_unpause = rospy.get_param("~wait_before_unpause", 0.0)
    main(wait_before_unpause)
