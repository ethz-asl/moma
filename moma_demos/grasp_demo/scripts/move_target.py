#!/usr/bin/env python

import sys
import rospy
from gazebo_msgs.srv import ApplyBodyWrench
import time
from math import sqrt,pow

from geometry_msgs.msg import Wrench

def move_cmd(x,y,d,t):
    rospy.wait_for_service('/gazebo/apply_body_wrench')

    apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)

    m = 0.2 # mass of the orange

    v_norm = sqrt(pow(x,2)+pow(y,2))

    wrench = Wrench()
    wrench.force.x = 1/float(v_norm)*x*2*d/t*m
    wrench.force.y = 1/float(v_norm)*y*2*d/t*m
    wrench.force.z = 0
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 0

    stop = Wrench()
    stop.force.x = -1*wrench.force.x
    stop.force.y = -1*wrench.force.y
    stop.force.z = -1*wrench.force.z
    stop.torque.x = -1*wrench.torque.x
    stop.torque.y = -1*wrench.torque.y
    stop.torque.z = -1*wrench.torque.z


    try:
        apply_body_wrench(body_name = 'sphere_o_40mm::link_2',
                 reference_frame = '',
                 wrench = wrench,
                 duration = rospy.Duration(1))
        time.sleep(t)
        apply_body_wrench(body_name = 'sphere_o_40mm::link_2',
                 reference_frame = '',
                 wrench = stop,
                 duration = rospy.Duration(1))
        print("done")

    except rospy.ServiceException, e:
        print("Service call failed: %s", e)

def usage():
    return "%s x y d t"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 5:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        d = float(sys.argv[3])
        t = float(sys.argv[4])
        move_cmd(x,y,d,t)
    else:
        print usage()
        sys.exit(1)

# REFERENCE
# https://answers.ros.org/question/246790/reference-frame-for-applybodywrench/
# https://answers.ros.org/question/321758/calling-predefined-service-from-python-script/
# http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29
