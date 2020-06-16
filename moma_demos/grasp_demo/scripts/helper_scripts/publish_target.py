#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String

classesFile = "/home/zinnerc/catkin_ws/src/moma/moma_demos/grasp_demo/src/yolo/coco.names"
classes = None
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')

def pub_target(target):
    rospy.init_node('target_publisher', anonymous=False)

    rate = rospy.Rate(5)
    while target in classes:
        targetObj_pub = rospy.Publisher("/target_object",String, queue_size=1)

        if target in classes:
            targetObj_pub.publish(target)
        else: pass
        rate.sleep

def usage():
    return "%s target"%sys.argv[0]

def notClass():
    return "%s is not in the list of classes" % sys.argv[1]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        target = sys.argv[1]
        if target in classes:
            pub_target(target)
        else: print notClass()
    else:
        print usage()
        sys.exit(1)
