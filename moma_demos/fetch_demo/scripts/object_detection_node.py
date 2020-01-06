#!/usr/bin/env python

import actionlib
from fetch_demo.srv import ObjectDetection, ObjectDetectionResponse
from geometry_msgs.msg import Pose
import rospy


def handle_object_detection():
    found_object = True
    if found_object:
        return ObjectDetectionResponse()


def main():
    rospy.init_node("object_detection_server")
    s = rospy.Service("object_detection_service", ObjectDetection, handle_object_detection)
    rospy.loginfo("Object detection service ready")
    rospy.spin()

if __name__ == "__main__":
    main()
