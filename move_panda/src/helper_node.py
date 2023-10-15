#!/usr/bin/python3

import rospy
import time
import sys
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from astrocam_msgs.msg import Command, Key, Mode  

class Helper:
    def __init__(self):
        self._image_pub = rospy.Publisher('/usb_cam/image_raw/compressed', CompressedImage, 
                queue_size=1, tcp_nodelay=False)
        self._command_sub = rospy.Subscriber('/command', Command,
                queue_size=1, callback=self._command_cb, tcp_nodelay=True)

        self._mode = Mode.SETTINGS
        self.helper_imgs = [
                cv2.imread("helper/imgs/Mode_Settings.jpg", cv2.IMREAD_GRAYSCALE),
                cv2.imread("helper/imgs/Mode_Liveview.jpg", cv2.IMREAD_GRAYSCALE),
                cv2.imread("helper/imgs/Mode_ExtLiveview.jpg", cv2.IMREAD_GRAYSCALE),
                cv2.imread("helper/imgs/Mode_Imaging.jpg", cv2.IMREAD_GRAYSCALE),
                ]
        self.helper_imgs = [np.array(cv2.imencode(".jpg", 255-i)[1]).tobytes() for i in self.helper_imgs]
        time.sleep(0.1)
        rospy.loginfo("Helper Initialized")


    def _command_cb(self, msg):
        # Only do something upon entry
        if (msg.mode.mode != Mode.DISPLAY):
            self._mode = msg.mode.mode

        if not (msg.mode.has_changed):
            return

        if not (msg.mode.mode == Mode.DISPLAY):
            return

        rospy.loginfo("Displaying help message for " + str(self._mode))

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = self.helper_imgs[self._mode-1]
        self._image_pub.publish(msg)


if __name__=="__main__":
    rospy.init_node("helper", anonymous=False)
    Helper()
    
    rospy.spin()
