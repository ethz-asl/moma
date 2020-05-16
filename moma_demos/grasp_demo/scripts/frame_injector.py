#!/usr/bin/env python
import rospy
import sys
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class frame_injector:
    def __init__(self):
        self.image_pub = rospy.Publisher("/grasp_demo/image_red",Image, queue_size=1)

        self.image_sub = rospy.Subscriber("/webcam/image_raw",Image,self.callbackIMG)

        self.bridge = CvBridge()
        self.cv_image = None                    # Will be cv Mat

        self.frameCount = 0

        self.time = None

    # def time2sec(self):
    #     return float(self.time.secs)+float(self.time.nsecs/1000000000)

    def callbackIMG(self,data):
        # prev_time = self.time
        # self.time = rospy.Time.now()
        # delta = self.time2sec(self.time) - self.time2sec(prev_time)
        # print(1/delta)
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if self.frameCount == 10:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
            self.frameCount = 0
        else: self.frameCount += 1

def main():
  rospy.init_node('frame_injector', anonymous=False)
  ic = frame_injector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
