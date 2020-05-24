#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from grasp_demo.msg import BoundingBox, DetectionActionResult
import imutils
from operator import xor
import numpy as np
from math import sqrt

class object_tracker:

  # Class Attributes
  # initialize a dictionary that maps strings to their corresponding
  # OpenCV object tracker implementations
  OPENCV_OBJECT_TRACKERS = {
      "csrt": cv2.TrackerCSRT_create,
      "kcf": cv2.TrackerKCF_create,
      "boosting": cv2.TrackerBoosting_create,
      "mil": cv2.TrackerMIL_create,
      "tld": cv2.TrackerTLD_create,
      "medianflow": cv2.TrackerMedianFlow_create,
      "mosse": cv2.TrackerMOSSE_create
  }

  algorithm = "csrt"

  # grab the appropriate object tracker using our dictionary of
  # OpenCV object tracker objects
  tracker = OPENCV_OBJECT_TRACKERS[algorithm]()

  def __init__(self):
    # Publishers
    self.image_pub = rospy.Publisher("/object_tracker/image",Image, queue_size=1)
    self.motion_pub = rospy.Publisher("/object_tracker/motion",Bool, queue_size=1)
    self.objLock_pub = rospy.Publisher("/object_tracker/objLock",Bool, queue_size=1)


    
    
    # Subscribers
    self.image_sub = rospy.Subscriber("/fixed_camera/color/image_raw",Image,self.callbackIMG)
    self.BB_sub = rospy.Subscriber("/detection_action/result",DetectionActionResult,self.callbackBB)

    # Variables
    self.initBB = None
    self.box = None
    self.cv_image = None
    self.prev_box = None
    self.motion = None
    self.firstRound = True
    self.successTracking = False
    self.deltas = np.zeros(5)
    self.bridge = CvBridge()

    # # CV Window settings (Used for directly showing image)
    # self.winName = 'Object Tracking'
    # cv2.namedWindow(self.winName, cv2.WINDOW_NORMAL)

    
  def _drawBox(self):
    (x, y, w, h) = [int(v) for v in self.box]
    cv2.rectangle(self.cv_image, (x, y), (x + w, y + h),
        (0, 255, 0), 2)
    # Draws a small black circle in the center of the rectangle
    cv2.circle(self.cv_image, (x+w/2, y+h/2), 2,
        (0, 0, 0), 2)

  def _startTracker(self):
    self.tracker.init(self.cv_image,self.initBB)
    self._updateTracker()

  def _updateTracker(self):
    (self.successTracking, self.box) = self.tracker.update(self.cv_image)
    self._drawBox()

  def _motionDetector(self):
    if self.prev_box is not None:
      # box center calculation
      bc = (self.box[0]+self.box[2]/2,self.box[1]+self.box[3]/2)
      pbc = (self.prev_box[0]+self.prev_box[2]/2,self.prev_box[1]+self.prev_box[3]/2)
      # box center distance
      new_delta = sqrt((bc[0]-pbc[0])**2+(bc[1]-pbc[1])**2)
      # delta history
      self.deltas = np.roll(self.deltas,1)
      self.deltas = np.concatenate(([new_delta], self.deltas[0:-1]))
      
      tempDeltas = self.deltas

      if tempDeltas.sum()>(tempDeltas.size):
        self.motion = True
        rospy.loginfo("moving")
      else:
        self.motion = False
        rospy.loginfo("still")

    # update the boxes
    self.prev_box = self.box
  
  def callbackIMG(self,data):
    try:
      self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    if self.initBB is not None:
      if not self.successTracking:
        self._startTracker()
      else:
        self._updateTracker()
        self._motionDetector()

    # uncomment of script should show images directly
    # cv2.imshow(self.winName, self.cv_image)
    # cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
      self.motion_pub.publish(self.motion)
      self.objLock_pub.publish(self.successTracking)
    except CvBridgeError as e:
      print(e)

  def callbackBB(self,data):
    if data.result.success:
      targetBB = data.result.targetBB
      self.initBB = (targetBB.xmin,targetBB.ymin,abs(targetBB.xmax - targetBB.xmin),abs(targetBB.ymax - targetBB.ymin))
      self.successTracking = False
      print(self.initBB)

def main(args):
  rospy.init_node('object_tracker', anonymous=False)
  ic = object_tracker()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
