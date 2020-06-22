#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from grasp_demo.msg import BoundingBox, DetectionActionResult
from geometry_msgs.msg import Pose, PoseStamped
from operator import xor
import numpy as np
from math import sqrt
import tf

from helper_functions import create_pose, position2imgPoints

class object_tracker:

  def __init__(self):
    # Publishers
    self.image_pub = rospy.Publisher("/object_tracker/image",Image, queue_size=1)
    self.objRest_pub = rospy.Publisher("/object_tracker/objRest",String, queue_size=1)
    self.objLock_pub = rospy.Publisher("/object_tracker/objLock",String, queue_size=1)
    self.objBB_pub = rospy.Publisher("/object_tracker/objBB",BoundingBox, queue_size=1)


    # Subscribers
    self.image_sub = rospy.Subscriber("/fixed_camera/color/image_raw",Image,self.callbackIMG)
    self.BB_sub = rospy.Subscriber("/detection_action/result",DetectionActionResult,self.callbackBB)

    # Variables
    self.initBB = None
    self.box = None
    self.cv_image = None
    self.prev_box = None
    self.objRest = None
    self.firstRound = True
    self.successTracking = False
    self.successDetection = None
    self.bridge = CvBridge()

    self.motion_threshold = rospy.get_param("/moma_demo/tracking_motion_threshold")
    self.threshold_length = rospy.get_param("/moma_demo/tracking_threshold_length")
    self.deltas = np.zeros(self.threshold_length)

    # # CV Window settings (Used for directly showing image)
    # self.winName = 'Object Tracking'
    # cv2.namedWindow(self.winName, cv2.WINDOW_NORMAL)

    self.OPENCV_OBJECT_TRACKERS = {
      "csrt": cv2.TrackerCSRT_create,
      "kcf": cv2.TrackerKCF_create,
      "boosting": cv2.TrackerBoosting_create,
      "mil": cv2.TrackerMIL_create,
      "tld": cv2.TrackerTLD_create,
      "medianflow": cv2.TrackerMedianFlow_create,
      "mosse": cv2.TrackerMOSSE_create
    }

    self.algorithm = rospy.get_param("/moma_demo/tracking_method")

    self.listener = tf.TransformListener()

  def create_pose(self,trans,rot):
    pose=Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose

  def position2imgPoints(self,position):
    position_transformed = np.array([[position.x,position.y,position.z]])
    position_transformed = position_transformed.T
    cameraMatrix = np.array([602.1849879340944, 0.0, 320.5, 0.0, 602.1849879340944, 240.5, 0.0, 0.0, 1.0]).reshape(3,3)
    distCoeffs=np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    rvec = np.array([0,0,0], np.float) # rotation vector
    tvec = np.array([0,0,0], np.float) # translation vector
    imagePoints, jac = cv2.projectPoints(position_transformed, rvec=rvec, tvec=tvec, cameraMatrix=cameraMatrix,distCoeffs=distCoeffs)
    return [imagePoints[0][0][0],imagePoints[0][0][1]]
    
  def _drawBox(self):
    (x, y, w, h) = [int(v) for v in self.box]
    # Object box
    cv2.rectangle(self.cv_image, (x, y), (x + w, y + h),
        (0, 255, 0), 2)
    # Occlusion Margin
    m=rospy.get_param("/moma_demo/occlusion_margin")
    cv2.rectangle(self.cv_image, (x - m, y - m), (x + w + m, y + h + m),
        (200, 200, 200), 2)
    # Draws a small black circle in the center of the rectangle
    cv2.circle(self.cv_image, (x+w/2, y+h/2), 2,
        (0, 0, 0), 2)
    # Draws a small whie circle in the end effector of the robot
    (trans,rot) = self.listener.lookupTransform("/fixed_camera_depth_optical_frame","/panda_default_ee", rospy.Time(0))
    ee_pose_inCAM = self.create_pose(trans,rot)
    ee_imgPoints = self.position2imgPoints(ee_pose_inCAM.position)
    cv2.circle(self.cv_image, (int(ee_imgPoints[0]), int(ee_imgPoints[1])), 2,
      (255, 255, 255), 2)
      
    # Draws a small blue circle in the link8 of the robot
    (trans,rot) = self.listener.lookupTransform("/fixed_camera_depth_optical_frame","/panda_link8", rospy.Time(0))
    ee_pose_inCAM = self.create_pose(trans,rot)
    ee_imgPoints = self.position2imgPoints(ee_pose_inCAM.position)
    cv2.circle(self.cv_image, (int(ee_imgPoints[0]), int(ee_imgPoints[1])), 2,
      (255, 0, 0), 2)

  # function used to start tracking
  # used in callbackIMG()
  def _startTracker(self):
    self.tracker = self.OPENCV_OBJECT_TRACKERS[self.algorithm]()
    self.tracker.init(self.cv_image,self.initBB)
    self._updateTracker()

  # function used to update tracking
  # used in callbackIMG()
  def _updateTracker(self):
    (self.successTracking, self.box) = self.tracker.update(self.cv_image)
    self._drawBox()
    boundingbox = BoundingBox()
    boundingbox.xmin = self.box[0]
    boundingbox.xmax = self.box[0]+self.box[2]
    boundingbox.ymin = self.box[1]
    boundingbox.ymax = self.box[1]+self.box[3]
    self.objBB_pub.publish(boundingbox)

  # used to release tracker if object lost
  def _releaseTracker(self):
    self.tracker = self.OPENCV_OBJECT_TRACKERS[self.algorithm]()

  # fuction detecting motion of centerpoint of box over "threshold_length" frames
  def _motionDetector(self):
    if self.prev_box is not None:
      # box center calculation
      bc = (self.box[0]+self.box[2]/2,self.box[1]+self.box[3]/2)
      pbc = (self.prev_box[0]+self.prev_box[2]/2,self.prev_box[1]+self.prev_box[3]/2)
      # box center distance change
      new_delta = sqrt((bc[0]-pbc[0])**2+(bc[1]-pbc[1])**2)
      # storing distance delta
      self.deltas = np.roll(self.deltas,1)
      self.deltas = np.concatenate(([new_delta], self.deltas[0:-1]))

      if self.deltas.sum()>self.motion_threshold:
        self.objRest = False
      else:
        self.objRest = True

    # update the boxes
    self.prev_box = self.box
  
  # callback function performing tracking
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

    # # uncomment if script should show images directly
    # cv2.imshow(self.winName, self.cv_image)
    # cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
      if self.objRest:
        self.objRest_pub.publish("resting")
      if not self.objRest:
        self.objRest_pub.publish("moving")
      else:
        pass
      if self.successTracking and self.successDetection:
        self.objLock_pub.publish("locked")
      else:
        self.objLock_pub.publish("loose")
        self._releaseTracker()

    except CvBridgeError as e:
      print(e)

  # callback function to collect the bounding boxes from detection
  def callbackBB(self,data):
    if data.result.success:
      targetBB = data.result.targetBB
      self.initBB = (targetBB.xmin,targetBB.ymin,abs(targetBB.xmax - targetBB.xmin),abs(targetBB.ymax - targetBB.ymin))
      self.successTracking = False
      self.successDetection = True
    else:
      self.successDetection = False

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
