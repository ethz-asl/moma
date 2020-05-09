#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('sa_reactive_execution')
import sys
import rospy
import cv2
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# caz start
import imutils
from operator import xor
import numpy as np
from math import sqrt
# caz end

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

  prev_box = None

  motion = None

  # motion storage array
  deltas = np.zeros(5)

  # global firstRound
  firstRound = True

  # object lock variable
  success = None


  def __init__(self):
    self.image_pub = rospy.Publisher("/object_tracker/image",Image, queue_size=10)
    self.motion_pub = rospy.Publisher("/object_tracker/motion",Bool, queue_size=10)
    self.objLock_pub = rospy.Publisher("/object_tracker/objLock",Bool, queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/webcam/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #prev_box = object_tracker.prev_box
    #deltas = object_tracker.deltas
    motion = object_tracker.motion

    initBB = (333, 164, 50, 50)
    #firstRound = True

    # caz start
    cv_image = imutils.resize(cv_image, width = 500)
    
    if initBB is not None and object_tracker.firstRound:
      object_tracker.tracker.init(cv_image,initBB)
      print("CAZ: init", object_tracker.firstRound)
      object_tracker.firstRound = False
      #print(type(data))
    
    if xor(bool(initBB is not None), object_tracker.firstRound):
      # grab the new bounding box coordinates of the object
      (object_tracker.success, box) = object_tracker.tracker.update(cv_image)
      #print("CAZ: update")
      #print("new box", box)

      if object_tracker.success:
        (x, y, w, h) = [int(v) for v in box]
        cv2.rectangle(cv_image, (x, y), (x + w, y + h),
            (0, 255, 0), 2)
        # Draws a small black circle in the center of the rectangle
        cv2.circle(cv_image, (x+w/2, y+h/2), 2,
            (0, 0, 0), 2)

      # detecting motion
        #print('prev_box 1:', object_tracker.prev_box)

        if object_tracker.prev_box is not None:
          # box center calculation
          bc = (box[0]+box[2]/2,box[1]+box[3]/2)
          pbc = (object_tracker.prev_box[0]+object_tracker.prev_box[2]/2,object_tracker.prev_box[1]+object_tracker.prev_box[3]/2)
          # box center distance
          new_delta = sqrt((bc[0]-pbc[0])**2+(bc[1]-pbc[1])**2)
          # print(bc)
          # print(pbc)
          # print(new_delta)
          # delta history
          object_tracker.deltas = np.roll(object_tracker.deltas,1)
          object_tracker.deltas = np.concatenate(([new_delta], object_tracker.deltas[0:-1]))
          
          tempDeltas = object_tracker.deltas

          if tempDeltas.sum()>(tempDeltas.size):
            object_tracker.motion = True

          else: object_tracker.motion = False
        else:
          #print('init detector')
          object_tracker.prev_box = box
          #print('prev box 2', object_tracker.prev_box)

        # update the boxes
        object_tracker.prev_box = box

        # Feedback to ROS by publishing
        if object_tracker.motion:
            deltasString = str(object_tracker.deltas)
            #fpsString = "{:.2f}".format(fps.fps())
            #message = ','.join(["motion", deltasString, fpsString])
            #message = ','.join([str(success),"moving", deltasString])
            message = ','.join([str(object_tracker.success),"moving"])
            print(message)
            #ros_publisher(message)
        else:
            deltasString = str(object_tracker.deltas)
            #fpsString = "{:.2f}".format(fps.fps())
            #message = ','.join(["still", deltasString, fpsString])
            message = ','.join([str(object_tracker.success),"still"])
            print(message)
            #ros_publisher(message)

      else: print("lost item")

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.motion_pub.publish(self.motion)
      self.objLock_pub.publish(self.success)
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = object_tracker()
  rospy.init_node('object_tracker', anonymous=False)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)