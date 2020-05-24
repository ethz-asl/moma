#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import (
    Header,
    String,
    UInt8
    )
# from grasp_demo.msg import (
#     BoundingBoxes,
#     BoundingBox,
#     DetectionAction,
#     DetectionActionFeedback,
#     DetectionActionGoal,
#     DetectionActionResult,
#     DetectionFeedback,
#     DetectionGoal,
#     DetectionResult
#     )

from grasp_demo.msg import *

from actionlib import SimpleActionServer

from grasp_demo.msg import DetectionAction, DetectionResult
from grasp_demo.utils import create_robot_connection

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class yolo_action(object):

    # create messages that are used to publish feedback/result
    _feedback = grasp_demo.msg.DetectionFeedback()
    _result = grasp_demo.msg.DetectionResult()

    # Variables
    confThreshold = 0.5                #Confidence threshold
    nmsThreshold = 0.4                 #Non-maximum suppression threshold
    inpWidth = 416                     #Width of network's input image
    inpHeight = 416                    #Height of network's input image
    bridge = CvBridge()
    boundBoxTL = 0
    cv_image = None                    # Will be cv Mat

    # DNN
    # Load names of classes
    classesFile = "/home/zinnerc/catkin_ws/src/moma/moma_demos/grasp_demo/src/yolo/coco.names"
    classes = None
    with open(classesFile, 'rt') as f:
        classes = f.read().rstrip('\n').split('\n')

    # Give the configuration and weight files for the model and load the network using them.
    modelConfiguration = "/home/zinnerc/catkin_ws/src/moma/moma_demos/grasp_demo/src/yolo/yolov3.cfg"
    modelWeights = "/home/zinnerc/catkin_ws/src/moma/moma_demos/grasp_demo/src/yolo/yolov3.weights"

    # CV Window settings
    winName = 'Object Detection'
    cv2.namedWindow(winName, cv2.WINDOW_NORMAL)

    def __init__(self,name):
        self.robot_name = sys.argv[1]
        # self._load_parameters()
        # self._connect_robot()

        # self._action_name = name
        self._as = SimpleActionServer(
            "detection_action", DetectionAction, execute_cb=self.BBcheck_cb, auto_start = False
        )
        self._as.start()
        rospy.loginfo("Detection action server ready")

        self.boundingBox = None
        self.boundingBoxes = BoundingBoxes()

        # Subscriber
        # self.image_sub = rospy.Subscriber("/webcam/image_raw",Image,self.callback_IMG, queue_size=1)
        self.image_sub = rospy.Subscriber("/fixed_camera/color/image_raw",Image,self.callback_IMG, queue_size=1)

        self.net = cv2.dnn.readNetFromDarknet(self.modelConfiguration, self.modelWeights)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    # def _connect_robot(self):
    #     full_robot_name = (
    #         self.robot_name + "_" + self._robot_arm_names[0]
    #         if len(self._robot_arm_names) > 1
    #         else self.robot_name
    #     )
    #     self._robot_arm = create_robot_connection(full_robot_name)

    # Get the names of the output layers
    def getOutputsNames(self,net):
        # Get the names of all the layers in the network
        layersNames = net.getLayerNames()
        # Get the names of the output layers, i.e. the layers with unconnected outputs
        return [layersNames[i[0] - 1] for i in net.getUnconnectedOutLayers()]

    # Draw the predicted bounding box
    def drawPred(self,classId, conf, left, top, right, bottom):
        # Draw a bounding box.
        cv2.rectangle(self.cv_image, (left, top), (right, bottom), (255, 178, 50), 3)
        
        label = '%.2f' % conf
            
        # Get the label for the class name and its confidence
        if self.classes:
            assert(classId < len(self.classes))
            label = '%s:%s' % (self.classes[classId], label)

        #Display the label at the top of the bounding box
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, labelSize[1])
        cv2.rectangle(self.cv_image, (left, int(top - round(1.5*labelSize[1]))), (int(left + round(1.5*labelSize[0])), top + baseLine), (255, 255, 255), cv2.FILLED)
        cv2.putText(self.cv_image, label, (left, top), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,0), 1)

    # Remove the bounding boxes with low confidence using non-maxima suppression
    def postprocess(self, outs):
        frameHeight = self.cv_image.shape[0]
        frameWidth = self.cv_image.shape[1]

        # Scan through all the bounding boxes output from the network and keep only the
        # ones with high confidence scores. Assign the box's class label as the class with the highest score.
        classIds = []
        confidences = []
        boxes = []
        i=0
        for out in outs:
            for detection in out:
                scores = detection[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > self.confThreshold:
                    center_x = int(detection[0] * frameWidth)
                    center_y = int(detection[1] * frameHeight)
                    width = int(detection[2] * frameWidth)
                    height = int(detection[3] * frameHeight)
                    left = int(center_x - width / 2)
                    top = int(center_y - height / 2)
                    classIds.append(classId)
                    confidences.append(float(confidence))
                    boxes.append([left, top, width, height])
                    i=i+1

        # Perform non maximum suppression to eliminate redundant overlapping boxes with
        # lower confidences.
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confThreshold, self.nmsThreshold)
        for i in indices:
            i = i[0]
            box = boxes[i]
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]
            self.drawPred(classIds[i], confidences[i], left, top, left + width, top + height)

            self.boundingBox = BoundingBox()

            self.boundingBox.probability = confidences[i]
            self.boundingBox.xmin = int(left)
            self.boundingBox.ymin = int(top+height)
            self.boundingBox.xmax = int(left+width)
            self.boundingBox.ymax = int(top)
            self.boundingBox.id = int(classIds[i])
            self.boundingBox.Class = self.classes[classIds[i]]

            self.boundingBoxes.bounding_box.append(self.boundingBox)

    def callback_IMG(self,data):
        try:
            # rospy.loginfo('callback_IMG 1')
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def yolo_detector(self):
        self.boundingBoxes.bounding_box = []
        # Create a 4D blob from a frame.
        # rospy.loginfo('yolo_detector 1')
        blob = cv2.dnn.blobFromImage(self.cv_image, 1/float(255), (self.inpWidth, self.inpHeight), [0,0,0], 1, crop=False)
        # Sets the input to the network
        # rospy.loginfo('yolo_detector 2')
        self.net.setInput(blob)

        # Runs the forward pass to get output of the output layers
        # rospy.loginfo('yolo_detector 3')
        outs = self.net.forward(self.getOutputsNames(self.net))

        # Remove the bounding boxes with low confidence
        # rospy.loginfo('yolo_detector 4')
        self.postprocess(outs)

        # Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
        # t, _ = self.net.getPerfProfile()
        # label = 'Inference time: %.2f ms' % (t * 1000.0 / cv2.getTickFrequency())
        # cv2.putText(self.cv_image, label, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))

        # cv2.imshow(self.winName, self.cv_image)
        # cv2.waitKey(3)

        try:
            self.boundingBoxes.header.stamp = rospy.Time.now()
            self.boundingBoxes.header.frame_id = 'header_frame_id'
            return self.boundingBoxes
        except CvBridgeError as e:
            print(e)

    def BBcheck_cb(self, goal):
        success = False
        # self._result.targetBB = [None]*4
        self._result.targetBB = None
        rospy.loginfo('detection started')

        self._feedback.in_progress = True
        # rospy.loginfo('BBcheck_cb 1')
        
        interm_result = self.yolo_detector()
        self._feedback.detectedBB = interm_result
        rospy.loginfo('BBcheck_cb 2')
        rospy.loginfo('detection finished, %s objects present',len(interm_result.bounding_box))
        for box in interm_result.bounding_box:
            rospy.loginfo('  - %s',box.Class)
        # for item in interm_result.bounding_box:
        #     rospy.loginfo('  - %s',item.Class)
        # rospy.loginfo('start looking for %s' ,goal.name)
        # rospy.loginfo('start looking for orange")
        # rospy.loginfo('BBcheck_cb 4')
        i = 1
        # for box in self._feedback.detectedBB.bounding_box:
        rospy.loginfo(goal)
        for box in interm_result.bounding_box:
            # if box.Class == goal.name: # and self.request:
            if box.Class == "orange": # and self.request:
                self._result.targetBB = box
                # self._result.probability = box.probability
                # self._result.targetBB[0] = box.xmin
                # self._result.targetBB[1] = box.ymin
                # self._result.targetBB[2] = box.xmax
                # self._result.targetBB[3] = box.ymax
                self._result.success = True
                success = True
                self._feedback.in_progress = False
                i += 1
        # rospy.loginfo('point 5')

        if success:
            self._as.set_succeeded(self._result)
            rospy.loginfo("object %s %s found in this frame with prob. %s" ,i ,goal,self._result.targetBB.probability)
            # rospy.loginfo("BoundingBox: [%d,%d,%d,%d] " ,self._result.targetBB[0],self._result.targetBB[1],self._result.targetBB[2],self._result.targetBB[3])
            # rospy.loginfo("BoundingBox: {0}" ,self._result.targetBB.xmin)
            success = False
        else:
            self._result.success = False
            rospy.loginfo("object not found in this frame")

        print("-")*10


def main():
    rospy.init_node('yolo_action', anonymous=False)
    server = yolo_action(rospy.get_name())
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass