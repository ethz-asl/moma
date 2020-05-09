#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import (
    Header,
    String,
    UInt8
    )
from grasp_demo.msg import (
    BoundingBoxes,
    BoundingBox
    )

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class CVdetector():

    def __init__(self):
        # Variables
        self.confThreshold = 0.5                #Confidence threshold
        self.nmsThreshold = 0.4                 #Non-maximum suppression threshold
        self.inpWidth = 416                     #Width of network's input image
        self.inpHeight = 416                    #Height of network's input image
        self.bridge = CvBridge()
        self.boundBoxTL = 0
        self.cv_image = None                    # Will be cv Mat

        self.boundingBox = None
        self.boundingBoxes = BoundingBoxes()

        # CV Window settings
        self.winName = 'Deep learning object detection in OpenCV'
        cv2.namedWindow(self.winName, cv2.WINDOW_NORMAL)
        
        # DNN
        # Load names of classes
        classesFile = "/home/zinnerc/catkin_ws/src/moma/moma_demos/grasp_demo/src/yolo/coco.names"
        self.classes = None
        with open(classesFile, 'rt') as f:
            self.classes = f.read().rstrip('\n').split('\n')

        # Give the configuration and weight files for the model and load the network using them.
        modelConfiguration = "/home/zinnerc/catkin_ws/src/moma/moma_demos/grasp_demo/src/yolo/yolov3.cfg"
        modelWeights = "/home/zinnerc/catkin_ws/src/moma/moma_demos/grasp_demo/src/yolo/yolov3.weights"

        self.net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        # Subscriber
        self.image_sub = rospy.Subscriber("/webcam/image_raw",Image,self.callback_Yolo, queue_size=1)
        
        # Publisher
        self.boundBoxes_pub = rospy.Publisher("/grasp_demo/BoundingBoxes",BoundingBoxes,queue_size=1)

    def resizeVid(cap,width,height):
        cap.set(3,width)
        cap.set(4,height)
        return cap

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

    def callback_Yolo(self,data):
        r = rospy.Rate(3)
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Create a 4D blob from a frame.
        blob = cv2.dnn.blobFromImage(self.cv_image, 1/float(255), (self.inpWidth, self.inpHeight), [0,0,0], 1, crop=False)
        # Sets the input to the network
        self.net.setInput(blob)

        # Runs the forward pass to get output of the output layers
        outs = self.net.forward(self.getOutputsNames(self.net))

        # Remove the bounding boxes with low confidence
        self.postprocess(outs)

        # Put efficiency information. The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
        t, _ = self.net.getPerfProfile()
        label = 'Inference time: %.2f ms' % (t * 1000.0 / cv2.getTickFrequency())
        cv2.putText(self.cv_image, label, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255))

        cv2.imshow(self.winName, self.cv_image)
        cv2.waitKey(3)

        try:
            self.boundingBoxes.header.stamp = rospy.Time.now()
            self.boundingBoxes.header.frame_id = 'header_frame_id'
            self.boundBoxes_pub.publish(self.boundingBoxes)
            self.boundingBoxes.bounding_box = []
        except CvBridgeError as e:
            print(e)
        r.sleep()

def main():
    rospy.init_node('CVdetector', anonymous=False)
    ic = CVdetector()
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
