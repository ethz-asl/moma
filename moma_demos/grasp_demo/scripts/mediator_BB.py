#!/usr/bin/env python
import rospy

from grasp_demo.msg import BoundingBoxes, BoundingBox
from std_msgs.msg import String, Bool

class mediator():

    def __init__(self):
        # Publishers
        self.track_BB_pub = rospy.Publisher("/grasp_demo/BB_Goal",BoundingBox,queue_size=1)

        # Subscribers
        self.detect_BB_sub = rospy.Subscriber("/grasp_demo/BoundingBoxes",BoundingBoxes,self.callback_Detector)
        self.goal_obj_sub = rospy.Subscriber("/grasp_demo/GoalObject",String,self.callback_ObjectValidation)
        self.med_request_sub = rospy.Subscriber("/py_trees_BlBo/med_req",Bool,self.callback_Request)

        # Variables
        self.goal_obj = None        # Will be String
        self.valid_obj = None       # Will be Bool
        self.detections = None      # Will be List

        self.boundingBoxes = None   # Will be BoundingBoxes
        self.obj_BB = None          # Will be Boundingbox
        self.tracker_BB = None      # Will be Tuple

        self.request = False
        self.detection = False

        # Load names of classes
        # Used to check if goal feasible
        classesFile = "/home/zinnerc/catkin_ws/src/moma/moma_demos/grasp_demo/src/yolo/coco.names"
        self.classes = None
        with open(classesFile, 'rt') as f:
            self.classes = f.read().rstrip('\n').split('\n')

    def callback_Detector(self,data):
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes = data
        self.detection = True

    def callback_Request(self,data):
        self.request = data

    def callback_ObjectValidation(self,data):
        self.goal_obj = data.data

        if self.goal_obj in self.classes:
            self.valid_obj = True
        else:
            self.valid_obj = False
            print("object not valid")
        
        if self.valid_obj and self.detection:
            self._BB_selection()
            #self._BB_transformation()

    def _BB_selection(self):
        i = 0
        index = 0
        for box in self.boundingBoxes.bounding_box:
            if box.Class == self.goal_obj and self.request:
                print(self.goal_obj)
                print("found")
                self.track_BB_pub.publish(box)


def main():
    rospy.init_node('Mediator_BB', anonymous=False)
    ic = mediator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
