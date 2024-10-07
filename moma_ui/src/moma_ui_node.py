#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from ros_sam_msgs.srv import Segmentation, SegmentationRequest
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32MultiArray
from grid_map_msgs.msg import GridMap
from std_msgs.msg import String

import cv2
import numpy as np
import copy

class MomaUiNode:
    def __init__(self):
        # Initialize node
        rospy.init_node('image_segmentation_node')

        # ros params
        self.input_mode = rospy.get_param('~input_mode', 'image') # 'image' or 'grid_map
        assert self.input_mode in ['image', 'grid_map'], "Invalid input_mode. Must be 'image' or 'grid_map'"
        self.update_ctrl_img = True

        # Image subscriber and storage
        if self.input_mode == 'image':
            self.last_image = None
            self.image_sub = rospy.Subscriber("/rs_435_1/color/image_raw", Image, self.image_callback)
        elif self.input_mode == 'grid_map':
            self.last_elevation_map = None
            self.elevation_map_sub = rospy.Subscriber("/elevation_mapping/elevation_map", GridMap, self.elevation_map_callback)

        # variables
        self.stored_image = None

        # Mouse click subscriber and storage
        self.control_image = None
        self.click_sub = rospy.Subscriber("/control_image/mouse_click", PointStamped, self.click_callback)
        self.clicks = []

        # label subscriber and storage
        self.label_sub = rospy.Subscriber("moma_ui/sam/label", String, self.label_callback)
        self.label_list = None

        # Publishers
        self.control_img_pub = rospy.Publisher('/control_image', Image, queue_size=10)
        self.mask_pub = rospy.Publisher('/mask', Image, queue_size=10)

        # Services
        self.reset_sam_cfg_srv = rospy.Service('moma_ui/sam/reset', Empty, self.reset_sam_config)
        self.run_sam_srv = rospy.Service('moma_ui/sam/run', Trigger, self.run_sam)

        # CVBridge for image conversion
        self.bridge = CvBridge()

    # callbacks
    def image_callback(self, msg):
        """Callback to update the most recent image."""
        self.last_image = msg
        if self.update_ctrl_img == True:
            self.control_image = self.last_image
            self.control_img_pub.publish(self.last_image)
            self.update_ctrl_img = False
    
    def elevation_map_callback(self, msg):
        # assert not implemented
        raise NotImplementedError("Elevation map callback not implemented yet")

    def click_callback(self, msg):
        """Store only x and y coordinates from incoming clicks."""
        if msg is not None:
            self.clicks.append((msg.point.x, msg.point.y))
        control_img_cv2 = self.bridge.imgmsg_to_cv2(self.control_image, "bgr8") 
        # Draw the buffered clicks on the image
        if self.clicks is not None and len(self.clicks) > 0:
            for click in self.clicks:
                cv2.circle(control_img_cv2, (int(click[0]), int(click[1])), 5, (0, 255, 0), -1)
        # Convert back to ROS image
        control_img_msg = self.bridge.cv2_to_imgmsg(control_img_cv2, "bgr8")       
        # Publish the control points overlaid on the control image
        self.control_img_pub.publish(control_img_msg)

    def label_callback(self, msg):
        # check if label dictionary is empty
        if self.label_list is None:
            self.label_list = []
        # check if the label is already in the dictionary
        if msg.data not in self.label_list:
            self.label_list.append(msg.data)
        rospy.loginfo(f"moma_ui: Added label {msg.data} to the label list, with labels: {self.label_list}")

    # services
    def reset_sam_config(self, req):
        rospy.loginfo("moma_ui: Resetting SAM control image, points...")
        """Reset the buffer of click points."""
        self.clicks = []
        self.update_ctrl_img = True
        self.label_list = None
        if self.control_image is not None:
            self.control_img_pub.publish(self.control_image)
        return EmptyResponse()

    def run_sam(self, req):
        rospy.loginfo("moma_ui: Segmenting image...")
        """Call segmentation service with stored image and buffered clicks."""
        if self.stored_image is None:
            rospy.logwarn("moma_ui: No stored image to segment")
            resp = TriggerResponse()
            resp.success = False
            resp.message = "No stored image to segment!"
            return resp

        if not self.clicks:
            rospy.logwarn("moma_ui: No control points to segment with")
            resp = TriggerResponse()
            resp.success = False
            resp.message = "No control points to segment with!"     
            return resp

        # Prepare the service call
        rospy.wait_for_service('/ros_sam/segment')  # Ensure service is available
        try:
            segmentation_srv = rospy.ServiceProxy('/ros_sam/segment', Segmentation)

            # Create segmentation request
            seg_req = SegmentationRequest()
            seg_req.image = self.stored_image  # Stored image

            # Convert clicks to geometry_msgs/Point[]
            seg_req.query_points = [Point(x=click[0], y=click[1], z=0) for click in self.clicks]

            # Add labels (all '1')
            seg_req.query_labels = [1] * len(self.clicks)

            # Specify the TL and BR corners using Int32MultiArray
            boxes = Int32MultiArray()
            boxes.data = [0, 0, 1200, 680]
            seg_req.boxes = boxes  # Add boxes to the segmentation request

            # Call segmentation service
            rospy.loginfo("Calling segmentation service...")
            response = segmentation_srv(seg_req)

            # Process the response
            if response.masks:
                # img_masked = cv2.cvtColor(self.bridge.imgmsg_to_cv2(self.stored_image), cv2.COLOR_BGR2RGB)
                img_masked = cv2.cvtColor(self.bridge.imgmsg_to_cv2(self.stored_image), cv2.COLOR_BGR2RGB)
                # convert the image to 
                for mask in response.masks:
                    actual_mask = self.bridge.imgmsg_to_cv2(mask, desired_encoding='mono8')
                    boolean_array = actual_mask.astype(bool)
                    num_true = np.sum(boolean_array)
                    num_false = np.sum(~boolean_array)
                    print(f"Number of True pixels: {num_true}")
                    print(f"Number of False pixels: {num_false}")
                    # mask the image where the mask is false
                    img_masked[~boolean_array] = 0
                

                img_masked = self.bridge.cv2_to_imgmsg(img_masked, "bgr8")
                self.mask_pub.publish(img_masked)
                # self.mask_pub.publish(first_mask)
                rospy.loginfo("Published the first mask from the segmentation response")
            else:
                rospy.logwarn("No masks received from segmentation service")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

        return EmptyResponse()

if __name__ == '__main__':
    try:
        node = MomaUiNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
