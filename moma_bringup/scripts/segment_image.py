#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from std_srvs.srv import Empty, EmptyResponse
from ros_sam_msgs.srv import Segmentation, SegmentationRequest
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32MultiArray

import cv2
import numpy as np
import copy

class ImageSegmentationNode:
    def __init__(self):
        # Initialize node
        rospy.init_node('image_segmentation_node')

        # Image subscriber and storage
        self.image_sub = rospy.Subscriber("/rs_435_1/color/image_raw", Image, self.image_callback)
        self.stored_image = None
        self.stored_image_viz = None

        # Mouse click subscriber and storage
        self.click_sub = rospy.Subscriber("/select_control_pts/mouse_click", PointStamped, self.click_callback)
        self.clicks = []

        # Publishers
        self.image_pub = rospy.Publisher('/select_control_pts', Image, queue_size=10)
        self.mask_pub = rospy.Publisher('/mask', Image, queue_size=10)

        # Services
        self.store_image_srv = rospy.Service('store_image', Empty, self.store_image)
        self.reset_clicks_srv = rospy.Service('reset_clicks', Empty, self.reset_clicks)
        self.segment_srv = rospy.Service('segment', Empty, self.segment_image)

        # CVBridge for image conversion
        self.bridge = CvBridge()

    def image_callback(self, msg):
        """Callback to update the most recent image."""
        self.recent_image = msg

    def store_image(self, req):
        """Store the most recent image and create a visualization version."""
        if self.recent_image is not None:
            self.stored_image = self.recent_image
            try:
                # Convert the image to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(self.stored_image, "bgr8")

                # Create a copy for visualization
                self.stored_image_viz = cv_image.copy()

                # Draw the buffered clicks on the image
                # for click in self.clicks:
                #     cv2.circle(self.stored_image_viz, (int(click[0]), int(click[1])), 5, (0, 255, 0), -1)

                # Convert back to ROS image
                self.stored_image_viz = self.bridge.cv2_to_imgmsg(self.stored_image_viz, "bgr8")
                
                # Publish the visualization image
                self.image_pub.publish(self.stored_image_viz)
                rospy.loginfo("Stored image and visualized control points published successfully")
            
            except CvBridgeError as e:
                rospy.logerr(f"Failed to process image: {e}")
        else:
            rospy.logwarn("No image to store")
        return EmptyResponse()

    def click_callback(self, msg):
        """Store only x and y coordinates from incoming clicks."""
        rospy.loginfo(f"Mouse click received at: {msg.point.x}, {msg.point.y}")
        self.clicks.append((msg.point.x, msg.point.y))
        self.stored_image_viz = self.bridge.imgmsg_to_cv2(self.stored_image_viz, "bgr8")
        # Draw the buffered clicks on the image
        for click in self.clicks:
            cv2.circle(self.stored_image_viz, (int(click[0]), int(click[1])), 5, (0, 255, 0), -1)
        # Convert back to ROS image
        self.stored_image_viz = self.bridge.cv2_to_imgmsg(self.stored_image_viz, "bgr8")
        
        # Publish the visualization image
        self.image_pub.publish(self.stored_image_viz)

    def reset_clicks(self, req):
        """Reset the buffer of click points."""
        self.clicks = []
        rospy.loginfo("Clicks reset successfully")
        return EmptyResponse()

    def segment_image(self, req):
        """Call segmentation service with stored image and buffered clicks."""
        if self.stored_image is None:
            rospy.logwarn("No stored image to segment")
            return EmptyResponse()

        if not self.clicks:
            rospy.logwarn("No control points to segment with")
            return EmptyResponse()

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
        node = ImageSegmentationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
