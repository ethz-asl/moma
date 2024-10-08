#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from ros_sam_msgs.srv import Segmentation, SegmentationRequest
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32MultiArray
from grid_map_msgs.msg import GridMap
from std_msgs.msg import String, Float32
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

import matplotlib.pyplot as plt

import cv2
import numpy as np
import copy
import struct

class MomaUiNode:
    def __init__(self):
        # Initialize node
        rospy.init_node('image_segmentation_node')

        self.input_mode = rospy.get_param('~input_mode', 'elevation_map') # 'image' or 'elevation_map'
        assert self.input_mode in ['image', 'elevation_map'], "Invalid input mode. Choose 'image' or 'elevation_map'"

        # Image subscriber and storage
        self.last_received_img = None
        self.image_sub = rospy.Subscriber("/rs_435_1/color/image_raw", Image, self.image_callback)
        self.last_elevation_map = None
        self.elevation_map_sub = rospy.Subscriber("/elevation_mapping/elevation_map", GridMap, self.elevation_map_callback)

        # Mouse click subscriber and storage
        self.control_image = None
        self.click_sub = rospy.Subscriber("/moma_ui/sam/control_image/mouse_click", PointStamped, self.click_callback)
        self.control_points_xy = []
        self.control_points_label = []
        self.last_mask = None

        # label subscriber and storage
        self.fg_min_height_sub = rospy.Subscriber("moma_ui/sam/foreground_min_height", Float32, self.fg_min_height_callback)
        
        # self.label_list = None
        self.current_label = 'fg'
        self.fg_min_height = 0.0

        # Publishers
        self.control_img_pub = rospy.Publisher('moma_ui/sam/control_image', Image, queue_size=10)
        self.mask_pub = rospy.Publisher('moma_ui/sam/mask_image', Image, queue_size=10)
        self.masked_pub = rospy.Publisher('moma_ui/sam/masked_image', Image, queue_size=10)
        self.filtered_elevation_map_pub = rospy.Publisher('moma_ui/sam/filtered_elevation_map', GridMap, queue_size=10)
        self.elev_map_img_pub = rospy.Publisher('moma_ui/sam/elevation_map_image', Image, queue_size=10)
        # Services
        self.reset_sam_cfg_srv = rospy.Service('moma_ui/sam/reset', Empty, self.reset_sam_config)
        self.run_sam_srv = rospy.Service('moma_ui/sam/run', Trigger, self.run_sam)
        self.set_label_fg_bg_srv = rospy.Service('moma_ui/sam/set_label_fg_bg', SetBool, self.set_label_fg_bg)
        # CVBridge for image conversion
        self.bridge = CvBridge()

    # callbacks
    def image_callback(self, msg):
        """Callback to update the most recent image."""
        if self.input_mode == 'image':
            self.last_received_img = msg
    
    def elevation_map_callback(self, msg):
        self.last_elevation_map = msg
        num_rows = msg.data[0].layout.dim[0].size
        num_cols = msg.data[0].layout.dim[1].size
        color_layer = np.array(msg.data[msg.layers.index('color')].data).reshape((num_rows, num_cols))
        # mask out all nan and inf values
        color_layer[np.isnan(color_layer)] = 0
        color_layer[np.isinf(color_layer)] = 0
        # convert color_img to rgb image
        color_img = np.zeros((num_rows, num_cols, 3), dtype=np.uint8)
        for i in range(num_rows):
            for j in range(num_cols):
                color = color_layer[i, j]
                # convert 
                color_raw = struct.unpack('I', struct.pack('f', color))[0]
                b = (color_raw >> 16) & 0x0000ff
                g = (color_raw >> 8) & 0x0000ff
                r =  color_raw & 0x0000ff
                color_img[i, j] = [r, g, b]
        ros_image = self.bridge.cv2_to_imgmsg(color_img, encoding="bgr8")
        self.elev_map_img_pub.publish(ros_image)
        # if elev_map mode, store the it as the last received image
        if self.input_mode == 'elevation_map':
            self.last_received_img = ros_image
            if self.last_mask is not None:
                elevation_layer = np.array(msg.data[msg.layers.index('elevation')].data).reshape((num_rows, num_cols))
                elevation_layer[~self.last_mask] = 0.0
                msg.data[msg.layers.index('elevation')].data = elevation_layer.flatten().tolist()
                self.filtered_elevation_map_pub.publish(msg)
            else:
                self.filtered_elevation_map_pub.publish(msg)

    def rgba_to_bgr(self, color):
        # Color comes as (R, G, B, A), we ignore A and multiply RGB by 255 for OpenCV
        r, g, b, _ = color
        return (int(b * 255), int(g * 255), int(r * 255))

    def click_callback(self, msg):
        """Store only x and y coordinates from incoming clicks."""
        if self.control_image is None:
            rospy.logwarn("moma_ui: No stored image to click on")
            return
        if msg is not None:
            self.control_points_xy.append((msg.point.x, msg.point.y))
            if self.current_label == 'fg':
                self.control_points_label.append(1)
            else:
                self.control_points_label.append(0)
        control_img_cv2 = self.bridge.imgmsg_to_cv2(self.control_image, "bgr8") 
        # Draw the buffered clicks on the image
        if self.control_points_xy is not None and len(self.control_points_xy) > 0:
            i = 0
            for i in range(len(self.control_points_xy)):
                # print('label_list:', self.label_list)
                click_xy = self.control_points_xy[i]
                label = self.control_points_label[i]
                color = self.rgba_to_bgr(plt.cm.tab20(label))
                if self.input_mode == 'elevation_map':
                    circle_radius = 1
                elif self.input_mode == 'image':
                    circle_radius = 5
                cv2.circle(control_img_cv2, (int(click_xy[0]), int(click_xy[1])), circle_radius, color, -1)
        # Convert back to ROS image
        control_img_msg = self.bridge.cv2_to_imgmsg(control_img_cv2, "bgr8")       
        # Publish the control points overlaid on the control image
        self.control_img_pub.publish(control_img_msg)

    def fg_min_height_callback(self, msg):
        rospy.loginfo(f"moma_ui: Received foreground min height: {msg.data}")
        self.fg_min_height = msg.data

    # services
    def reset_sam_config(self, req):
        rospy.loginfo("moma_ui: Resetting SAM control image, points...")
        """Reset the buffer of click points."""
        self.control_points_xy = []
        self.control_points_label = []
        self.last_mask = None
        if self.last_received_img is not None:
            self.control_image = self.last_received_img
            self.control_img_pub.publish(self.control_image)
        return EmptyResponse()

    def set_label_fg_bg(self, req):
        rospy.loginfo("moma_ui: Setting label to foreground or background")
        """Set the label to either foreground or background"""
        if req.data:
            self.current_label = 'fg'
            rospy.loginfo("Label set to foreground")
        else:
            self.current_label = 'bg'
            rospy.loginfo("Label set to background")
        return SetBoolResponse(success=True, message="Label set successfully")

    def run_sam(self, req):
        rospy.loginfo("moma_ui: Segmenting image...")
        """Call segmentation service with stored image and buffered clicks."""
        if self.control_image is None:
            rospy.logwarn("moma_ui: No control image to segment")
            resp = TriggerResponse()
            resp.success = False
            resp.message = "No control image to segment!"
            return resp

        if not self.control_points_xy:
            rospy.logwarn("moma_ui: No control points to segment with")
            resp = TriggerResponse()
            resp.success = False
            resp.message = "No control points to segment with!"     
            return resp
        
        if len(self.control_points_xy) != len(self.control_points_label):
            rospy.logwarn("moma_ui: Number of points and labels do not match")
            resp = TriggerResponse()
            resp.success = False
            resp.message = "Number of points and labels do not match!"
            return resp

        # Prepare the service call
        rospy.wait_for_service('/ros_sam/segment')  # Ensure service is available
        try:
            segmentation_srv = rospy.ServiceProxy('/ros_sam/segment', Segmentation)

            # Create segmentation request
            seg_req = SegmentationRequest()
            seg_req.image = self.control_image  # Stored image

            # fill up the query points and labels
            for i in range(len(self.control_points_xy)):
                query_point = Point(x=self.control_points_xy[i][0], y=self.control_points_xy[i][1], z=0)
                seg_req.query_points.append(query_point)
                label = self.control_points_label[i]
                seg_req.query_labels.append(label)

            # Specify the TL and BR corners using Int32MultiArray
            boxes = Int32MultiArray()
            boxes.data = [0, 0, 1200, 680]
            seg_req.boxes = boxes  # Add boxes to the segmentation request

            # Call segmentation service
            rospy.loginfo("Calling segmentation service...")
            response = segmentation_srv(seg_req)

            # Process the response
            if response.masks:
                img_masked = self.bridge.imgmsg_to_cv2(self.control_image).copy()
                # convert the image to 
                rospy.loginfo(f"Received {len(response.masks)} masks from segmentation service")
                for mask in response.masks:
                    actual_mask = self.bridge.imgmsg_to_cv2(mask, desired_encoding='mono8')
                    boolean_array = actual_mask.astype(bool)
                    self.last_mask = boolean_array
                    # mask the image where the mask is false
                    img_masked[~boolean_array] = 0              

                img_masked = self.bridge.cv2_to_imgmsg(img_masked, "bgr8")
                mask_image = response.masks[0]
                self.mask_pub.publish(mask_image)
                self.masked_pub.publish(img_masked)
                # self.mask_pub.publish(first_mask)
                rospy.loginfo("Published the first mask from the segmentation response")
            else:
                rospy.logwarn("No masks received from segmentation service")

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

        resp = TriggerResponse()
        resp.success = True
        resp.message = "Segmentation successful"
        return resp
    

if __name__ == '__main__':
    try:
        node = MomaUiNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
