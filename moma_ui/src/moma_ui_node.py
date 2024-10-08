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
from visualization_msgs.msg import Marker, MarkerArray

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

        # stuff
        self.workplane_frame = rospy.get_param('~workplane_frame', 'workplane')
        self.last_marker_msg = None
        self.fg_is_positive = rospy.get_param('~fg_is_positive', False)
        
        # label subscriber and storage
        self.fg_min_height_sub = rospy.Subscriber("moma_ui/sam/foreground_min_height", Float32, self.fg_min_height_callback)
        
        # self.label_list = None
        self.current_label = 'positive'
        self.fg_min_height = 10.0

        # Publishers
        self.control_img_pub = rospy.Publisher('moma_ui/sam/control_image', Image, queue_size=10)
        self.mask_pub = rospy.Publisher('moma_ui/sam/mask_image', Image, queue_size=10)
        self.masked_pub = rospy.Publisher('moma_ui/sam/masked_image', Image, queue_size=10)
        self.filtered_elevation_map_pub = rospy.Publisher('moma_ui/sam/filtered_elevation_map', GridMap, queue_size=10)
        self.elev_map_img_pub = rospy.Publisher('moma_ui/sam/elevation_map_image', Image, queue_size=10)
        self.viz_marker_array_pub = rospy.Publisher('moma_ui/viz_marker_array', MarkerArray, queue_size=10)
        
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
                if self.fg_is_positive:
                    elevation_layer[~self.last_mask] = 0.0
                else:
                    elevation_layer[self.last_mask] = 0.0
                msg_copy = copy.deepcopy(msg)
                msg_copy.data[msg.layers.index('elevation')].data = elevation_layer.flatten().tolist()
                self.filtered_elevation_map_pub.publish(msg_copy)
            else:
                self.filtered_elevation_map_pub.publish(msg)
        
        # marker
        if self.last_marker_msg is not None:
            self.viz_marker_array_pub.publish(self.last_marker_msg)

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
            if self.current_label == 'positive':
                self.control_points_label.append(1)
            elif self.current_label == 'negative':
                self.control_points_label.append(0)
            else:
                rospy.logwarn("moma_ui: Unknown label type")
                return
        control_img_cv2 = self.bridge.imgmsg_to_cv2(self.control_image, "bgr8") 
        # Draw the buffered clicks on the image
        if self.control_points_xy is not None and len(self.control_points_xy) > 0:
            i = 0
            for i in range(len(self.control_points_xy)):
                # print('label_list:', self.label_list)
                click_xy = self.control_points_xy[i]
                label = self.control_points_label[i]
                # color = self.rgba_to_bgr(plt.cm.tab20(label))
                if label == 1:
                    color = (0, 255, 0)
                else:
                    color = (0, 0, 255)
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
        # create a marker array to visualize the fg_min_height as a plane
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.workplane_frame
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.id = 0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = self.fg_min_height
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 0.001
        marker.color.a = 0.3
        marker.color.r = 0
        marker.color.g = 1
        marker.color.b = 0
        marker_array.markers.append(marker)
        self.viz_marker_array_pub.publish(marker_array)

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
        rospy.loginfo("moma_ui: Setting label to POSITIVE or NEGATIVE")
        """Set the label to either foreground or background"""
        if req.data:
            self.current_label = 'positive'
            rospy.loginfo("Label set to POSITIVE")
        else:
            self.current_label = 'negative'
            rospy.loginfo("Label set to NEGATIVE")
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

        # if not self.control_points_xy:
        #     rospy.logwarn("moma_ui: No control points to segment with")
        #     resp = TriggerResponse()
        #     resp.success = False
        #     resp.message = "No control points to segment with!"     
        #     return resp
        
        # if len(self.control_points_xy) != len(self.control_points_label):
        #     rospy.logwarn("moma_ui: Number of points and labels do not match")
        #     resp = TriggerResponse()
        #     resp.success = False
        #     resp.message = "Number of points and labels do not match!"
        #     return resp

        ###########################################################
        ## in last received height map, filter out the points that are above the fg_min_height
        elevation_layer = np.array(self.last_elevation_map.data[self.last_elevation_map.layers.index('elevation')].data).reshape((self.last_elevation_map.data[0].layout.dim[0].size, self.last_elevation_map.data[0].layout.dim[1].size))
        # iterate over this array and find the cells that are above the fg_min_height
        control_points_xy_height = []
        control_points_label_height = []
        cnt = 0
        for i in range(elevation_layer.shape[0]):
            for j in range(elevation_layer.shape[1]):
                if elevation_layer[i, j] > self.fg_min_height:
                    control_points_xy_height.append((i, j))
                    if self.fg_is_positive:
                        # print('fg_is_positive:', self.fg_is_positive)
                        control_points_label_height.append(1)
                        cnt += 1
                    else:
                        # print('fg_is_positive:', self.fg_is_positive)
                        control_points_label_height.append(0)
                        cnt += 1
        rospy.loginfo(f"Found {cnt} points above the fg_min_height")
                    
        ## visualize both set of control on control image
        control_img_cv2 = self.bridge.imgmsg_to_cv2(self.control_image, "bgr8")
        # Draw the buffered clicks on the image
        circle_radius = 5
        if control_points_xy_height is not None and len(control_points_xy_height) > 0:
            rospy.loginfo(f"Drawing {len(control_points_xy_height)} control points on the image from height map")
            i = 0
            for i in range(len(control_points_xy_height)):
                click_xy = control_points_xy_height[i]
                label = control_points_label_height[i]
                # color = self.rgba_to_bgr(plt.cm.tab20(label))
                # choose color as blue
                if self.fg_is_positive:
                    color = (255, 0, 0)
                else:
                    # orange
                    color = (0, 165, 255)
                if self.input_mode == 'elevation_map':
                    circle_radius = 1
                elif self.input_mode == 'image':
                    circle_radius = 5
                cv2.circle(control_img_cv2, (int(click_xy[1]), int(click_xy[0])), circle_radius, color, -1)

        # Draw the buffered clicks on the image
        if self.control_points_xy is not None and len(self.control_points_xy) > 0:
            rospy.loginfo(f"Drawing {len(self.control_points_xy)} control points on the image from clicks")
            i = 0
            for i in range(len(self.control_points_xy)):
                click_xy = self.control_points_xy[i]
                label = self.control_points_label[i]
                if self.control_points_label[i] == 1:
                    color = (0, 255, 0)
                else:
                    color = (0, 0, 255)
                if self.input_mode == 'elevation_map':
                    circle_radius = 1
                elif self.input_mode == 'image':
                    circle_radius = 5
                cv2.circle(control_img_cv2, (int(click_xy[0]), int(click_xy[1])), circle_radius, color, -1)

        # Convert back to ROS image
        control_img_msg = self.bridge.cv2_to_imgmsg(control_img_cv2, "bgr8")
        # Publish the control points overlaid on the control image
        self.control_img_pub.publish(control_img_msg)

        # check befor we continue
        if (control_points_xy_height is None or len(control_points_xy_height) == 0) and (self.control_points_xy is None or len(self.control_points_xy) == 0):
            rospy.logwarn("moma_ui: No control points to segment with")
            resp = TriggerResponse()
            resp.success = False
            resp.message = "No control points to segment with!"     
            return resp       

        ## create the sum of the set of control points for segmentation

        control_points_xy_combined = control_points_xy_height + self.control_points_xy
        control_points_label_combined = control_points_label_height + self.control_points_label
        ###########################################################

        # Prepare the service call
        rospy.wait_for_service('/ros_sam/segment')  # Ensure service is available
        try:
            segmentation_srv = rospy.ServiceProxy('/ros_sam/segment', Segmentation)

            # Create segmentation request
            seg_req = SegmentationRequest()
            seg_req.image = self.control_image  # Stored image

            # fill up the query points and labels
            for i in range(len(control_points_xy_combined)):
                query_point = Point(x=control_points_xy_combined[i][0], y=control_points_xy_combined[i][1], z=0)
                seg_req.query_points.append(query_point)
                label = control_points_label_combined[i]
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
                    if self.fg_is_positive:
                        img_masked[~boolean_array] = 0
                    else:
                        img_masked[boolean_array] = 0
                    # img_masked[~boolean_array] = 0              

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
