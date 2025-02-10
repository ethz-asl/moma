#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Empty, Trigger, TriggerResponse
from cv_bridge import CvBridge
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32
from std_srvs.srv import SetBool, SetBoolResponse
from visualization_msgs.msg import Marker, MarkerArray
from dynamic_reconfigure.server import Server

import std_msgs.msg as std_msgs

from interactive_markers.interactive_marker_server import *

import subprocess
import datetime

from moma_ui.cfg import moma_ui_paramConfig
# ColorRGBA
from std_msgs.msg import ColorRGBA

import matplotlib.pyplot as plt

from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

from nav_msgs.msg import Path

import os

import cv2
import numpy as np
import copy
import struct
from sensor_msgs.msg import PointCloud2
import pyransac3d as pyrsc
import scipy.spatial.transform as sst
from geometry_msgs.msg import Pose, Quaternion
from sensor_msgs import point_cloud2 as pc2

import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
# import color

from segment_anything import SamAutomaticMaskGenerator, sam_model_registry

class MomaUiNode:
    def __init__(self):
        # Initialize node
        rospy.init_node('moma_ui_node')

        self.input_mode = rospy.get_param('~input_mode', 'elevation_map') # 'image' or 'elevation_map'
        assert self.input_mode in ['image', 'elevation_map'], "Invalid input mode. Choose 'image' or 'elevation_map'"

        # Image subscriber and storage
        self.last_received_img = None
        self.image_sub = rospy.Subscriber("/rs_435_1/color/image_raw", Image, self.image_callback)
        self.last_elevation_map = None
        self.elevation_map_sub = rospy.Subscriber("/elevation_mapping/elevation_map", GridMap, self.elevation_map_callback)

        self.sweep_sub = rospy.Subscriber("moma_ui/sweep/plan_in", Path, self.sweep_callback) 

        # Mouse click subscriber and storage
        self.control_image = None
        self.click_sub = rospy.Subscriber("/moma_ui/sam/control_image/mouse_click", PointStamped, self.click_callback)
        self.control_points_xy = []
        self.control_points_label = []
        self.last_mask = None
        self.last_masks_from_sam = None

        # stuff
        self.world_frame = rospy.get_param('world_frame_id', 'world')
        self.work_plane_frame = rospy.get_param('work_plane_id', 'workplane')
        self.last_marker_msg = None
        self.fg_is_positive = rospy.get_param('~fg_is_positive', False)
        
        # label subscriber and storage
        self.fg_min_height_sub = rospy.Subscriber("moma_ui/sam/foreground_min_height", Float32, self.fg_min_height_callback)
        
        self.current_label = 'positive'
        self.fg_min_height = 10.0

        self.sweep_marker_enabled = True

        self.last_received_sweep_path = None

        ## Publishers
        self.control_img_pub = rospy.Publisher('moma_ui/sam/control_image', Image, queue_size=10)
        self.mask_pub = rospy.Publisher('moma_ui/sam/mask_image', Image, queue_size=10)
        self.masked_pub = rospy.Publisher('moma_ui/sam/masked_image', Image, queue_size=10)
        self.filtered_elevation_map_pub = rospy.Publisher('moma_ui/sam/filtered_elevation_map', GridMap, queue_size=10)
        self.elev_map_rgb_img_pub = rospy.Publisher('moma_ui/sam/elevation_map_rgb_image', Image, queue_size=10)
        self.elev_map_height_img_pub = rospy.Publisher('moma_ui/sam/elevation_map_height_image', Image, queue_size=10)
        self.viz_marker_array_pub = rospy.Publisher('moma_ui/viz_marker_array', MarkerArray, queue_size=10)

        ## Services
        # self.reset_sam_cfg_srv = rospy.Service('moma_ui/sam/reset', Empty, self.reset_sam_config)
        self.reset_sam_cfg_topic_srv = rospy.Subscriber('moma_ui/sam/reset', std_msgs.Empty, self.reset_sam_config)
        # self.run_sam_srv = rospy.Service('moma_ui/sam/run', Trigger, self.run_sam)
        self.run_sam_topic_srv = rospy.Subscriber('moma_ui/sam/run', std_msgs.Empty, self.run_sam)
        self.set_label_fg_bg_srv = rospy.Service('moma_ui/sam/set_label_fg_bg', SetBool, self.set_label_fg_bg)
        self.start_stop_rosbag_rec_srv = rospy.Service('moma_ui/rosbag_recorder/start_stop', SetBool, self.start_stop_rosbag_rec)
        self.clear_map_srv = rospy.Service('moma_ui/map/clear', Trigger, self.clear_map)
        self.use_sweep_from_topic_srv = rospy.Service('moma_ui/sweep/use_sweep_topic', SetBool, self.use_sweep_from_topic)

        # CVBridge for image conversion
        self.bridge = CvBridge()

        ## for WP detection
        self.wp_detection_srv = rospy.Service('moma_ui/work_plane/detect', Trigger, self.wp_detection)
        self.point_cloud_sub = rospy.Subscriber('/rs_435_3/depth/color/points_passthrough_xyz', PointCloud2, self.point_cloud_cb)
        self.last_received_pointcloud = None
        # the prior for the work plane either as a pose or as a support and normal
        # T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP = rospy.get_param('/T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP', '0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0')      
        T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP = rospy.get_param('/T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP', '0.4, 0.0, 0.3, 0, 0, -0.7071068, 0.7071068')      
        self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose = Pose()
        self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.position.x = float(T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP.split(',')[0])
        self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.position.y = float(T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP.split(',')[1])
        self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.position.z = float(T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP.split(',')[2])
        self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.orientation.x = float(T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP.split(',')[3])
        self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.orientation.y = float(T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP.split(',')[4])
        self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.orientation.z = float(T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP.split(',')[5])
        self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.orientation.w = float(T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP.split(',')[6])

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # dynrec server
        self.dynrec_cfg = None
        self.dynrec_srv = Server(moma_ui_paramConfig, self.dynrecCb)
        # ros timer
        self.timer = rospy.Timer(rospy.Duration(0.1), lambda msg: self.timer_cb(msg))

        # sweep interactive marker
        self.sweep_marker_enabled = True
        # self.interact_marker_server = InteractiveMarkerServer("moma_ui/interactive_marker_server")
        # self.init_interactive_markers()

        # load SAM
        path_to_sam_model = rospy.get_param('~path_to_sam_model', '/root/moma_ws/src/moma/moma_ui/sam_models/sam_vit_h_4b8939.pth')
        sam = sam_model_registry["vit_h"](checkpoint=path_to_sam_model)
        # sam = sam_model_registry["vit_l"](checkpoint="/root/moma_ws/src/ros_sam/ros_sam/models/sam_vit_l_0b3195.pth")
        device = "cuda"
        sam = sam.to(device)

        self.mask_generator = SamAutomaticMaskGenerator(
            model=sam,
            points_per_side=32,
            pred_iou_thresh=0.86,
            stability_score_thresh=0.92,
            crop_n_layers=1,
            crop_n_points_downscale_factor=2,
            min_mask_region_area=100,  # Requires open-cv to run post-processing
        )

        # rosbag recorder
        self.rosbag_record_subprocess = None

    def sweep_callback(self, msg):
        rospy.loginfo(f"moma_ui: Received sweep path")
        # check how many waypoints are in the path
        last_received_sweep_path = copy.deepcopy(msg)
        num_waypoints = len(last_received_sweep_path.poses)
        if num_waypoints == 0:
            rospy.logwarn("moma_ui: Received sweep path has no waypoints")
            return
        if num_waypoints == 1:
            rospy.logwarn("moma_ui: Received sweep path has only one waypoint")
            return
        if num_waypoints > 1:
            rospy.loginfo(f"moma_ui: Received sweep path has {num_waypoints} waypoints will just use first and last")
            last_received_sweep_path.poses = [last_received_sweep_path.poses[0], last_received_sweep_path.poses[-1]]
        # the sweep has to be in the work plane frame!
        if self.last_received_sweep_path.header.frame_id != self.work_plane_frame:
            rospy.logwarn("moma_ui: Sweep path is not in the right frame")
            return
        self.last_received_sweep_path = last_received_sweep_path
        # visualize the sweep path
        marker_array_msg = MarkerArray()
        # make a box that starts at the first waypoint and ends at the last waypoint, it should start on the work plane and be perpendicular to the work plane
        thickness = 0.001
        sweep_marker = Marker()
        sweep_marker.header.frame_id = self.work_plane_frame
        sweep_marker.header.stamp = rospy.Time(0)
        sweep_marker.ns = 'sweep_marker'
        sweep_marker.id = 0
        sweep_marker.type = 1
        sweep_marker.action = 0
        sweep_marker.pose.position = self.last_received_sweep_path.poses[0].position
        sweep_marker.scale = Vector3(0.5, 0.5, thickness)
        sweep_marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.5)
        marker_array_msg.markers.append(sweep_marker)
        self.viz_marker_array_pub.publish(marker_array_msg)

    def use_sweep_from_topic(self, req):
        self.sweep_marker_enabled = req.data
        rospy.loginfo(f"moma_ui: Sweep marker enabled: {self.sweep_marker_enabled}")
        # return SetBoolResponse(True, "Sweep marker enabled")
        if self.sweep_marker_enabled:
            return SetBoolResponse(True, "Sweep marker enabled")
        else:
            return SetBoolResponse(True, "Sweep marker disabled")

    def start_stop_rosbag_rec(self, req):   
        if req.data and self.rosbag_record_subprocess is None:
            # get the params moma_ui/rosbag_recorder/bag_output_dir and moma_ui/rosbag_recorder/topics_to_record
            bag_output_dir = rospy.get_param('moma_ui/rosbag_recorder/bag_output_dir', '/tmp')
            topics_to_record = rospy.get_param('moma_ui/rosbag_recorder/topics_to_record', [])
            # get all the individual topics from the list
            topics_to_record_str = ' '.join(topics_to_record)
            print(f"topics_to_record_str: {topics_to_record_str}")
            # topics_to_record_str = ' '.join(topics_to_record)
            # check if dir exists
            if not os.path.exists(bag_output_dir):
                rospy.logwarn(f"moma_ui: Bag output dir {bag_output_dir} does not exist, creating it...")
                os.makedirs(bag_output_dir)
            rospy.loginfo("moma_ui: Starting rosbag recording...")
            # self.rosbag_record_subprocess = subprocess.Popen(["rosbag", "record", "-a"])
            # get date and time as YYYY-MM-DD-HH-MM-SS
            date_time = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            cmd = ["rosbag", "record", "-O", f"{bag_output_dir}/moma_ui_{date_time}.bag"] + topics_to_record
            self.rosbag_record_subprocess = subprocess.Popen(cmd)
            # self.rosbag_record_subprocess = subprocess.Popen(["rosbag", "record", "-O", f"{bag_output_dir}/moma_ui_{rospy.Time.now().to_sec()}.bag", topics_to_record_str])
            rospy.loginfo("moma_ui: Started rosbag recording")
            resp = SetBoolResponse()
            resp.success = True
            resp.message = "Started rosbag recording"
            return resp
        elif req.data and self.rosbag_record_subprocess is not None:
            rospy.logwarn("moma_ui: Rosbag recording already in progress, stop it first before starting again")
            resp = SetBoolResponse()
            resp.success = False
            resp.message = "Rosbag recording already in progress, stop it first before starting again"
            return resp
        elif not req.data and self.rosbag_record_subprocess is None:
            rospy.logwarn("moma_ui: Rosbag recording not started yet, nothing to stop")
            resp = SetBoolResponse()
            resp.success = False
            resp.message = "Rosbag recording not started yet, nothing to stop"
            return resp
        elif not req.data and self.rosbag_record_subprocess is not None:
            rospy.loginfo("moma_ui: Stopping rosbag recording...")
            self.rosbag_record_subprocess.terminate()
            self.rosbag_record_subprocess.wait()
            rospy.loginfo("moma_ui: Stopped rosbag recording")
            self.rosbag_record_subprocess = None
            resp = SetBoolResponse()
            resp.success = True
            resp.message = "Stopped rosbag recording"
            return resp

    # callbacks
    def image_callback(self, msg):
        """Callback to update the most recent image."""
        if self.input_mode == 'image':
            self.last_received_img = msg
    
    def timer_cb(self, msg):
        # viz marker
        marker_array_msg = MarkerArray()
        thickness = 0.001
        plane_marker = Marker()
        plane_marker.header.frame_id = self.world_frame
        plane_marker.header.stamp = rospy.Time(0)
        plane_marker.ns = 'work_plane_marker'
        plane_marker.id = 0
        plane_marker.type = 1
        plane_marker.action = 0
        plane_marker.pose = self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose
        plane_marker.scale = Vector3(0.5, 0.5, thickness)
        plane_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.5)
        marker_array_msg.markers.append(plane_marker)
        self.viz_marker_array_pub.publish(marker_array_msg)
        # TF
        pos = (self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.position.x,
               self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.position.y, 
               self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.position.z)
        rot = (self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.orientation.x, 
               self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.orientation.y,
               self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.orientation.z, 
               self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose.orientation.w)
        
        # Create a TransformStamped message
        transform = TransformStamped()

        # Set the time, frame IDs, and the position/rotation
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.work_plane_frame

        # Set the position (translation)
        transform.transform.translation.x = pos[0]
        transform.transform.translation.y = pos[1]
        transform.transform.translation.z = pos[2]

        # Set the orientation (rotation)
        transform.transform.rotation.x = rot[0]
        transform.transform.rotation.y = rot[1]
        transform.transform.rotation.z = rot[2]
        transform.transform.rotation.w = rot[3]

        # Broadcast the transform
        # rospy.loginfo(f"moma_ui: Broadcasting transform from {transform.header.frame_id} to {transform.child_frame_id}")
        self.tf_broadcaster.sendTransform(transform)

    def elevation_map_callback(self, msg):
        # check if frame is in work plane frame
        if msg.info.header.frame_id != self.work_plane_frame:
            rospy.logwarn("moma_ui: Elevation map is not in the right frame")
            return
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

        # rotate it by +90 degrees
        # color_img = np.rot90(color_img, k=-1)
        # # flip lr
        color_img = np.fliplr(color_img)    
        ros_image = self.bridge.cv2_to_imgmsg(color_img, encoding="bgr8")
        self.elev_map_rgb_img_pub.publish(ros_image)
        # if elev_map mode, store the it as the last received image
        if self.input_mode == 'elevation_map':
            self.last_received_img = ros_image
            if self.last_mask is None and self.fg_is_positive:
                self.last_mask = np.ones((num_rows, num_cols), dtype=bool)
            elif self.last_mask is None and not self.fg_is_positive:
                self.last_mask = np.zeros((num_rows, num_cols), dtype=bool)
            msg_copy = copy.deepcopy(msg)
            elevation_layer = np.array(msg_copy.data[msg_copy.layers.index('elevation')].data).reshape((num_rows, num_cols))
            
            corrected_mask = copy.deepcopy(self.last_mask)
            # flip lr
            corrected_mask = np.fliplr(corrected_mask)
            # rotate it by +90 degrees
            # corrected_mask = np.rot90(corrected_mask, k=1)

            if self.fg_is_positive:
                elevation_layer[~corrected_mask] = 0.0
            else:
                elevation_layer[corrected_mask] = 0.0
            msg_copy.data[msg_copy.layers.index('elevation')].data = elevation_layer.flatten().tolist()
            self.filtered_elevation_map_pub.publish(msg_copy)
            
            # extract elevation layer and create a height image
            elevation_layer = copy.deepcopy(np.array(msg_copy.data[msg_copy.layers.index('elevation')].data).reshape((num_rows, num_cols)))
            # mask out all nan and inf values
            elevation_layer[np.isnan(elevation_layer)] = 0
            elevation_layer[np.isinf(elevation_layer)] = 0
            # make all positive
            if np.min(elevation_layer) < 0:
                elevation_layer -= np.min(elevation_layer)
            # create a height image
            height_img = np.zeros((num_rows, num_cols), dtype=np.uint8)
            max_height = np.max(elevation_layer)
            height_img = (elevation_layer / max_height) * 255.0
            height_img = height_img.astype(np.uint8)
            ros_height_img = self.bridge.cv2_to_imgmsg(height_img, encoding="mono8")
            self.elev_map_height_img_pub.publish(ros_height_img)
 
        # marker
        if self.last_marker_msg is not None:
            self.viz_marker_array_pub.publish(self.last_marker_msg)

    # DYNREC SERVER
    def dynrecCb(self, config, level):
        rospy.loginfo('moma_ui: Got dynamic reconfigure request.')
        self.dynrec_cfg = config
        return self.dynrec_cfg

    def point_cloud_cb(self, msg):
        self.last_received_pointcloud = msg

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
        marker.header.frame_id = self.work_plane_frame
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
    def clear_map(self, req):
        rospy.loginfo("moma_ui: Clearing the map...")

        rospy.wait_for_service('/voxblox_node/clear_map')
        try:
            # Create a service proxy (client)
            clear_vb_map = rospy.ServiceProxy('/voxblox_node/clear_map', Empty)
            # Call the service
            response = clear_vb_map()
            rospy.loginfo("moma_ui: Succesfully cleared voxblox map!")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


        rospy.wait_for_service('/elevation_mapping/clear_map')
        try:
            # Create a service proxy (client)
            clear_elev_map = rospy.ServiceProxy('/elevation_mapping/clear_map', Empty)
            # Call the service
            response = clear_elev_map()
            rospy.loginfo("moma_ui: Succesfully cleared elevation map!")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
        
        resp = TriggerResponse()
        resp.success = True
        resp.message = "Clearing map was succesful!"
        return resp

    def planeSupportAndNormalToSupportPose(self, support_xyz, normal_xyz):
        plane_support_pose = Pose()
        plane_support_pose.position.x = support_xyz[0]
        plane_support_pose.position.y = support_xyz[1]
        plane_support_pose.position.z = support_xyz[2]
        # compute quaternion from normal and x_prime
        n_x_prime = np.array([0.0, -1.0, 0.0])
        n_z = np.array(normal_xyz)
        if n_z[2] < 0.0:
            n_z = -n_z
        n_z = n_z/np.linalg.norm(n_z)  # required?
        n_y = np.cross(n_z, n_x_prime)
        n_x = np.cross(n_y, n_z)
        q_plane = sst.Rotation.from_matrix(
            np.vstack((n_x, n_y, n_z)).T).as_quat()
        # set new plane pose
        plane_support_pose.orientation = Quaternion(
            q_plane[0], q_plane[1], q_plane[2], q_plane[3])
        return plane_support_pose

    def planeSupportPoseFromPlaneParamsAndPriorPose(self, plane_params_abcd, prior_plane_support_pose):
        # compute new plane in support+normal representation
        support_x = prior_plane_support_pose.position.x
        support_y = prior_plane_support_pose.position.y
        support_z = (0.0 - plane_params_abcd[0]*support_x - plane_params_abcd[1]
                     * support_y - plane_params_abcd[3])/plane_params_abcd[2]
        support = np.array([support_x, support_y, support_z])
        normal = np.array([plane_params_abcd[0], plane_params_abcd[1], plane_params_abcd[2]])
        # convert to support pose
        new_plane_support_pose = self.planeSupportAndNormalToSupportPose(
            support, normal)
        return new_plane_support_pose

    def wp_detection(self, req):
        rospy.loginfo("moma_ui: Detecting work plane...")
        if self.last_received_pointcloud is None:
            rospy.logwarn("moma_ui: No point cloud received")
            resp = TriggerResponse()
            resp.success = False
            resp.message = "No point cloud received!"
            return resp

        # check if the point cloud is in the right frame
        if self.last_received_pointcloud.header.frame_id != self.world_frame:
            rospy.logerr("moma_ui: Point cloud is not in the right frame")
            resp = TriggerResponse()
            resp.success = False
            resp.message = "Point cloud is not in the right frame!"
            return resp

        # run RANSAC to detect the work plane
        # unpack array
        gen = pc2.read_points(self.last_received_pointcloud, skip_nans=True)
        int_data = list(gen)
        xyz = []
        for x in int_data:
            xyz.append([x[0], x[1], x[2]])
        xyz = np.array(xyz)
        xyz_subs = xyz[::100, :]
        # fit plane
        plane1 = pyrsc.Plane()
        plane_params, best_inliers = plane1.fit(
            pts=xyz_subs, thresh=self.dynrec_cfg.plane_fit_ransac_inlier_distance, maxIteration=self.dynrec_cfg.plane_fit_ransac_max_iteration)
        new_plane_support_pose = self.planeSupportPoseFromPlaneParamsAndPriorPose(
            plane_params, self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose)
        
        # extract the pose from the plane_params
        self.T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP_pose = new_plane_support_pose

        rospy.loginfo("moma_ui: Successfully detected work plane!")
        resp = TriggerResponse()
        resp.success = True
        resp.message = "Successfully detected work plane!"
        return resp

    def show_anns(self, anns):
        if len(anns) == 0:
            return
        sorted_anns = sorted(anns, key=(lambda x: x['area']), reverse=True)
        ax = plt.gca()
        ax.set_autoscale_on(False)

        img = np.ones((sorted_anns[0]['segmentation'].shape[0], sorted_anns[0]['segmentation'].shape[1], 4))
        img[:,:,3] = 0
        for ann in sorted_anns:
            m = ann['segmentation']
            color_mask = np.concatenate([np.random.random(3), [0.35]])
            img[m] = color_mask
        return img

    def reset_sam_config(self, msg):
        rospy.loginfo("moma_ui: Resetting SAM control image, points...")
        """Reset the buffer of click points."""
        self.control_points_xy = []
        self.control_points_label = []
        self.last_mask = None
        self.last_masks_from_sam = None
        if self.last_received_img is not None:
            self.control_image = self.last_received_img
            # # # fully segment with SAM
            # # # sam = sam_model_registry["vit_h"](checkpoint="/root/moma_ws/src/ros_sam/ros_sam/models/sam_vit_h_4b8939.pth")
            # # # sam = sam_model_registry["vit_l"](checkpoint="/root/moma_ws/src/ros_sam/ros_sam/models/sam_vit_l_0b3195.pth")

            # # device = "cuda"
            # # sam = sam.to(device)

            # self.mask_generator = SamAutomaticMaskGenerator(
            #     model=self.sam,
            #     points_per_side=32,
            #     pred_iou_thresh=0.86,
            #     stability_score_thresh=0.92,
            #     crop_n_layers=1,
            #     crop_n_points_downscale_factor=2,
            #     min_mask_region_area=100,  # Requires open-cv to run post-processing
            # )
            # get the image
            image = self.bridge.imgmsg_to_cv2(self.control_image, desired_encoding="rgb8")
            rospy.loginfo("moma_ui: SAM will now find all masks")
            masks = self.mask_generator.generate(image)
            rospy.loginfo("moma_ui: SAM found all masks")
            mask_img = self.show_anns(masks)
            # Ensure the mask has the same shape as the image
            mask_rgb = mask_img[:, :, :3]*255.0  # Take only the RGB channels

            # Blend using alpha from the mask
            blended = cv2.addWeighted(image, 1, mask_rgb, 0.5,
                                        0, dtype=cv2.CV_8U)
            
            # Convert back to ROS image
            control_img_msg = self.bridge.cv2_to_imgmsg(blended, "bgr8")

            self.control_img_pub.publish(control_img_msg)
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg((mask_img[:, :, :3] * 255.0).astype(np.uint8), "bgr8"))
            self.control_image = control_img_msg
            self.last_masks_from_sam = masks
            rospy.loginfo("moma_ui: Reset SAM control image and oversegmented it")
        
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

    def run_sam(self, msg):
        rospy.loginfo("moma_ui: Segmenting image...")
        if self.control_image is None:
            rospy.logwarn("moma_ui: No control image to segment")
            return
        if self.last_masks_from_sam is None:
            rospy.logwarn("moma_ui: No masks from SAM")
            return

        # go through the masks and select the ones that contain the control points
        control_points_xy = np.array(self.control_points_xy)
        control_points_label = np.array(self.control_points_label)
        # get the image
        image = self.bridge.imgmsg_to_cv2(self.control_image, desired_encoding="rgb8")

        # iterate over the masks and select the ones that contain the control points
        i = 0
        positive_masks = []
        for mask in self.last_masks_from_sam:
            mask_array = np.array(mask['segmentation'])
            # go through the control points and check if they are in the mask
            nr_of_positive_points = 0
            nr_of_negative_points = 0
            if len(control_points_xy) == 1:
                point = control_points_xy[0]
                # convert to int
                point = (int(point[0]), int(point[1]))
                label = control_points_label[0]
                if mask_array[point[1], point[0]]:
                    if label == 1:
                        nr_of_positive_points += 1
                    else:
                        nr_of_negative_points += 1
            elif len(control_points_xy) > 1:
                for j in range(len(control_points_xy)):
                    point = control_points_xy[j]
                    # convert to int
                    point = (int(point[0]), int(point[1]))
                    label = control_points_label[j]
                    if mask_array[point[1], point[0]]:
                        if label == 1:
                            nr_of_positive_points += 1
                        else:
                            nr_of_negative_points += 1
            # if the mask contains strictly more positive points than negative points, add it to the positive masks
            if nr_of_positive_points > nr_of_negative_points:
                positive_masks.append(mask)
            i += 1
               
        # create final mask from OR across all positive masks
        final_mask = np.zeros((image.shape[0], image.shape[1]), dtype=bool)

        for mask in positive_masks:
            final_mask = np.logical_or(final_mask, mask['segmentation'])
            rospy.loginfo("moma_ui: Publishing positive mask")

        self.last_mask = final_mask

        img_masked = self.bridge.imgmsg_to_cv2(self.control_image).copy()

        # mask the image where the mask is false
        img_masked[~final_mask] = 0

        # convert to ros image
        img_masked = self.bridge.cv2_to_imgmsg(img_masked, "bgr8")

        # Now publish
        self.masked_pub.publish(img_masked)

        # convert the image to 
        # rospy.loginfo(f"Received {len(response.masks)} masks from segmentation service")
        # for mask in response.masks:
        #     actual_mask = self.bridge.imgmsg_to_cv2(mask, desired_encoding='mono8')
        #     boolean_array = actual_mask.astype(bool)
        #     self.last_mask = boolean_array
        #     # mask the image where the mask is false
        #     if self.fg_is_positive:
        #         img_masked[~boolean_array] = 0
        #     else:
        #         img_masked[boolean_array] = 0
        #     # img_masked[~boolean_array] = 0              

        # img_masked = self.bridge.cv2_to_imgmsg(img_masked, "bgr8")
        # mask_image = response.masks[0]
        # self.mask_pub.publish(mask_image)
        # self.masked_pub.publish(img_masked)

        '''
        # publish the final mask
        mask_img = np.zeros((image.shape[0], image.shape[1], 4))
        mask_img[:,:,3] = 0
        mask_img[final_mask] = [0, 255, 0, 0.35]
        # convert to ros image
        mask_rgb = mask_img[:, :, :3]*255.0  # Take only the RGB channels
        # Convert float64 image to uint8
        mask_rgb = np.clip(mask_rgb * 255, 0, 255).astype(np.uint8)  # Scale and cast
        # Now publish
        self.masked_pub.publish(self.bridge.cv2_to_imgmsg(mask_rgb, "bgr8"))
        '''

        rospy.loginfo("moma_ui: Successfully segmented image!")

    '''
    def run_sam(self, req):
        rospy.loginfo("moma_ui: Segmenting image...")
        """Call segmentation service with stored image and buffered clicks."""
        if self.control_image is None:
            rospy.logwarn("moma_ui: No control image to segment")
            resp = TriggerResponse()
            resp.success = False
            resp.message = "No control image to segment!"
            return resp

        # in last received height map, filter out the points that are above the fg_min_height
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
    '''

if __name__ == '__main__':
    try:
        node = MomaUiNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
