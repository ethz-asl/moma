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
from dynamic_reconfigure.server import Server

from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import InteractiveMarkerControl, InteractiveMarker
import tf


from moma_ui.cfg import moma_ui_paramConfig
# ColorRGBA
from std_msgs.msg import ColorRGBA

import matplotlib.pyplot as plt

from geometry_msgs.msg import TransformStamped, Vector3, Quaternion

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


class MomaUiNode:
    def __init__(self):
        # Initialize node
        rospy.init_node('mom_ui_node')

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
        self.world_frame = rospy.get_param('world_frame_id', 'world')
        self.work_plane_frame = rospy.get_param('work_plane_id', 'work_plane')
        self.last_marker_msg = None
        self.fg_is_positive = rospy.get_param('~fg_is_positive', False)
        
        # label subscriber and storage
        self.fg_min_height_sub = rospy.Subscriber("moma_ui/sam/foreground_min_height", Float32, self.fg_min_height_callback)
        
        self.current_label = 'positive'
        self.fg_min_height = 10.0

        ## Publishers
        self.control_img_pub = rospy.Publisher('moma_ui/sam/control_image', Image, queue_size=10)
        self.mask_pub = rospy.Publisher('moma_ui/sam/mask_image', Image, queue_size=10)
        self.masked_pub = rospy.Publisher('moma_ui/sam/masked_image', Image, queue_size=10)
        self.filtered_elevation_map_pub = rospy.Publisher('moma_ui/sam/filtered_elevation_map', GridMap, queue_size=10)
        self.elev_map_img_pub = rospy.Publisher('moma_ui/sam/elevation_map_image', Image, queue_size=10)
        self.viz_marker_array_pub = rospy.Publisher('moma_ui/viz_marker_array', MarkerArray, queue_size=10)

        ## Services
        self.reset_sam_cfg_srv = rospy.Service('moma_ui/sam/reset', Empty, self.reset_sam_config)
        self.run_sam_srv = rospy.Service('moma_ui/sam/run', Trigger, self.run_sam)
        self.set_label_fg_bg_srv = rospy.Service('moma_ui/sam/set_label_fg_bg', SetBool, self.set_label_fg_bg)
        # self.start_stop_rosbag_rec_srv = rospy.Service('moma_ui/rosbag_recorder/start_stop', SetBool, self.start_stop_rosbag_rec)
        self.clear_map_srv = rospy.Service('moma_ui/map/clear', Trigger, self.clear_map)

        # CVBridge for image conversion
        self.bridge = CvBridge()

        ## for WP detection
        self.wp_detection_srv = rospy.Service('moma_ui/work_plane/detect', Trigger, self.wp_detection)
        self.point_cloud_sub = rospy.Subscriber('/world_cloud', PointCloud2, self.point_cloud_cb)
        self.last_received_pointcloud = None
        # the prior for the work plane either as a pose or as a support and normal
        T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP = rospy.get_param('/T_W_WP_as_tx_ty_tz_qx_qy_qz_qw_TF_W_WP', '0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0')      
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

        # rosbag recorder
        

    '''
    def processFeedback(self, feedback):
        p = feedback.pose.position
        print(feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))

    def init_interactive_markers(self):
        rospy.loginfo("Interactive markers initialized")
        # create an interactive marker for our server
        sweep_start_int_marker = InteractiveMarker()
        sweep_start_int_marker.header.frame_id = self.world_frame
        sweep_start_int_marker.name = "sweep_start_marker"
        sweep_start_int_marker.description = "Sweep Start"
        sweep_start_int_marker.pose.position.x = 0.5
        sweep_start_int_marker.scale = 0.2

        # Add controls for translation in x and y
        control_x = InteractiveMarkerControl()
        control_x.name = "move_x"
        control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_x.orientation.w = 1.0  # No rotation for the x-axis movement
        sweep_start_int_marker.controls.append(control_x)

        # create a grey box marker
        sweep_start_box_marker = Marker()
        sweep_start_box_marker.type = Marker.CYLINDER
        sweep_start_box_marker.scale.x = 0.07   
        sweep_start_box_marker.scale.y = 0.07
        sweep_start_box_marker.scale.z = 0.1
        # sweep_start_box_marker.pose.position.z = 0.025
        sweep_start_box_marker.color.r = 0.0
        sweep_start_box_marker.color.g = 0.5
        sweep_start_box_marker.color.b = 0.5
        sweep_start_box_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        sweep_start_box_control = InteractiveMarkerControl()
        sweep_start_box_control.always_visible = True

        sweep_start_box_control.markers.append( sweep_start_box_marker )
        # add the control to the interactive marker
        sweep_start_int_marker.controls.append( sweep_start_box_control )

        # create a control which will move the box along the x-axis
        sweep_start_trans_x_ctrl = InteractiveMarkerControl()
        sweep_start_trans_x_ctrl.name = "move_x"
        sweep_start_trans_x_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        # sweep_start_trans_x_ctrl.scale = 0.1
        sweep_start_int_marker.controls.append(sweep_start_trans_x_ctrl)

        # create a control which will move the box along the y-axis
        sweep_start_trans_y_ctrl = InteractiveMarkerControl()
        sweep_start_trans_y_ctrl.name = "move_y"
        sweep_start_trans_y_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        rot_as_arr = tf.transformations.quaternion_from_euler(0, 0, 1.5708)
        sweep_start_trans_y_ctrl.orientation.x = rot_as_arr[0]
        sweep_start_trans_y_ctrl.orientation.y = rot_as_arr[1]
        sweep_start_trans_y_ctrl.orientation.z = rot_as_arr[2]
        sweep_start_trans_y_ctrl.orientation.w = rot_as_arr[3]
        sweep_start_int_marker.controls.append(sweep_start_trans_y_ctrl)

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        self.interact_marker_server.insert(sweep_start_int_marker, self.processFeedback)
        
        # add sweep end marker
        sweep_end_int_marker = InteractiveMarker()
        sweep_end_int_marker.header.frame_id = self.work_plane_frame
        sweep_end_int_marker.name = "sweep_end_marker"
        sweep_end_int_marker.description = "Sweep End"
        sweep_end_int_marker.pose.position.x = 0.5
        sweep_end_int_marker.pose.position.y = 0.0
        sweep_end_int_marker.scale = 0.2
        
        # 'commit' changes and send to all clients
        self.interact_marker_server.applyChanges()
    '''

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
        self.tf_broadcaster.sendTransform(transform)

    def elevation_map_callback(self, msg):
        # check if frame is in work plane frame
        if msg.info.frame_id != self.work_plane_frame:
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

    # DYNREC SERVER
    def dynrecCb(self, config, level):
        rospy.logwarn('moma_ui: Got dynrec request!')
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

        rospy.logwarn("moma_ui: Clearing map not implemented yet!")

        resp = TriggerResponse()
        resp.success = False
        resp.message = "Clearing map not implemented yet!"
        return resp

    def planeSupportAndNormalToSupportPose(self, support_xyz, normal_xyz):
        plane_support_pose = Pose()
        plane_support_pose.position.x = support_xyz[0]
        plane_support_pose.position.y = support_xyz[1]
        plane_support_pose.position.z = support_xyz[2]
        # compute quaternion from normal and x_prime
        n_x_prime = np.array([1.0, 0.0, 0.0])
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
        # if self.last_received_pointcloud.header.frame_id != self.world_frame:
        #     rospy.logwarn("moma_ui: Point cloud is not in the right frame")
        #     resp = TriggerResponse()
        #     resp.success = False
        #     resp.message = "Point cloud is not in the right frame!"
        #     return resp

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
