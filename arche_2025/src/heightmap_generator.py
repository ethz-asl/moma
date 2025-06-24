#!/usr/bin/env python

import rospy
import ros_numpy
import numpy as np
import cv2

from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

class HeightmapGenerator:
    def __init__(self):
        self.pixel_size = rospy.get_param("~pixel_size", 0.01)  # meters per pixel
        self.max_height = rospy.get_param("~max_height", 0.01)   # meters
        self.grid_size = rospy.get_param("~grid_size", 64)   # in cells

        self.hsv_lower = np.array(rospy.get_param("~hsv_lower", [30, 50, 100]), dtype=np.uint8)   # light green default lower
        self.hsv_upper = np.array(rospy.get_param("~hsv_upper", [90, 100, 255]), dtype=np.uint8) # light green default upper

        self.bridge = CvBridge()
        self.latest_plane = None
        self.latest_cloud = None

        rospy.Subscriber("/segmented_plane_transform", TransformStamped, self.plane_callback, queue_size=1)
        rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.cloud_callback, queue_size=1)

        self.image_pub = rospy.Publisher("/plane_heightmap", Image, queue_size=1)

        rospy.loginfo("HeightmapGenerator initialized and waiting for data...")

    def plane_callback(self, msg):
        self.latest_plane = msg
        self.try_generate_heightmap()

    def cloud_callback(self, msg):
        self.latest_cloud = msg
        self.try_generate_heightmap()

    def try_generate_heightmap(self):
        if self.latest_plane is None or self.latest_cloud is None:
            return

        # Extract plane pose
        tf = self.latest_plane.transform
        origin = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        quat = np.array([tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])

        # Rotation: world → plane frame
        rot = R.from_quat(quat)
        R_plane = rot.as_matrix()
        plane_x = R_plane[:, 0]  # aligned with camera X (right)
        plane_y = R_plane[:, 1]  # aligned with camera Y (down)
        plane_z = R_plane[:, 2]  # plane normal (Z up from plane)

        # Convert PointCloud2 to XYZ points
        pc_arr = ros_numpy.point_cloud2.pointcloud2_to_array(self.latest_cloud)
        xyz = ros_numpy.point_cloud2.get_xyz_points(pc_arr, remove_nans=False)
        valid = np.isfinite(xyz).all(axis=1)

        points = xyz[valid]  # shape: (N_valid, 3)

        if points.shape[0] == 0:
            rospy.logwarn("No valid points in cloud.")
            return

        # Extract RGB and convert to HSV
        if 'rgb' in pc_arr.dtype.names:
            # Extract RGB from packed float32
            rgb_floats = pc_arr['rgb'][valid]
            rgb_uint32 = rgb_floats.view(np.uint32)

            r = ((rgb_uint32 >> 16) & 255).astype(np.uint8)
            g = ((rgb_uint32 >> 8) & 255).astype(np.uint8)
            b = (rgb_uint32 & 255).astype(np.uint8)

            rgb = np.stack([r, g, b], axis=1)  # shape: (N, 3)

            # Reshape to an image-like shape so OpenCV doesn't complain
            rgb_reshaped = rgb.reshape((-1, 1, 3))
            hsv_reshaped = cv2.cvtColor(rgb_reshaped, cv2.COLOR_RGB2HSV)
            hsv = hsv_reshaped.reshape((-1, 3)).astype(np.uint8)  # Final (N, 3)
        else:
            rospy.logwarn("No RGB data in point cloud.")
            hsv = None

        # Relative positions to plane centroid
        relative_points = points - origin

        # Project to plane-aligned frame
        x = relative_points @ plane_x
        y = relative_points @ plane_y
        z = relative_points @ plane_z  # height above plane

        # Set up grid
        half_width = self.pixel_size * self.grid_size / 2
        valid_mask = (
            (np.abs(x) < half_width) &
            (np.abs(y) < half_width)
        )

        # Also filter by HSV
        if hsv is not None:
            hsv_img = hsv.reshape(-1, 1, 3)
            mask = cv2.inRange(hsv_img, self.hsv_lower, self.hsv_upper).reshape(-1)
            hsv_masked = valid_mask & (mask != 0)
            valid_mask &= (mask == 0)

        if not np.any(valid_mask):
            rospy.logwarn("No points within heightmap bounds.")
            return

        ix = ((x[valid_mask] + half_width) / self.pixel_size).astype(int)
        iy = ((y[valid_mask] + half_width) / self.pixel_size).astype(int)
        ix = np.clip(ix, 0, self.grid_size - 1)
        iy = np.clip(iy, 0, self.grid_size - 1)
        iy = self.grid_size - 1 - iy  # Flip vertically

        ix_hsv = ((x[hsv_masked] + half_width) / self.pixel_size).astype(int)
        iy_hsv = ((y[hsv_masked] + half_width) / self.pixel_size).astype(int)
        ix_hsv = np.clip(ix_hsv, 0, self.grid_size - 1)
        iy_hsv = np.clip(iy_hsv, 0, self.grid_size - 1)

        # Normalize heights to 0–255
        height_norm = np.clip(z / self.max_height, 0, 1)
        height_uint8 = (height_norm * 255).astype(np.uint8)
        height_uint8 = height_uint8[valid_mask]

        # Create image
        img = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)
        for x_pix, y_pix, h_val in zip(ix, iy, height_uint8):
            if h_val > img[y_pix, x_pix]:
                img[y_pix, x_pix] = h_val

        # Publish as ROS image
        ros_img = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
        ros_img.header = self.latest_cloud.header
        self.image_pub.publish(ros_img)

        rospy.loginfo("Published heightmap image.")

if __name__ == "__main__":
    rospy.init_node("heightmap_generator")
    node = HeightmapGenerator()
    rospy.spin()