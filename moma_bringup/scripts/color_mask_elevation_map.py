#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from grid_map_msgs.msg import GridMap
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

# Global variable to store the grid map message
grid_map = None

# Define the brown color range
brown_lower = np.array([100, 50, 0], dtype=np.uint8)  # Lower bound for brown (R, G, B)
brown_upper = np.array([200, 150, 100], dtype=np.uint8)  # Upper bound for brown (R, G, B)

# Initialize CvBridge for converting OpenCV images to ROS Image messages
bridge = CvBridge()

def pointcloud_callback(pointcloud_msg):
    global grid_map

    # Ensure grid map is already received
    if grid_map is None:
        rospy.loginfo("Waiting for grid map message...")
        return

    # Extract point cloud data
    points = pc2.read_points(pointcloud_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
    
    # Get grid map size and resolution from grid map metadata
    map_length_x = grid_map.info.length_x
    map_length_y = grid_map.info.length_y
    resolution = grid_map.info.resolution

    # Calculate number of rows and columns based on grid map dimensions
    num_rows = int(map_length_y / resolution)
    num_cols = int(map_length_x / resolution)

    # Initialize arrays for storing color information
    color_sum = np.zeros((num_rows, num_cols, 3), dtype=np.float32)  # To accumulate RGB values
    point_count = np.zeros((num_rows, num_cols), dtype=np.int32)      # To count points per cell

    # Iterate over the point cloud and process the points
    for point in points:
        x, y, z, rgb_float = point
        r, g, b = extract_rgb(rgb_float)

        # Calculate row and column indices based on the point's position in the grid
        col_idx = int((x - grid_map.info.pose.position.x) / resolution)
        row_idx = int((y - grid_map.info.pose.position.y) / resolution)

        # Ensure the indices are within bounds
        if 0 <= row_idx < num_rows and 0 <= col_idx < num_cols:
            # Sum the RGB values in the corresponding grid cell
            color_sum[row_idx, col_idx, 0] += r
            color_sum[row_idx, col_idx, 1] += g
            color_sum[row_idx, col_idx, 2] += b
            # Increment the point count in the corresponding cell
            point_count[row_idx, col_idx] += 1

    # Compute the average color per grid cell where points exist
    avg_color = np.zeros((num_rows, num_cols, 3), dtype=np.uint8)
    valid_cells = point_count > 0
    avg_color[valid_cells] = (color_sum[valid_cells] / point_count[valid_cells][:, None]).astype(np.uint8)

    # Create a mask based on the average color (example: filter based on intensity)
    # Create the mask based on the brown color range
    intensity_mask = ((avg_color[..., 0] >= brown_lower[0]) & (avg_color[..., 0] <= brown_upper[0]) &
                (avg_color[..., 1] >= brown_lower[1]) & (avg_color[..., 1] <= brown_upper[1]) &
                (avg_color[..., 2] >= brown_lower[2]) & (avg_color[..., 2] <= brown_upper[2]))
    # Convert the binary mask to a grayscale image (0: black, 255: white)
    mask_image = np.zeros((num_rows, num_cols), dtype=np.uint8)
    mask_image[intensity_mask] = 255  # Set mask regions to white

    # Publish the mask as an image over a ROS topic
    publish_mask_as_image(mask_image)

    # Optionally, publish the mask as a Float32MultiArray (if needed)
    mask_msg = Float32MultiArray(data=intensity_mask.flatten().astype(np.float32))
    mask_pub.publish(mask_msg)

def extract_rgb(rgb_float):
    """Helper function to extract RGB from a packed float value."""
    rgb_int = int(rgb_float)
    r = (rgb_int >> 16) & 0x0000ff
    g = (rgb_int >> 8) & 0x0000ff
    b = (rgb_int) & 0x0000ff
    return r, g, b

def publish_mask_as_image(mask_image):
    """Converts the mask image to a ROS Image message and publishes it."""
    image_msg = bridge.cv2_to_imgmsg(mask_image, encoding="mono8")  # Convert to ROS Image message
    mask_image_pub.publish(image_msg)
    rospy.loginfo("Mask image published as ROS Image message")

def gridmap_callback(gridmap_msg):
    global grid_map
    grid_map = gridmap_msg

if __name__ == '__main__':
    rospy.init_node('pointcloud_color_to_elevation_mask', anonymous=True)

    # Subscriber for PointCloud2
    pointcloud_sub = rospy.Subscriber('/concatenated_cloud_passthrough_xyz_downsampled', PointCloud2, pointcloud_callback)

    # Subscriber for GridMap
    gridmap_sub = rospy.Subscriber('/elevation_mapping/elevation_map', GridMap, gridmap_callback)

    # Publisher for the binary mask as a Float32MultiArray
    mask_pub = rospy.Publisher('/color_mask', Float32MultiArray, queue_size=10)

    # Publisher for the mask image as a ROS Image message
    mask_image_pub = rospy.Publisher('/mask_image', Image, queue_size=10)

    rospy.spin()
