#!/usr/bin/env python
# ROS wrapper node: sweep_search_node.py

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from sweep_search_core import find_valid_sweeps
from scipy.spatial.transform import Rotation as R

class SweepSearchNode:
    def __init__(self):
        rospy.init_node("sweep_search_node")

        self.min_len = rospy.get_param("~min_sweep_length", 1)
        self.max_len = rospy.get_param("~max_sweep_length", 16)
        self.width = rospy.get_param("~end_effector_width", 4)
        self.visualize = rospy.get_param("~visualize", True)

        self.sub = rospy.Subscriber(
            "/plane_heightmap_grid", OccupancyGrid, self.grid_cb, queue_size=1)
        self.marker_pub = rospy.Publisher(
            "/sweep_search/markers", MarkerArray, queue_size=1, latch=True)

        rospy.loginfo("SweepSearchNode initialized.")

    def grid_cb(self, msg):
        rospy.loginfo("Received grid. Processing.")
        grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        valid_sweeps, total_possible = find_valid_sweeps(
            grid, self.min_len, self.max_len, self.width)

        rospy.loginfo(
            f"Valid sweeps: {len(valid_sweeps)} / {total_possible} total candidates")

        if self.visualize:
            self.publish_markers(valid_sweeps, msg.info, msg.header.frame_id)

    def publish_markers(self, sweeps, info, frame_id):
        resolution = info.resolution
        origin = np.array([info.origin.position.x, info.origin.position.y, info.origin.position.z])
        quat = [
            info.origin.orientation.x,
            info.origin.orientation.y,
            info.origin.orientation.z,
            info.origin.orientation.w
        ]
        rot = R.from_quat(quat)

        markers = MarkerArray()
        for i, sweep in enumerate(sweeps):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = sweep['namespace']
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = resolution * 0.2
            norm_len = sweep['length'] / float(self.max_len)
            marker.color = ColorRGBA(1.0 - norm_len, 0.0, norm_len, 0.8)

            for gx, gy in [sweep['start'], sweep['end']]:
                local_xy = np.array([(gx + 0.5) * resolution, (gy + 0.5) * resolution, 0.0])
                world_xy = origin + rot.apply(local_xy)
                pt = Point(x=world_xy[0], y=world_xy[1], z=world_xy[2])
                marker.points.append(pt)

            markers.markers.append(marker)

        self.marker_pub.publish(markers)

if __name__ == '__main__':
    try:
        node = SweepSearchNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
