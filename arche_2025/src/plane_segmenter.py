#!/usr/bin/env python

import rospy
import ros_numpy
import numpy as np
import open3d as o3d
import tf2_ros
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty, EmptyResponse
from visualization_msgs.msg import Marker, MarkerArray


class PlaneSegmenter:
    def __init__(self):
        rospy.init_node('plane_segmenter')

        # Parameters
        self.distance_threshold = rospy.get_param("~distance_threshold", 0.005)
        self.ransac_n = rospy.get_param("~ransac_n", 3)
        self.num_iterations = rospy.get_param("~num_iterations", 1000)

        # Subscribers and publishers
        self.pointcloud_sub = rospy.Subscriber(
            "/camera/depth/color/points", PointCloud2, self.pc_callback)
        self.marker_pub = rospy.Publisher(
            "/segmented_plane_markers", MarkerArray, queue_size=1, latch=True)
        self.plane_tf_pub = rospy.Publisher(
            "/segmented_plane_transform", TransformStamped, queue_size=1, latch=True)
        self.segment_service = rospy.Service(
            "~segment_plane", Empty, self.segment_callback)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.latest_cloud = None
        rospy.loginfo(
            "PlaneSegmenter initialized and waiting for service call.")

    def pc_callback(self, msg):
        self.latest_cloud = msg

    def segment_callback(self, req):
        if self.latest_cloud is None:
            rospy.logwarn("No point cloud received yet.")
            return EmptyResponse()

        rospy.loginfo("Received service call. Segmenting plane...")

        # Convert PointCloud2 to Open3D point cloud
        cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(
            self.latest_cloud)
        xyz = ros_numpy.point_cloud2.get_xyz_points(
            cloud_array, remove_nans=True)

        if len(xyz) == 0:
            rospy.logwarn("No valid points in cloud.")
            return EmptyResponse()

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.distance_threshold,
            ransac_n=self.ransac_n,
            num_iterations=self.num_iterations
        )

        [a, b, c, d] = plane_model

        inlier_ratio = len(inliers) / float(len(pcd.points))

        rospy.loginfo(
            "Plane model: %.3fx + %.3fy + %.3fz + %.3f = 0", a, b, c, d)
        rospy.loginfo("Inliers: %d / %d (%.1f%% inliers)",
                      len(inliers), len(pcd.points), 100.0 * inlier_ratio)

        # Visualize the plane as a square patch
        self.publish_plane_marker(a, b, c, d, np.asarray(pcd.points)[inliers])

        return EmptyResponse()

    def publish_plane_transform(self, centroid, quat, parent_frame="camera_link", child_frame="segmented_plane"):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = centroid[0]
        t.transform.translation.y = centroid[1]
        t.transform.translation.z = centroid[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.plane_tf_pub.publish(t)
        self.tf_broadcaster.sendTransform(t)

    def publish_plane_marker(self, a, b, c, d, inlier_points):
        from scipy.spatial.transform import Rotation as R

        centroid = np.mean(inlier_points, axis=0)
        normal = np.array([a, b, c])
        normal = normal / np.linalg.norm(normal)

        # Ensure normal points toward camera
        if np.dot(normal, centroid) > 0:
            normal = -normal

        # Reference camera axes in optical frame
        camera_x = np.array([1.0, 0.0, 0.0])  # right in image
        camera_y = np.array([0.0, 1.0, 0.0])  # down in image

        # Project camera_x onto the plane to get plane X-axis
        plane_z = normal
        plane_x = camera_x - np.dot(camera_x, plane_z) * plane_z
        if np.linalg.norm(plane_x) < 1e-6:
            # In degenerate case (e.g. frontal wall), use camera_y
            plane_x = camera_y - np.dot(camera_y, plane_z) * plane_z
        plane_x /= np.linalg.norm(plane_x)

        # Plane Y-axis
        plane_y = np.cross(plane_z, plane_x)

        # Construct rotation matrix: columns are the plane axes
        R_plane = np.stack([plane_x, plane_y, plane_z], axis=1)  # 3x3

        # Convert to quaternion
        from scipy.spatial.transform import Rotation as R
        quat = R.from_matrix(R_plane).as_quat()

        u = plane_x
        v = plane_y

        # Project points onto (u, v) plane basis
        rel_points = inlier_points - centroid
        u_coords = rel_points @ u
        v_coords = rel_points @ v

        scale_x = np.max(u_coords) - np.min(u_coords)
        scale_y = np.max(v_coords) - np.min(v_coords)
        scale_z = 0.0005  # Thin plane

        marker = Marker()
        marker.header.frame_id = self.latest_cloud.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "plane"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = centroid[0]
        marker.pose.position.y = centroid[1]
        marker.pose.position.z = centroid[2]
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        self.marker_pub.publish(MarkerArray(markers=[marker]))

        self.publish_plane_transform(
            centroid, quat, parent_frame=self.latest_cloud.header.frame_id)


if __name__ == "__main__":
    try:
        PlaneSegmenter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
