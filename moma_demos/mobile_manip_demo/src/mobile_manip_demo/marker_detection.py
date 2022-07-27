#!/usr/bin/env python

from typing import List

from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import PoseStamped, Pose
from mobile_manip_demo.msg import MarkerPoses
import rospy
import tf2_ros
import tf2_geometry_msgs


class MarkerDetectionNode:
    def __init__(self):
        """Initialize ROS nodes."""
        # Parameters
        frequency = rospy.get_param("detection_frequency", 10.0)
        self.rate = rospy.Rate(frequency)
        self.reference_frame = rospy.get_param("reference_frame", "/map")
        self.camera_frame = rospy.get_param("camera_frame", "/camera_link")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Keep track of markers ID with their last known poses
        # Key is an int representing the ID
        # Value is a Pose representing the pose of the marker with said ID
        self.known_markers = {}

        self.marker_publisher = rospy.Publisher(
            "/marker_poses", MarkerPoses, queue_size=10
        )

    def marker_callback(self):
        while not rospy.is_shutdown():
            detection_list = rospy.wait_for_message(
                "/tag_detections", AprilTagDetectionArray
            )
            rospy.loginfo("Received tag detection messages!")

            self.__digest_marker_poses(detection_list.detections)
            marker_poses_msg = self.__build_pub_msg()

            self.marker_publisher.publish(marker_poses_msg)
            self.rate.sleep()

    def __build_pub_msg(self):
        msg = MarkerPoses()
        for key, value in self.known_markers.items():
            msg.marker_ids.append(key)
            msg.poses.append(value)

        return msg

    def __digest_marker_poses(self, current_detection: List[AprilTagDetection]) -> None:
        """Store the detected markers poses to build robot knowledge."""
        for i, detection in enumerate(current_detection):
            marker_pose = self.__transform_marker_pose(detection.pose)
            if marker_pose is not None:
                self.known_markers[detection.id] = marker_pose

    def __transform_marker_pose(self, marker_pose: PoseStamped) -> Pose or None:
        """Transform the marker pose in the desired reference frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.reference_frame,
                rospy.Time(0),
                rospy.Duration(5),
            )
        except tf2_ros.LookupException:
            return None

        transformed_pose = self.tf_listener.transformPose(marker_pose, t)
        return transformed_pose.pose


def main():
    rospy.init_node("MarkerDetectionNode")
    node = MarkerDetectionNode()

    try:
        node.marker_callback()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
