#!/usr/bin/env python
"""ROS node for the AprilTag detection."""

from typing import List

from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from geometry_msgs.msg import Pose
from mobile_manip_demo.msg import MarkerPoses
import rospy
import tf2_ros


class MarkerDetectionNode:
    def __init__(self):
        """Initialize ROS nodes."""
        rospy.init_node("marker_detection_node")
        # Parameters
        frequency = rospy.get_param("detection_frequency", 10.0)
        self.rate = rospy.Rate(frequency)
        self.reference_frame = rospy.get_param("reference_frame", "map")

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
            rospy.sleep(0.4)

            self.__digest_marker_poses(detection_list.detections)
            marker_poses_msg = self.__build_pub_msg()

            self.marker_publisher.publish(marker_poses_msg)
            self.rate.sleep()

    def __build_pub_msg(self):
        msg = MarkerPoses()
        for key, value in self.known_markers.items():
            msg.header.frame_id = self.reference_frame
            msg.marker_ids.append(key)
            msg.poses.append(value)

        return msg

    def __digest_marker_poses(self, current_detection: List[AprilTagDetection]) -> None:
        """Store the detected markers poses to build robot knowledge."""
        for i, detection in enumerate(current_detection):
            marker_pose = self.__transform_marker_pose(detection.id[0])
            if marker_pose is not None:
                self.known_markers[detection.id[0]] = marker_pose

    def __transform_marker_pose(self, id: int) -> Pose or None:
        """Transform the marker pose in the desired reference frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.reference_frame,
                "tag_" + str(id),
                rospy.Time(0),
                rospy.Duration(5),
            )
        except tf2_ros.LookupException:
            return None

        transformed_pose = Pose()
        transformed_pose.position.x = t.transform.translation.x
        transformed_pose.position.y = t.transform.translation.y
        transformed_pose.position.z = t.transform.translation.z
        transformed_pose.orientation.x = t.transform.rotation.x
        transformed_pose.orientation.y = t.transform.rotation.y
        transformed_pose.orientation.z = t.transform.rotation.z
        transformed_pose.orientation.w = t.transform.rotation.w

        return transformed_pose


def main():
    node = MarkerDetectionNode()

    try:
        node.marker_callback()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
