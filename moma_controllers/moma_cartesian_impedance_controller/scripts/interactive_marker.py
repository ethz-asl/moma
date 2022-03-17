#!/usr/bin/env python3

import rospy
import tf2_ros

from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarkerFeedback
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseStamped

int_marker = InteractiveMarker()
marker_pose = PoseStamped()
menu_handler = MenuHandler()
pose_pub = None

def make_sphere(radius):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = 2.0 * radius
    marker.scale.y = 2.0 * radius
    marker.scale.z = 2.0 * radius
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.4
    return marker


def publisher_callback(feedback):
    marker_pose.header.stamp = rospy.get_rostime()
    pose_pub.publish(marker_pose)


def process_feedback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        marker_pose.pose.position.x = feedback.pose.position.x
        marker_pose.pose.position.y = feedback.pose.position.y
        marker_pose.pose.position.z = feedback.pose.position.z
        marker_pose.pose.orientation = feedback.pose.orientation
    server.applyChanges()


def wait_for_initial_pose(feedback, base_frame, target_frame):
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    try:
        trans = buffer.lookup_transform(base_frame, target_frame, rospy.Time(), rospy.Duration(10.0))
        marker_pose.header.frame_id = base_frame
        marker_pose.header.stamp = rospy.get_rostime()
        marker_pose.pose.orientation = trans.transform.rotation
        marker_pose.pose.position = trans.transform.translation
        int_marker.pose = marker_pose.pose
        if feedback is not None:
            server.insert(int_marker, process_feedback)
            server.applyChanges()
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as exc:
        rospy.logwarn(exc)
        return False
    return True


if __name__ == "__main__":
    rospy.init_node("equilibrium_pose_node")

    base_frame_id = rospy.get_param("~base_frame")
    target_frame_id = rospy.get_param("~target_frame")
    topic_name = rospy.get_param("~topic_name")

    if not wait_for_initial_pose(None, base_frame_id, target_frame_id):
        rospy.logerr("Failed to initialize the marker pose.")

    pose_pub = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
    server = InteractiveMarkerServer("interactive_pose_marker")
    int_marker.header.frame_id = base_frame_id
    int_marker.scale = 0.3
    int_marker.name = "equilibrium_pose"
    int_marker.description = ("Target Cartesian Pose")
    int_marker.pose = marker_pose.pose

    # insert a box
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.markers.append(make_sphere(0.1))
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    server.insert(int_marker, process_feedback)

    # add a menu handler
    menu_handler.insert("Reset Pose", callback= lambda fb : wait_for_initial_pose(fb, base_frame_id, target_frame_id))
    menu_handler.insert("Send Target Pose", callback=publisher_callback)
    menu_handler.apply( server, int_marker.name)

    # apply changes and spin
    server.applyChanges()
    rospy.spin()
