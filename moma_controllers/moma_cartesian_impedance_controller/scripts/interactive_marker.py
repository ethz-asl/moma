#!/usr/bin/env python3
import sys
import rospy
import tf2_ros

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

int_marker = InteractiveMarker()
menu_handler = MenuHandler()
target_pub = None
poses = []


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


def add_target_callback(mode, feedback):
    global poses
    target_pose = PoseStamped()
    target_pose.header.stamp = rospy.get_rostime()
    target_pose.header.frame_id = int_marker.header.frame_id
    target_pose.pose = feedback.pose
    if mode == "pose":
        poses = [target_pose]
    else:
        poses.append(target_pose)

    interactive_path = Path()
    interactive_path.header = target_pose.header
    interactive_path.poses = poses
    interactive_path_pub.publish(interactive_path)


def publisher_callback(mode, feedback):
    global poses
    if len(poses) == 0:
        rospy.logwarn("Path is empty! Cannot send target.")
        return

    target = PoseStamped() if mode == "pose" else Path()
    target_pose = PoseStamped()
    target_pose.header.stamp = rospy.get_rostime()
    target_pose.header.frame_id = int_marker.header.frame_id
    target_pose.pose = feedback.pose

    if mode == "pose":
        target = poses[0]
    else:
        target.header = target_pose.header
        start_time = rospy.get_rostime()
        delta_time = 5.0
        for i, pose in enumerate(poses):
            pose.header.stamp = start_time + rospy.Duration.from_sec(i * delta_time)
        target.poses = poses

    target_pub.publish(target)
    reset_path(target)


def reset_path(target):
    global poses
    poses = []
    path = Path()
    path.header = target.header
    interactive_path_pub.publish(path)


def process_feedback(feedback):
    server.applyChanges()


def wait_for_initial_pose(feedback, base_frame, target_frame):
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    try:
        trans = buffer.lookup_transform(
            base_frame, target_frame, rospy.Time(0), rospy.Duration(10.0)
        )
        int_marker.header.frame_id = base_frame
        int_marker.pose.position = trans.transform.translation
        int_marker.pose.orientation = trans.transform.rotation
        if feedback is not None:
            server.insert(int_marker, process_feedback)
            server.applyChanges()
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ) as exc:
        rospy.logwarn(exc)
        return False
    return True


if __name__ == "__main__":
    rospy.init_node("equilibrium_pose_node")

    server_name = rospy.get_param("~server_name")
    base_frame_id = rospy.get_param("~base_frame")
    target_frame_id = rospy.get_param("~target_frame")
    topic_name = rospy.get_param("~topic_name")
    mode = rospy.get_param("~mode")  # can be 'path' or 'pose'

    if mode not in {"path", "pose"}:
        rospy.logerr(f"Wrong interactive marker mode: {mode}")
        sys.exit(-1)

    if not wait_for_initial_pose(None, base_frame_id, target_frame_id):
        rospy.logerr("Failed to initialize the marker pose.")

    msg_type = PoseStamped if mode == "pose" else Path
    target_pub = rospy.Publisher(topic_name, msg_type, queue_size=10)
    interactive_path_pub = rospy.Publisher(
        "/interactive_path", Path, queue_size=1, latch=True
    )

    server = InteractiveMarkerServer(server_name)
    int_marker.header.frame_id = base_frame_id
    int_marker.scale = 0.3
    int_marker.name = server_name + "_marker"
    int_marker.description = "Target Cartesian Pose"

    # insert a box
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 0
    control.markers.append(make_sphere(0.05))
    control.name = "sphere_rotate_x"
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
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
    menu_handler.insert(
        "Reset Pose",
        callback=lambda fb: wait_for_initial_pose(fb, base_frame_id, target_frame_id),
    )
    menu_handler.insert(
        "Add Target Pose",
        callback=lambda fb: add_target_callback(mode=mode, feedback=fb),
    )
    menu_handler.insert(
        "Send Target", callback=lambda fb: publisher_callback(mode=mode, feedback=fb)
    )
    menu_handler.apply(server, int_marker.name)

    # apply changes and spin
    server.applyChanges()
    rospy.spin()
