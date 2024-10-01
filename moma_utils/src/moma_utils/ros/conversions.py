import numpy as np
import scipy
import geometry_msgs.msg
import rospy
import std_msgs.msg

#  from moma_utils.transform import Rotation, Transform
from moma_utils.spatial import Rotation, Transform


def from_point_msg(msg: geometry_msgs.msg.Point) -> np.array:
    return np.r_[msg.x, msg.y, msg.z]


def from_quat_msg(msg: geometry_msgs.msg.Quaternion) -> Rotation:
    return Rotation.from_quat([msg.x, msg.y, msg.z, msg.w])


def from_pose_msg(msg: geometry_msgs.msg.Pose) -> Transform:
    position = from_point_msg(msg.position)
    orientation = from_quat_msg(msg.orientation)
    return Transform(orientation, position)


def from_transform_msg(msg: geometry_msgs.msg.Transform) -> Transform:
    translation = from_vector3_msg(msg.translation)
    rotation = from_quat_msg(msg.rotation)
    return Transform(rotation, translation)


def from_vector3_msg(msg: geometry_msgs.msg.Vector3) -> np.array:
    return np.r_[msg.x, msg.y, msg.z]


def to_color_msg(color: list) -> std_msgs.msg.ColorRGBA:
    msg = std_msgs.msg.ColorRGBA()
    msg.r = color[0]
    msg.g = color[1]
    msg.b = color[2]
    msg.a = color[3] if len(color) == 4 else 1.0
    return msg


def to_point_msg(point: np.array) -> geometry_msgs.msg.Point:
    msg = geometry_msgs.msg.Point()
    msg.x = point[0]
    msg.y = point[1]
    msg.z = point[2]
    return msg


def to_pose_msg(transform: Transform) -> geometry_msgs.msg.Pose:
    msg = geometry_msgs.msg.Pose()
    msg.position = to_point_msg(transform.translation)
    msg.orientation = to_quat_msg(transform.rotation)
    return msg


def to_pose_stamped_msg(transform : Transform, frame_id: str) -> geometry_msgs.msg.PoseStamped:
    msg = geometry_msgs.msg.PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose = to_pose_msg(transform)
    return msg


def to_quat_msg(orientation: scipy.spatial.transform.Rotation) -> geometry_msgs.msg.Quaternion:
    quat = orientation.as_quat()
    msg = geometry_msgs.msg.Quaternion()
    msg.x = quat[0]
    msg.y = quat[1]
    msg.z = quat[2]
    msg.w = quat[3]
    return msg


def to_transform_msg(transform: Transform) -> geometry_msgs.msg.Transform:
    msg = geometry_msgs.msg.Transform()
    msg.translation = to_vector3_msg(transform.translation)
    msg.rotation = to_quat_msg(transform.rotation)
    return msg


def to_vector3_msg(vector3: list) -> geometry_msgs.msg.Vector3:
    msg = geometry_msgs.msg.Vector3()
    msg.x = vector3[0]
    msg.y = vector3[1]
    msg.z = vector3[2]
    return msg


def waypoint_to_pose_msg(waypoint: list) -> geometry_msgs.msg.PoseStamped:
    """Converts 2D waypoint to a PoseStamped message.
    
    Arguments:
        waypoint {list} -- Format: [position_x, position_y, yaw], where yaw is in deg.
    """
    orn = Rotation.from_euler("z", waypoint[2], degrees=True).as_quat()
    msg = geometry_msgs.msg.PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = waypoint[0]
    msg.pose.position.y = waypoint[1]
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = orn[0]
    msg.pose.orientation.y = orn[1]
    msg.pose.orientation.z = orn[2]
    msg.pose.orientation.w = orn[3]
    return msg
