import geometry_msgs.msg
import numpy as np
import rospy
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField

from scipy.spatial.transform import Rotation

from moma_utils.transform import Rotation, Transform


def to_point_msg(position):
    """Convert a numpy array to a Point message."""
    msg = geometry_msgs.msg.Point()
    msg.x = position[0]
    msg.y = position[1]
    msg.z = position[2]
    return msg


def from_point_msg(msg):
    """Convert a Point messag to a numpy array."""
    return np.r_[msg.x, msg.y, msg.z]


def from_vector3_msg(msg):
    """Convert a Vector3 message to a numpy array."""
    return np.r_[msg.x, msg.y, msg.z]


def to_vector3_msg(vector3):
    """Convert numpy array to a Vector3 message."""
    msg = geometry_msgs.msg.Vector3()
    msg.x = vector3[0]
    msg.y = vector3[1]
    msg.z = vector3[2]
    return msg


def from_quat_msg(msg):
    """Convert a Quaternion message to a Rotation object."""
    return Rotation.from_quat([msg.x, msg.y, msg.z, msg.w])


def to_quat_msg(orientation):
    """Convert a Rotation object to a Quaternion message."""
    quat = orientation.as_quat()
    msg = geometry_msgs.msg.Quaternion()
    msg.x = quat[0]
    msg.y = quat[1]
    msg.z = quat[2]
    msg.w = quat[3]
    return msg


def to_pose_msg(transform):
    """Convert a Transform object to a Pose message."""
    msg = geometry_msgs.msg.Pose()
    msg.position = to_point_msg(transform.translation)
    msg.orientation = to_quat_msg(transform.rotation)
    return msg


def from_pose_msg(msg):
    """Convert a Pose msg to a Transform object."""
    position = from_point_msg(msg.position)
    orientation = from_quat_msg(msg.orientation)
    return Transform(orientation, position)


def from_transform_msg(msg):
    """Convert a Transform message to a Transform object."""
    translation = from_vector3_msg(msg.translation)
    rotation = from_quat_msg(msg.rotation)
    return Transform(rotation, translation)


def to_transform_msg(transform):
    msg = geometry_msgs.msg.Transform()
    msg.translation = to_vector3_msg(transform.translation)
    msg.rotation = to_quat_msg(transform.rotation)
    return msg


def to_point_cloud_msg(points, frame_id=None, stamp=None):
    """Convert a list of unstructured points to a PointCloud2 message.

    Args:
        points: Point coordinates as array of shape (N,3).
    """
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp or rospy.Time.now()

    msg.height = 1
    msg.width = points.shape[0]
    msg.is_bigendian = False
    msg.is_dense = False

    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    data = points

    msg.point_step = 12
    msg.row_step = msg.point_step * points.shape[0]
    msg.data = data.astype(np.float32).tostring()

    return msg


def waypoint_to_pose_msg(waypoint):
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
