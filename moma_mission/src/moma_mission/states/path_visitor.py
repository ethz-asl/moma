import rospy
import numpy as np
import tf
import pinocchio as pin

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path

from moma_mission.core import StateRosControl
from moma_mission.utils.transforms import *


def _cartesian_dist(pos1, pos2):
    return np.sqrt(np.power(pos1.x - pos2.x, 2) +
                   np.power(pos1.y - pos2.y, 2) +
                   np.power(pos1.z - pos2.z, 2))


def _angular_dist(rot1, rot2):
    rot = np.dot(rot1, rot2.T)
    theta = (np.trace(rot) - 1)/2
    return np.arccos(theta)


class PathVisitorState(StateRosControl):
    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)

        self.control_frame = self.get_scoped_param("control_frame", "tracking_camera_odom")
        self.target_frame = self.get_scoped_param("target_frame", "object")
        self.offset = self.get_scoped_param("offset", [0, 0, 0])
        self.angle_z = self.get_scoped_param("angle_z", 0)
        self.duration = self.get_scoped_param("duration", 0.0)
        self.timeout_factor = self.get_scoped_param("timeout_factor", 2)
        self.linear_speed = self.get_scoped_param("linear_speed", 0.1) # m/s
        self.angular_speed = self.get_scoped_param("angular_speed", 0.5) # rad/s
        self.section = self.get_scoped_param("section", "all") # "first", "last"
        self.mode = self.get_scoped_param("mode", "path") # "path", "pose"
        self.poses = None

        if self.mode not in {"path", "pose"}:
            raise NameError(f"Wrong path following mode: {self.mode}")

        self.poses_subscriber = rospy.Subscriber(
            self.get_scoped_param("poses_topic", "/poses"), PoseArray, self._poses_msg, queue_size=1)

        self.pose_publisher = rospy.Publisher(
            self.get_scoped_param("pose_topic", "/desired_pose"), PoseStamped, queue_size=1)

        self.path_publisher = rospy.Publisher(
            self.get_scoped_param("path_topic", "/desired_path"), Path, queue_size=1)

    def _poses_msg(self, msg: PoseArray):
        self.poses = msg

    def run(self):
        if self.poses is None:
            rospy.logerr(f"No poses received yet, can't publish path")
            return 'Failure'

        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        path = Path()
        path.header.frame_id = self.control_frame
        # Add the current position to the path,
        # such that the path motion velocity from the starting pose is respected
        transform_se3 = self.get_transform(self.control_frame, self.ee_frame)
        pose_stamped = se3_to_pose_stamped(transform_se3, self.control_frame)
        pose_stamped.header.stamp = rospy.get_rostime()
        path.poses.append(pose_stamped)

        H_t_toff = tf.transformations.rotation_matrix(self.angle_z, [0,0,1])
        H_t_toff[0:3, 3] = self.offset
        T_t_toff = pin.SE3(H_t_toff)

        poses = self.poses.poses
        if self.section == 'all':
            pass
        elif self.section == 'first':
            poses = [poses[0]]
        elif self.section == 'last':
            poses = [poses[-1]]
        else:
            rospy.logerr("Unknown value for 'section'")
            return 'Failure'

        t = rospy.get_rostime()

        for pose in poses:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.control_frame

            # Transform to correct frame, independent of what the original PoseArray was using
            T_x_t = pose_to_se3(pose)
            T_c_x = self.get_transform(self.control_frame, self.poses.header.frame_id)
            # And add a constant offset, if desired
            T_c_t = T_c_x.act(T_x_t)
            T_c_toff = T_c_t.act(T_t_toff)

            pose_stamped.pose = se3_to_pose_ros(T_c_toff)

            # Calculate motion speed based on linear and angular distance
            cartesian_dist = _cartesian_dist(path.poses[-1].pose.position, pose_stamped.pose.position)
            angular_dist = _angular_dist(pose_to_se3(path.poses[-1].pose).rotation, T_c_toff.rotation)
            dt = cartesian_dist / self.linear_speed + angular_dist / self.angular_speed
            t += rospy.Duration.from_sec(dt)
            pose_stamped.header.stamp = t

            path.poses.append(pose_stamped)

        if self.mode == 'path':
            self.path_publisher.publish(path)
        elif self.mode == 'pose':
            for i, pose in enumerate(path.poses[1:]):
                self.pose_publisher.publish(pose)
                rospy.sleep(path.poses[i+1].header.stamp.to_sec() - path.poses[i].header.stamp.to_sec())

        if not self.wait_until_reached(self.ee_frame, path.poses[-1], timeout=max(self.timeout_factor * (t.to_sec() - rospy.get_rostime().to_sec()), 5.0)):
            return 'Failure'

        rospy.sleep(2.0)
        return 'Completed'
