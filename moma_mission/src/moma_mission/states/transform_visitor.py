import rospy
import numpy as np
import tf
import pinocchio as pin

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from moma_mission.core import StateRosControl
from moma_mission.utils.transforms import *


class TransformVisitorState(StateRosControl):
    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)

        self.control_frame = self.get_scoped_param("control_frame", "odom")
        self.target_frame = self.get_scoped_param("target_frame", "object")
        self.offset = self.get_scoped_param("offset", [0, 0, 0])
        self.angle_z = self.get_scoped_param("angle_z", 0)
        self.duration = self.get_scoped_param("duration", 0.0)
        self.timeout = self.get_scoped_param(
            "timeout", max(2 * self.duration, 5.0) if self.duration > 0 else 30.0
        )
        self.mode = self.get_scoped_param("mode", "path")  # "path", "pose"
        self.allow_flip = self.get_scoped_param("allow_flip", False)
        self.linear_tolerance = self.get_scoped_param("linear_tolerance", 0.02)
        self.angular_tolerance = self.get_scoped_param("angular_tolerance", 0.1)

        if self.mode == "pose":
            self.pose_publisher = rospy.Publisher(
                self.get_scoped_param("pose_topic", "/desired_pose"),
                PoseStamped,
                queue_size=1,
            )
        elif self.mode == "path":
            self.path_publisher = rospy.Publisher(
                self.get_scoped_param("path_topic", "/desired_path"), Path, queue_size=1
            )

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return "Failure"

        path = Path()
        path.header.frame_id = self.control_frame
        if self.duration > 0 and self.mode == "path":
            # Add the current position to the path,
            # such that the path motion velocity is respected,
            # which is not the case for a singleton path
            transform_se3 = self.get_transform(self.control_frame, self.ee_frame)
            pose_stamped = se3_to_pose_stamped(transform_se3, self.control_frame)
            pose_stamped.header.stamp = rospy.get_rostime()
            path.poses.append(pose_stamped)

        T_c_t = self.get_transform(self.control_frame, self.target_frame)
        H_t_toff = tf.transformations.rotation_matrix(self.angle_z, [0, 0, 1])
        H_t_toff[0:3, 3] = self.offset
        H_c_toff = T_c_t.homogeneous @ H_t_toff

        if self.allow_flip:
            T_c_ee = self.get_transform(self.control_frame, self.ee_frame)
            H_c_ee = T_c_ee.homogeneous
            if np.dot(H_c_toff[0:3, 0], H_c_ee[0:3, 0]) < 0:
                rospy.loginfo("Using a flipped pose")
                H_toff_toffflip = tf.transformations.rotation_matrix(np.pi, [0, 0, 1])
                H_c_toff = H_c_toff @ H_toff_toffflip
        T_c_toff = pin.SE3(H_c_toff)

        pose_stamped = se3_to_pose_stamped(T_c_toff, self.control_frame)
        pose_stamped.header.stamp = rospy.get_rostime() + rospy.Duration.from_sec(
            self.duration
        )
        path.poses.append(pose_stamped)

        if self.mode == "path":
            self.path_publisher.publish(path)
        elif self.mode == "pose":
            self.pose_publisher.publish(path.poses[-1])

        if not self.wait_until_reached(
            self.ee_frame,
            path.poses[-1],
            timeout=self.timeout,
            linear_tolerance=self.linear_tolerance,
            angular_tolerance=self.angular_tolerance,
        ):
            return "Failure"

        rospy.sleep(2.0)
        return "Completed"
