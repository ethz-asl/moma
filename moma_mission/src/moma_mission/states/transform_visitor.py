import rospy
import numpy as np
import tf
import pinocchio as pin

from nav_msgs.msg import Path

from moma_mission.core import StateRosControl
from moma_mission.utils.transforms import *


class TransformVisitorState(StateRosControl):
    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)

        self.world_frame = self.get_scoped_param("world_frame", "world")
        self.target_frame = self.get_scoped_param("target_frame", "object")
        self.offset = self.get_scoped_param("offset", [0, 0, 0])
        self.angle_z = self.get_scoped_param("angle_z", 0)
        self.duration = self.get_scoped_param("duration", 0.0)
        self.timeout = self.get_scoped_param("timeout", max(30.0, 2 * self.duration))
        self.allow_flip = self.get_scoped_param("allow_flip", False)

        self.path_publisher = rospy.Publisher(
            self.get_scoped_param("path_topic", "/desired_path"), Path, queue_size=1)

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        path = Path()
        path.header.frame_id = self.world_frame
        if self.duration > 0:
            # Add the current position to the path,
            # such that the path motion velocity is respected,
            # which is not the case for a singleton path
            transform_se3 = self.get_transform(self.world_frame, self.ee_frame)
            pose_stamped = se3_to_pose_stamped(transform_se3, self.world_frame)
            pose_stamped.header.stamp = rospy.get_rostime()
            path.poses.append(pose_stamped)

        T_w_t = self.get_transform(self.world_frame, self.target_frame)
        H_t_toff = tf.transformations.rotation_matrix(self.angle_z, [0, 0, 1])
        H_t_toff[0:3, 3] = self.offset
        H_w_toff = T_w_t.homogeneous @ H_t_toff
        # Allow the target pose to be flipped by 180° around z axis
        # if that avoids cumbersome gripper rotation
        if self.allow_flip:
            T_w_ee = self.get_transform(self.world_frame, self.ee_frame)
            H_w_ee = T_w_ee.homogeneous
            if np.dot(H_w_toff[0:3, 0], H_w_ee[0:3, 0]) < 0:
                rospy.loginfo('Using a flipped pose')
                H_toff_toffflip = tf.transformations.rotation_matrix(np.pi, [0, 0, 1])
                H_w_toff = H_w_toff @ H_toff_toffflip
        T_w_toff = pin.SE3(H_w_toff)

        pose_stamped = se3_to_pose_stamped(T_w_toff, self.world_frame)
        pose_stamped.header.stamp = rospy.get_rostime() + rospy.Duration.from_sec(self.duration)
        path.poses.append(pose_stamped)

        self.path_publisher.publish(path)
        if not self.wait_until_reached(self.ee_frame, path.poses[-1], timeout=self.timeout):
            return 'Failure'

        rospy.sleep(2.0)
        return 'Completed'
