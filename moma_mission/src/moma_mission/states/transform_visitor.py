import rospy
import tf
import pinocchio as pin
import rospy

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
        self.timeout = self.get_scoped_param("timeout", max(30.0, 2 * self.duration))

        self.path_publisher = rospy.Publisher(
            self.get_scoped_param("path_topic", "/desired_path"), Path, queue_size=1)

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        path = Path()
        path.header.frame_id = self.control_frame
        if self.duration > 0:
            # Add the current position to the path,
            # such that the path motion velocity is respected,
            # which is not the case for a singleton path
            transform_se3 = self.get_transform(self.control_frame, self.ee_frame)
            pose_stamped = se3_to_pose_stamped(transform_se3, self.control_frame)
            pose_stamped.header.stamp = rospy.get_rostime()
            path.poses.append(pose_stamped)

        T_c_t = self.get_transform(self.control_frame, self.target_frame)
        H_t_toff = tf.transformations.rotation_matrix(self.angle_z, [0,0,1])
        H_t_toff[0:3, 3] = self.offset
        H_c_toff = T_c_t.homogeneous @ H_t_toff
        T_c_toff = pin.SE3(H_c_toff)

        pose_stamped = se3_to_pose_stamped(T_c_toff, self.control_frame)
        pose_stamped.header.stamp = rospy.get_rostime() + rospy.Duration.from_sec(self.duration)
        path.poses.append(pose_stamped)

        self.path_publisher.publish(path)
        if not self.wait_until_reached(self.ee_frame, path.poses[-1], timeout=self.timeout):
            return 'Failure'

        rospy.sleep(2.0)
        return 'Completed'
