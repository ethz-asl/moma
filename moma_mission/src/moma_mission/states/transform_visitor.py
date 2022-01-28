import rospy
import tf

from nav_msgs.msg import Path

from moma_mission.core import StateRosControl
from moma_mission.utils.transforms import *


class TransformVisitorState(StateRosControl):
    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)

        self.world_frame = self.get_scoped_param("world_frame", "world")
        self.target_frame = self.get_scoped_param("target_frame", "object")
        self.offset = self.get_scoped_param("offset", [0, 0, 0])
        self.duration = self.get_scoped_param("duration", 0.0)
        self.timeout = self.get_scoped_param("timeout", max(30.0, 2 * self.duration))

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

        transform_se3 = self.get_transform(self.world_frame, self.target_frame)
        pose_stamped = se3_to_pose_stamped(transform_se3, self.world_frame)
        offset = transform_se3.rotation @ self.offset
        pose_stamped.pose.position.x += offset[0]
        pose_stamped.pose.position.y += offset[1]
        pose_stamped.pose.position.z += offset[2]
        pose_stamped.header.stamp = rospy.get_rostime() + rospy.Duration.from_sec(self.duration)
        path.poses.append(pose_stamped)

        self.path_publisher.publish(path)
        if not self.wait_until_reached(self.ee_frame, path.poses[-1], timeout=self.timeout):
            return 'Failure'

        return 'Completed'
