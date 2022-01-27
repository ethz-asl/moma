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
        self.ee_frame = self.get_scoped_param("ee_frame", "tool_frame")
        self.duration = self.get_scoped_param("duration", 10.0)

        self.path_publisher = rospy.Publisher(
            self.get_scoped_param("path_topic", "/desired_path"), Path, queue_size=1)

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        path = Path()
        path.header.frame_id = self.world_frame
        transform_se3 = self.get_transform(self.target_frame, self.world_frame)
        pose_stamped = se3_to_pose_stamped(transform_se3, self.world_frame)
        pose_stamped.header.stamp = rospy.get_rostime() + rospy.Duration.from_sec(self.duration)
        path.poses.append(pose_stamped)

        self.path_publisher.publish(path)
        if not self.wait_until_reached(self.ee_frame, path.poses[-1], quiet=True, linear_tolerance=0.02):
            return 'Failure'

        return 'Completed'