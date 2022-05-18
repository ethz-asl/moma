import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from moma_mission.core import StateRos
from moma_mission.missions.piloting.frames import Frames


class TransformRecorderState(StateRos):
    """
    Broadcast waypoints from the gRCS inspection plan as ROS transforms
    """

    def __init__(self, ns, outcomes=["Completed", "Failure"]):
        StateRos.__init__(self, ns=ns, outcomes=outcomes)
        self.transform_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.control_frame = self.get_scoped_param("control_frame", Frames.map_frame)
        self.target_frame = self.get_scoped_param("target_frame")  #  , self.ee_frame)
        self.store_frame = self.get_scoped_param("store_frame", "recorded")

    def run(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.control_frame,
                self.target_frame,
                rospy.Time(0),  # tf at first available time
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ):
            return "Failure"

        transform.child_frame_id = self.store_frame

        self.transform_broadcaster.sendTransform(transform)
        return "Completed"
