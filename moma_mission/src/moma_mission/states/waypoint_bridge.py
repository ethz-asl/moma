import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from moma_mission.core import StateRos
from moma_mission.missions.piloting.frames import Frames


class WaypointBroadcasterState(StateRos):
    """
    Broadcast waypoints from the gRCS inspection plan as ROS transforms
    """

    def __init__(self, ns, outcomes=["Completed", "Failure"]):
        StateRos.__init__(self, ns=ns, outcomes=outcomes)
        self.waypoint_pose_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.map_frame = self.get_scoped_param("map_frame", Frames.map_frame)
        self.waypoint_frame = self.get_scoped_param("waypoint_frame", "waypoint")

    def run(self):
        gRCS = self.global_context.ctx.gRCS
        waypoint = gRCS.get_next_waypoint()

        waypoint_pose = TransformStamped()
        waypoint_pose.header.frame_id = self.map_frame
        waypoint_pose.header.stamp = rospy.get_rostime()
        waypoint_pose.child_frame_id = self.waypoint_frame
        waypoint_pose.transform.translation.x = waypoint.x
        waypoint_pose.transform.translation.y = waypoint.y
        waypoint_pose.transform.translation.z = waypoint.z
        waypoint_pose.transform.rotation.x = waypoint.param2
        waypoint_pose.transform.rotation.y = waypoint.param3
        waypoint_pose.transform.rotation.z = waypoint.param4
        waypoint_pose.transform.rotation.w = waypoint.param1

        self.waypoint_pose_broadcaster.sendTransform(waypoint_pose)
        rospy.sleep(2.0)
        return "Completed"


class WaypointReachedState(StateRos):
    def __init__(self, ns, outcomes=["Completed", "Next", "Failure"]):
        StateRos.__init__(self, ns=ns, outcomes=outcomes)

    def run(self):
        gRCS = self.global_context.ctx.gRCS
        return gRCS.set_waypoint_reached()
