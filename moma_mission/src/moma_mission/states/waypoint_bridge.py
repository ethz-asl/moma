import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from moma_mission.core import StateRos
from moma_mission.missions.piloting.frames import Frames


TASK_TYPE_POSE_UUID = ""
TASK_TYPE_ACTION_VISUAL_UUID = "5b186846-f73a-44b7-8f19-267e382fbea7"
TASK_TYPE_ACTION_GAUGE_UUID = (
    "9e3a4325-61c6-4d5a-9636-5cb6581cbe3a"  # Structured Light UUID
)


class WaypointBroadcasterState(StateRos):
    """
    Broadcast waypoints from the gRCS inspection plan as ROS transforms
    """

    def __init__(
        self, ns, outcomes=["POSE", "ACTION_VISUAL", "ACTION_GAUGE", "Failure"]
    ):
        StateRos.__init__(self, ns=ns, outcomes=outcomes)
        self.waypoint_pose_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.map_frame = self.get_scoped_param("map_frame", Frames.map_frame)
        self.pose_waypoint_frame = self.get_scoped_param(
            "pose_waypoint_frame", "waypoint"
        )
        self.action_visual_waypoint_frame = self.get_scoped_param(
            "action_visual_waypoint_frame", "action_visual"
        )
        self.action_gauge_waypoint_frame = self.get_scoped_param(
            "action_gauge_waypoint_frame", "action_gauge"
        )

    def run(self):
        gRCS = self.global_context.ctx.gRCS
        waypoint = gRCS.get_next_waypoint()

        waypoint_pose = TransformStamped()
        waypoint_pose.header.frame_id = self.map_frame
        waypoint_pose.header.stamp = rospy.get_rostime()
        waypoint_pose.transform.translation.x = waypoint.x
        waypoint_pose.transform.translation.y = waypoint.y
        waypoint_pose.transform.translation.z = waypoint.z
        waypoint_pose.transform.rotation.x = waypoint.param2
        waypoint_pose.transform.rotation.y = waypoint.param3
        waypoint_pose.transform.rotation.z = waypoint.param4
        waypoint_pose.transform.rotation.w = waypoint.param1

        result = "Failure"
        if waypoint.task_type_uuid == TASK_TYPE_POSE_UUID:
            rospy.loginfo("Current waypoint is a POSE waypoint.")
            waypoint_pose.child_frame_id = self.pose_waypoint_frame
            result = "POSE"
        elif waypoint.task_type_uuid == TASK_TYPE_ACTION_VISUAL_UUID:
            rospy.loginfo("Current waypoint is an ACTION_VISUAL waypoint.")
            waypoint_pose.child_frame_id = self.action_visual_waypoint_frame

            # TODO HACK pass on waypoint param as desired turning angle
            rospy.loginfo(
                f"Passing action visual waypoint param1 ({waypoint.param1} rad) to valve turning controller."
            )
            self.set_context("valve_desired_angle", waypoint.param1)

            result = "ACTION_VISUAL"
        elif waypoint.task_type_uuid == TASK_TYPE_ACTION_GAUGE_UUID:
            rospy.loginfo("Current waypoint is an ACTION_GAUGE waypoint.")
            waypoint_pose.child_frame_id = self.action_gauge_waypoint_frame
            result = "ACTION_GAUGE"
        else:
            rospy.logerr(f"Unknown waypoint task type uuid {waypoint.task_type_uuid}")
            return "Failure"

        self.waypoint_pose_broadcaster.sendTransform(waypoint_pose)
        rospy.sleep(2.0)
        return result


class WaypointReachedState(StateRos):
    def __init__(self, ns, outcomes=["Completed", "Next", "Failure"]):
        StateRos.__init__(self, ns=ns, outcomes=outcomes)

    def run(self):
        gRCS = self.global_context.ctx.gRCS
        return gRCS.set_waypoint_reached()
