import rospy
import pinocchio as pin
from copy import deepcopy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros

from moma_mission.utils.transforms import pose_to_se3, tf_to_se3


def get_timed_path_to_target(start_pose, target_pose, linear_velocity, angular_velocity):
    """ Return a path from current pose to target timed with a specific velocity """
    if start_pose.header.frame_id != target_pose.header.frame_id:
        raise NameError("Start and target pose are in different frames")

    start = pose_to_se3(start_pose.pose)

    path = Path()
    path.header.stamp = rospy.get_rostime()
    path.header.frame_id = start_pose.header.frame_id

    pose_stamped_start = PoseStamped()
    pose_stamped_start = deepcopy(start_pose)
    pose_stamped_start.header.stamp = rospy.get_rostime()
    pose_stamped_start.header.frame_id = start_pose.header.frame_id
    pose_stamped_end = target_pose

    end = pose_to_se3(target_pose.pose)
    vel = pin.log6(end.actInv(start))
    max_lin = max(abs(vel.linear))  # linear velocity to get there in 1 sec
    max_ang = max(abs(vel.angular))  # angular velocity to get there in 1 sec

    reach_time = 1.0 * max(1.0, max(max_lin / linear_velocity, max_ang / angular_velocity))
    pose_stamped_end.header.stamp = rospy.Duration.from_sec(reach_time) + pose_stamped_start.header.stamp

    path.poses.append(pose_stamped_start)
    path.poses.append(pose_stamped_end)
    return path

