import rospy
import pinocchio as pin
from copy import deepcopy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros

from moma_mission.utils.transforms import pose_to_se3, tf_to_se3, get_transform


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


def wait_until_reached(target_frame, target_pose, linear_tolerance=0.01, angular_tolerance=0.1, timeout=15, quiet=False):
    """
    Returns once the target pose has been reached
    """
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tolerance_met = False
    time_elapsed = 0.0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if tolerance_met:
            return True

        t_current = get_transform(target=target_pose.header.frame_id, source=target_frame)
        t_desired = pose_to_se3(target_pose.pose)
        error = pin.log6(t_current.actInv(t_desired))  # motion that brings in 1 sec ee to target
        linear_error = max(abs(error.linear))
        angular_error = max(abs(error.angular))
        if linear_error < linear_tolerance and angular_error < angular_tolerance:
            tolerance_met = True

        rate.sleep()
        time_elapsed += 0.1

        if timeout != 0 and time_elapsed > timeout:
            if quiet:
                rospy.logwarn(
                    "Timeout elapsed while reaching a pose. Current distance to target is: {}".format(linear_error))
                return True
            else:
                rospy.logerror("Timeout elapsed while reaching a pose")
                return False
