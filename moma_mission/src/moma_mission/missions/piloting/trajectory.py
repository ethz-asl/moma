import rospy
import tf2_ros
import numpy as np
import pinocchio as pin
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path

from moma_mission.missions.piloting.frames import Frames
from moma_mission.missions.piloting.valve import Valve
from moma_mission.utils.transforms import se3_to_pose_ros, tf_to_se3
from moma_mission.utils.rotation import CompatibleRotation as R


class ValveTrajectoryGenerator(object):
    """
    Generates a trajectory for a manipulator to open a valve.
    The convention is that the grasp pose is with the z axis pointing the same as the
    the axis of rotation of the valve, and the x axis pointing inward, towards the valve
    center
    """

    def __init__(self):

        self.valve = Valve
        self.frames = Frames

        self.valve_origin = None
        self.valve_axis = None

        self.theta = 0.0
        self.theta_dot = 0.1  # rad/s
        self.theta_dot_max = 1.0
        self.theta_desired = 0.0

        # tf_ee_valve is the relative offset valve to end effector when grasped. This will be different
        # for lateral and frontal grasp
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_ref_valve = None
        self.tf_grasp_valve = None
        self.tf_ref_valve_bc = tf2_ros.StaticTransformBroadcaster()
        self.target_pose_pub = rospy.Publisher(
            "/target_pose", PoseStamped, queue_size=1
        )

    def estimate_valve_from_lateral_grasp(self):
        """
        Estimate the current valve tf assuming a lateral stable grasp
        and the known fixed transform between the ee frame and the estimated valve
        frame
        """
        tf_ref_ee = self.get_end_effector_pose()
        tf_valve_grasp = pin.SE3(
            self.valve.rotation_valve_latgrasp, self.valve.translation_valve_latgrasp
        )
        self.tf_grasp_valve = tf_valve_grasp.inverse()
        # grasp == ee
        self.tf_ref_valve = tf_ref_ee.act(self.tf_grasp_valve)
        self.reset_valve_tf()

    def estimate_valve_from_frontal_grasp(self):
        """
        Estimate the current valve tf assuming a frontal stable grasp
        and the known fixed transform between the ee frame and the estimated valve
        frame
        """
        tf_ref_ee = self.get_end_effector_pose()
        self.tf_grasp_valve = pin.SE3(
            self.valve.rotation_valve_frontgrasp,
            self.valve.translation_valve_frontgrasp,
        ).inverse()
        # grasp == ee
        self.tf_ref_valve = tf_ref_ee.act(self.tf_grasp_valve)
        self.reset_valve_tf()

    def reset_valve_tf(self):
        self.valve_origin = self.tf_ref_valve.translation
        self.valve_axis = self.tf_ref_valve.rotation[:, 2]

        tf_ref_valve_ros = TransformStamped()
        tf_ref_valve_ros.header.stamp = rospy.Time.now()
        tf_ref_valve_ros.header.frame_id = self.frames.base_frame
        tf_ref_valve_ros.child_frame_id = self.frames.valve_haptic_frame
        tf_ref_valve_ros.transform.translation.x = self.tf_ref_valve.translation[0]
        tf_ref_valve_ros.transform.translation.y = self.tf_ref_valve.translation[1]
        tf_ref_valve_ros.transform.translation.z = self.tf_ref_valve.translation[2]
        q = R.from_dcm(self.tf_ref_valve.rotation).as_quat()
        tf_ref_valve_ros.transform.rotation.x = q[0]
        tf_ref_valve_ros.transform.rotation.y = q[1]
        tf_ref_valve_ros.transform.rotation.z = q[2]
        tf_ref_valve_ros.transform.rotation.w = q[3]

        self.tf_ref_valve_bc.sendTransform(tf_ref_valve_ros)

    def compute_target(self, theta):
        """
        Compute the new target point advancing the rotation angle of the valve
        in the valve frame anc then converting into reference frame
        """
        q = R.from_euler("xyz", [0.0, 0.0, theta], degrees=False).as_quat()
        t = np.array(
            [
                self.valve.valve_radius * np.cos(theta),
                self.valve.valve_radius * np.sin(theta),
                0.0,
            ]
        )
        tf_valve_grasp = pin.SE3(pin.Quaternion(q[3], q[0], q[1], q[2]), t)

        # Apply rotation offset and project to ref frame
        tf_grasp_eedes = pin.SE3(self.tf_grasp_valve.rotation, np.array([0, 0, 0]))
        tf_ref_eedes = self.tf_ref_valve.act(tf_valve_grasp.act(tf_grasp_eedes))

        target_pose = PoseStamped()
        target_pose.pose = se3_to_pose_ros(tf_ref_eedes)
        target_pose.header.frame_id = self.frames.base_frame
        target_pose.header.stamp = rospy.get_rostime()
        return target_pose

    def reset(self):
        self.theta_desired = 0.0
        self.theta = 0.0

    def advance(self, dt):
        """
        Increment theta according to current velocity and step size and computes the new
        target pose for the end effector
        :param dt:
        :return:
        """
        self.theta_desired += self.theta_dot * dt
        target = self.compute_target(self.theta_desired)
        self.target_pose_pub.publish(target)

    def adapt_velocity(self):
        pass
        # error = self.theta_desired - self.theta
        # self.theta_dot = np.min([self.theta_dot_max, 1.0/error])

    def get_end_effector_pose(self):
        """Retrieve the end effector pose. Let it fail if unable to get the transform"""
        transform = self.tf_buffer.lookup_transform(
            self.frames.base_frame,  # target frame
            self.frames.tool_frame,  # source frame
            rospy.Time(0),  # tf at first available time
            rospy.Duration(3),
        )
        return tf_to_se3(transform)

    def run(self, dt, theta_target):
        rate = rospy.Rate(1 / dt)
        while self.theta_desired < theta_target and not rospy.is_shutdown():
            self.adapt_velocity()
            self.advance(dt)
            rate.sleep()
        return True

    def get_path(
        self, angle_start_deg, angle_end_deg, speed_deg=5.0, angle_delta_deg=1.0
    ):
        rospy.loginfo(
            "Computing trajectory: angle start: {}, angle end: {}, speed: {}".format(
                angle_start_deg, angle_end_deg, speed_deg
            )
        )
        angle_start = np.deg2rad(angle_start_deg)
        angle_end = np.deg2rad(angle_end_deg)
        speed = np.deg2rad(speed_deg)
        angle_delta = np.deg2rad(angle_delta_deg)

        if speed == 0:
            rospy.logwarn("Trajectory generator: velocity is zero!")
            return None

        if angle_end == angle_start:
            rospy.logwarn("Trajectory generator: start and end angles are the same.")
            return None

        speed = abs(speed)
        dt = abs(angle_delta / speed)
        angle_delta = speed * dt
        direction = (angle_end - angle_start) / abs(angle_end - angle_start)
        rospy.loginfo(
            "angle delta is: {}, direction is: {}".format(angle_delta, direction)
        )

        time = 0.0
        path = Path()
        start_time = rospy.get_rostime()
        angle = angle_start
        while True:
            pose = self.compute_target(angle)
            pose.header.stamp = rospy.Duration.from_sec(time) + start_time
            angle += direction * angle_delta
            time += dt
            path.poses.append(pose)

            if (angle - angle_end) * direction > 0:
                break

        path.header = path.poses[0].header
        rospy.loginfo("Generated trajectory with {} poses.".format(len(path.poses)))
        return path
