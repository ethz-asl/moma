import numpy as np
import rospy
from nav_msgs.msg import Path
from moma_mission.core import StateRosControl

from moma_mission.utils.transforms import *
from moma_mission.utils.robot import Robot

class ValveManipulationState(StateRosControl):
    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)

        self.world_frame = self.get_scoped_param("world_frame", "world")
        self.reference_frame = self.get_scoped_param("reference_frame", "tracking_camera_odom")
        self.grasp_frame = self.get_scoped_param("grasp_frame", "grasp_point")
        self.turning_angle = np.deg2rad(
            self.get_scoped_param("turning_angle_deg", 45.0))
        self.turning_speed = np.deg2rad(
            self.get_scoped_param("turning_speed_deg", 5.0))

        self.valve_description_name = self.get_scoped_param(
            "valve_description_name", "valve_description")
        self.pinocchio_robot = Robot(self.valve_description_name)

        path_topic_name = self.get_scoped_param("path_topic_name", "/desired_path")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1, latch=True)

        self.R_V_EE = R.from_euler('xyz', [0.0, 180.0, -90.0], degrees=True).as_dcm()
        self.T_V_EE = pin.SE3(self.R_V_EE, np.array([0.0, 0.0, 0.0]))

    def generate_valve_turning_trajectory(self, turning_angle, turning_speed, steps = 20) -> Path:
        """
        Generate a valve turning profile, using the urdf model as a method to compute subsequent grasp
        locations as a function of the turning angle
        """
        if turning_angle == 0:
            steps = 1

        path = Path()
        path.header.frame_id = self.reference_frame
        increment = 1.0 * turning_angle / steps
        t0 = rospy.get_rostime()
        dt = increment / turning_speed
        q = np.array([0.0])

        rospy.loginfo(
            f"Generating trajectory: increment={increment}, steps={steps}, dt waypoints={dt}")
        for i in range(steps):
            q[0] = i * increment
            T_W_V = self.pinocchio_robot.get_frame_placement(
                self.grasp_frame, q)
            T_W_V.translation[2] -= 0.022
            T_W_EE = T_W_V.act(self.T_V_EE)
            T_R_W = tf_to_se3(self.tf_buffer.lookup_transform(self.reference_frame,  # target frame
                                                              self.world_frame,  # source frame
                                                              rospy.Time(0),
                                                              rospy.Duration(3)))
            T_R_EE = T_R_W.act(T_W_EE)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.reference_frame
            pose_stamped.header.stamp = t0 + rospy.Duration.from_sec(i * dt)
            pose_stamped.pose = se3_to_pose_ros(T_R_EE)
            path.poses.append(pose_stamped)
        return path

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        #rospy.sleep(2.0)

        path = self.generate_valve_turning_trajectory(self.turning_angle, self.turning_speed)
        self.path_publisher.publish(path)
        if not self.wait_until_reached(self.ee_frame, path.poses[-1], quiet=True):
            return 'Failure'


        return 'Completed'
