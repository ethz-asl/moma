import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, Pose
from moma_mission.core import StateRos

from moma_mission.utils.transforms import *
from moma_mission.utils.robot import Robot


class ValveManipulationUrdfState(StateRos):
    def __init__(self, ns):
        StateRos.__init__(self, ns=ns)

        self.world_frame = self.get_scoped_param("world_frame", "world")
        self.grasp_frame = self.get_scoped_param("grasp_frame", "grasp_point")
        self.turning_angle = np.deg2rad(
            self.get_scoped_param("turning_angle_deg", 45.0)
        )

        self.valve_description_name = self.get_scoped_param(
            "valve_description_name", "valve_description"
        )
        self.pinocchio_robot = Robot(self.valve_description_name)

        poses_topic = self.get_scoped_param("poses_topic", "/valve_poses")
        self.poses_publisher = rospy.Publisher(
            poses_topic, PoseArray, queue_size=1, latch=True
        )

        self.R_V_EE = R.from_euler(
            "xyz",
            self.get_scoped_param("grasp_orientation", [0.0, 180.0, -90.0]),
            degrees=True,
        ).as_dcm()
        self.T_V_EE = pin.SE3(self.R_V_EE, np.array([0.0, 0.0, 0.0]))

    def generate_valve_turning_poses(self, turning_angle, steps=20) -> PoseArray:
        """
        Generate a valve turning profile, using the urdf model as a method to compute subsequent grasp
        locations as a function of the turning angle
        """
        if turning_angle == 0:
            steps = 1

        poses = PoseArray()
        poses.header.frame_id = self.world_frame
        increment = 1.0 * turning_angle / steps
        q = np.array([0.0])

        rospy.loginfo(f"Generating trajectory: increment={increment}, steps={steps}")
        for i in range(steps):
            q[0] = i * increment
            T_W_V = self.pinocchio_robot.get_frame_placement(self.grasp_frame, q)
            T_W_V.translation[2] -= 0.022
            T_W_EE = T_W_V.act(self.T_V_EE)

            pose = se3_to_pose_ros(T_W_EE)
            poses.poses.append(pose)
        return poses

    def run(self):
        poses = self.generate_valve_turning_poses(self.turning_angle)
        self.poses_publisher.publish(poses)

        return "Completed"
