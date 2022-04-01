import numpy as np
import rospy
from geometry_msgs.msg import PoseArray

from moma_mission.utils.transforms import *
from moma_mission.utils.robot import Robot


class ValveUrdfPlanner:
    def __init__(self, robot: Robot, world_frame, grasp_frame, grasp_orientation):
        self.pinocchio_robot = robot
        self.world_frame = world_frame
        self.grasp_frame = grasp_frame
        R_v_ee = R.from_euler("xyz", grasp_orientation, degrees=True).as_dcm()
        self.T_v_ee = pin.SE3(R_v_ee, np.array([0.0, 0.0, 0.0]))

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
            T_w_v = self.pinocchio_robot.get_frame_placement(self.grasp_frame, q)
            T_w_v.translation[2] -= 0.022
            T_w_ee = T_w_v.act(self.T_v_ee)

            pose = se3_to_pose_ros(T_w_ee)
            poses.poses.append(pose)
        return poses
