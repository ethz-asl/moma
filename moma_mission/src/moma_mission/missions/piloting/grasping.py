import copy
import rospy
import tf2_ros
import numpy as np
import pinocchio as pin
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from moma_mission.missions.piloting.frames import Frames
from moma_mission.missions.piloting.valve import Valve
from moma_mission.utils.rotation import CompatibleRotation as R
from moma_mission.utils.transforms import (
    se3_to_pose_ros,
    pose_to_se3,
    numpy_to_pose_stamped,
    tf_to_se3,
)


def project_to_plane(plane_origin, plane_normal, p, in_plane=False):
    """
    Takes a non normalized plane normal, its origin, a point in 3d space and returns the closest point
    to p in the plane
    :param plane_origin:
    :param plane_normal:
    :param p:
    :param in_plane: set to True to only return the point vector in the plane frame
    :return:
    """
    p = np.asarray(p)
    plane_origin = np.asarray(plane_origin)
    plane_normal = np.asarray(plane_normal)
    plane_normal = plane_normal / np.linalg.norm(plane_normal)
    tangent_vector = (p - plane_origin) - np.dot(
        p - plane_origin, plane_normal
    ) * plane_normal
    if in_plane:
        return tangent_vector
    else:
        return plane_origin + tangent_vector


class GraspPlanner:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def get_transform(self, target, source):
        transform = self.tf_buffer.lookup_transform(
            target,
            source,
            rospy.Time(0),  # tf at first available time
            rospy.Duration(3),
        )
        return tf_to_se3(transform)

    def compute_candidate_grasps(
        self, radial_offset=0.0, normal_offset=0.0, rotation=None
    ):
        path = Path()
        path.header.frame_id = Frames.base_frame
        angles = np.linspace(start=0, stop=2 * np.pi, num=25)
        t_base_valve = self.get_transform(Frames.base_frame, Frames.valve_frame)

        radius = Valve.valve_radius
        for angle in angles:
            t = np.array(
                [
                    (radial_offset + radius) * np.cos(angle),
                    (radial_offset + radius) * np.sin(angle),
                    normal_offset,
                ]
            )
            orientation = np.ndarray(shape=(3, 3))
            orientation[:, 2] = np.array([0.0, 0.0, 1.0])
            radial_vector = np.array([t[0], t[1], 0.0])
            orientation[:, 0] = radial_vector / np.linalg.norm(radial_vector)
            orientation[:, 1] = np.cross(orientation[:, 2], orientation[:, 0])
            r = R.from_dcm(orientation).as_quat()
            q = pin.Quaternion(r[3], r[0], r[1], r[2])

            t_valve_grasp = pin.SE3(q, t)
            t_base_grasp = t_base_valve.act(t_valve_grasp)

            if rotation is not None:
                t_rel = np.array([0.0, 0.0, 0.0])
                q_rel = pin.Quaternion(
                    rotation[3], rotation[0], rotation[1], rotation[2]
                )
                t_grasp_tool = pin.SE3(q_rel, t_rel)
                t_base_grasp = t_base_grasp.act(
                    t_grasp_tool
                )  # should be t_base_tool but to keep one variable later on

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = Frames.base_frame
            pose_stamped.pose = se3_to_pose_ros(t_base_grasp)
            path.poses.append(pose_stamped)
        path.header.frame_id = Frames.base_frame
        return path

    def compute_candidate_lateral_grasps(self):
        return self.compute_candidate_grasps(rotation=Valve.quaternion_valve_latgrasp)

    def filter_grasps(self, poses, method="top"):
        methods = ["distance", "top"]
        if method not in methods:
            rospy.logerr(
                "Wrong grasp filtering method. Availables are: {}. Given {}".format(
                    methods, method
                )
            )

        best_grasp = poses[0]

        if method == "top":
            max_height = -np.inf
            for candidate in poses:
                if candidate.pose.position.z > max_height:
                    best_grasp = copy.deepcopy(candidate)
                    max_height = candidate.pose.position.z
            return best_grasp

        if method == "distance":
            ee_pose = self.get_transform(
                target=Frames.base_frame, source=Frames.tool_frame
            )
            min_dist = np.inf
            for candidate in poses:
                grasp_pose = pose_to_se3(candidate.pose)
                dist = np.linalg.norm(grasp_pose.translation - ee_pose.translation)
                if dist < min_dist:
                    min_dist = dist
                    best_grasp = copy.deepcopy(candidate)
            return best_grasp

    def compute_lateral_approach_pose(self):
        candidates = self.compute_candidate_grasps(
            rotation=Valve.quaternion_valve_latgrasp,
            radial_offset=abs(Valve.lateral_grasp_offset),
            normal_offset=0.05,
        ).poses
        return self.filter_grasps(candidates)

    def compute_lateral_pre_grasp_pose(self):
        candidates = self.compute_candidate_grasps(
            rotation=Valve.quaternion_valve_latgrasp,
            radial_offset=abs(Valve.lateral_grasp_offset),
        ).poses
        return self.filter_grasps(candidates)

    def compute_lateral_grasp_pose(self):
        candidates = self.compute_candidate_grasps(
            rotation=Valve.quaternion_valve_latgrasp, radial_offset=0
        ).poses
        return self.filter_grasps(candidates)

    def compute_post_lateral_grasp(self):
        return self.compute_lateral_grasp(Valve.post_lateral_grasp_offset)

    def compute_grasp(self, relative_grasp_rotation, offset=0.0):
        """
        Computes the grasp pose when the adopted strategy is a lateral grasp
        Assumption: z of the valve pointing down
        Assumption: z of tool point out of the end effector
        """

        print(
            "\n\n\nGetting transform from {} to {}\n\n\n".format(
                Frames.valve_frame, Frames.tool_frame
            )
        )
        T_tool_valve = self.get_transform(
            target=Frames.tool_frame, source=Frames.valve_frame
        )
        origin = T_tool_valve.translation
        rotation = T_tool_valve.rotation
        normal = rotation[:, 2]

        # get the point on the plane which intersects with the perimeter
        base_point = [
            0,
            0,
            0,
        ]  # the base point is the point to project, in this case the tool frame
        plane_vector = project_to_plane(origin, normal, base_point, in_plane=True)
        plane_vector = plane_vector / np.linalg.norm(plane_vector) * Valve.valve_radius

        # position
        grasp_position = origin + plane_vector

        # orientation convention:
        # normal as the valve z axis
        # x axis as the radial vector
        # y axis resulting from cross<z, x>
        grasp_orientation = np.ndarray(shape=(3, 3))
        grasp_orientation[:, 2] = normal
        grasp_orientation[:, 0] = plane_vector / np.linalg.norm(plane_vector)
        grasp_orientation[:, 1] = np.cross(
            grasp_orientation[:, 2], grasp_orientation[:, 0]
        )
        grasp_orientation = np.dot(grasp_orientation, relative_grasp_rotation)

        T_tool_grasp = pin.SE3(grasp_orientation, grasp_position)
        T_grasp_graspdes = pin.SE3(
            pin.Quaternion(1, 0, 0, 0), np.array([0.0, 0.0, offset])
        )
        T_base_tool = self.get_transform(
            target=Frames.base_frame, source=Frames.tool_frame
        )
        T_base_grasp = T_base_tool.act(T_tool_grasp.act(T_grasp_graspdes))

        grasp_pose_ros = PoseStamped()
        grasp_pose_ros.header.frame_id = Frames.base_frame
        grasp_pose_ros.header.stamp = rospy.get_rostime()
        grasp_pose_ros.pose = se3_to_pose_ros(T_base_grasp)
        return grasp_pose_ros

    def compute_lateral_grasp(self, offset):
        return self.compute_grasp(
            offset=offset, relative_grasp_rotation=Valve.rotation_valve_latgrasp
        )

    # #######################################################
    # # Lateral grasps utility functions
    # #######################################################
    #
    #
    # def compute_pre_lateral_grasp():
    #     return _compute_lateral_grasp(valve_traj_data.base_frame, valve_traj_data.lateral_grasp_offset)
    #
    #
    # def compute_lateral_grasp():
    #     return _compute_lateral_grasp(valve_traj_data.base_frame, 0.0)
    #
    #
    #
    #
    # #######################################################
    # # Frontal grasps utility functions
    # #######################################################
    # def _compute_frontal_grasp(tool_frame, offset):
    #     return compute_grasp(tool_frame=tool_frame,
    #                          offset=offset,
    #                          reference_frame=valve_traj_data.base_frame,
    #                          valve_frame=valve_traj_data.valve_frame,
    #                          valve_radius=valve_traj_data.valve_radius,
    #                          relative_grasp_rotation=valve_traj_data.rotation_valve_frontgrasp)
    #
    #
    # def compute_pre_frontal_grasp():
    #     return _compute_frontal_grasp(valve_traj_data.base_frame, valve_traj_data.frontal_grasp_offset)
    #
    #
    # def compute_frontal_grasp():
    #     return _compute_frontal_grasp(valve_traj_data.base_frame, 0.0)
    #
    #
    # def compute_post_frontal_grasp():
    #     return _compute_frontal_grasp(valve_traj_data.tool_frame, valve_traj_data.frontal_grasp_offset)
