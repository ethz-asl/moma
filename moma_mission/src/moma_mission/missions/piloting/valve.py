import rospy
import numpy as np
from moma_mission.missions.piloting.frames import Frames
from moma_mission.utils.rotation import CompatibleRotation as R
from moma_mission.utils.transforms import numpy_to_pose_stamped


class Valve:
    """
    Contains all the info necessary for grasp and trajectory generation
    """

    # geometry
    valve_radius = 0.12

    # relative transformation from grasp to point on valve perimeter
    rotation_valve_latgrasp = R.from_euler(
        "zyx", [180.0, -90.0, 0.0], degrees=True
    ).as_dcm()
    quaternion_valve_latgrasp = R.from_euler(
        "zyx", [180.0, -90.0, 0.0], degrees=True
    ).as_quat()
    rotation_valve_frontgrasp = R.from_euler(
        "xyz", [0.0, 0.0, 180.0], degrees=True
    ).as_dcm()
    translation_valve_latgrasp = np.array([valve_radius, 0.0, 0.0])
    translation_valve_frontgrasp = np.array([valve_radius, 0.0, 0.0])

    # offsets
    frontal_grasp_offset = -0.1
    lateral_grasp_offset = -0.1
    post_lateral_grasp_offset = -0.1

    # detection poses
    det_pose_position_0 = np.array([0.096, 0.266, 0.559])
    det_pose_orientation_0 = np.array([0.011, 0.751, 0.660, 0.010])
    det_pose_position_1 = np.array([0.161, 0.220, 0.515])
    det_pose_orientation_1 = np.array([0.118, 0.791, 0.593, 0.092])
    det_pose_position_2 = np.array([0.359, 0.171, 0.490])
    det_pose_orientation_2 = np.array([0.053, 0.758, 0.650, 0.016])
    det_pose_position_3 = np.array([0.324, 0.526, 0.494])
    det_pose_orientation_3 = np.array([0.050, 0.733, 0.671, 0.096])
    det_pose_position_4 = np.array([0.469, 0.321, 0.470])
    det_pose_orientation_4 = np.array([-0.060, 0.702, 0.700, 0.115])
    det_pose_position_5 = np.array([-0.016, 0.398, 0.517])
    det_pose_orientation_5 = np.array([0.272, 0.670, 0.673, 0.156])

    det_poses = {
        "position": [
            det_pose_position_0,
            det_pose_position_1,
            det_pose_position_2,
            det_pose_position_3,
            det_pose_position_4,
            det_pose_position_5,
        ],
        "orientation": [
            det_pose_orientation_0,
            det_pose_orientation_1,
            det_pose_orientation_2,
            det_pose_orientation_3,
            det_pose_orientation_4,
            det_pose_orientation_5,
        ],
    }

    @classmethod
    def init_from_ros(cls):
        try:
            cls.valve_radius = rospy.get_param("~valve_radius")
            return True
        except Exception as exc:
            rospy.logerr(exc)
            return False

    @classmethod
    def get_detection_poses(cls):
        poses = []
        for translation, orientation in zip(
            cls.det_poses["position"], cls.det_poses["orientation"]
        ):
            print(translation)
            print(orientation)
            poses.append(
                numpy_to_pose_stamped(translation, orientation, Frames.base_frame)
            )
        return poses
