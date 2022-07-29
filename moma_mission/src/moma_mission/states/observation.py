import rospy
import tf2_ros
import tf
import random
import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped, PoseArray, Pose
from sensor_msgs.msg import CameraInfo

from moma_mission.core import StateRos
from moma_mission.utils.transforms import se3_to_pose_ros, se3_to_transform


def spherical_to_cartesian(r, theta, phi):
    """Get a point on the hemisphere given its spherical coordinates"""
    r_sin_phi = r * np.sin(phi)
    return np.array(
        [r_sin_phi * np.cos(theta), r_sin_phi * np.sin(theta), r * np.cos(phi)]
    )


def spherical_to_pose(r, theta, phi, heading):
    """Give spherical coordinates and a desired heading returns the full pose"""
    coords = spherical_to_cartesian(r, theta, phi)

    z_axis_dir = -tf.transformations.unit_vector(coords)
    x_axis_dir_desired = heading
    y_axis_dir = tf.transformations.unit_vector(
        np.cross(z_axis_dir, x_axis_dir_desired)
    )
    x_axis_dir = np.cross(y_axis_dir, z_axis_dir)
    assert np.allclose(x_axis_dir, tf.transformations.unit_vector(x_axis_dir))

    T = np.eye(4)
    T[:3, 0] = x_axis_dir
    T[:3, 1] = y_axis_dir
    T[:3, 2] = z_axis_dir
    T[:3, 3] = coords
    T = pin.SE3(T)
    return T


def sample(range):
    """Utility function to sample uniformely in the given range"""
    return np.random.uniform(range[0], range[1])


class SphericalSamplerState(StateRos):
    """
    Observe an object from a pose normal to a bounded slice of a spherical surface centered around an object,
    given an initial position estimate of the object.
    The pose is calculated by random sampling on the surface.
    """

    def __init__(self, ns, outcomes=["Completed", "Failure"]):
        StateRos.__init__(self, ns=ns, outcomes=outcomes)

        self.sample_name = self.get_scoped_param("sample_name", "observation")
        self.base_frame = self.get_scoped_param("base_frame", "world")
        self.object_position_estimate = self.get_scoped_param(
            "object_position_estimate", [0, 0, 0]
        )
        self.radius_range = self.get_scoped_param("radius_range", [0.8, 1])
        self.theta_range = self.get_scoped_param("theta_range", [0, 2 * np.pi])
        self.phi_range = self.get_scoped_param(
            "phi_range", [0, np.pi / 2]
        )  # Use upper hemisphere by default
        self.heading = self.get_scoped_param("heading", [1, 0, 0])
        self.pose_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def run(self):
        T_obj_cam = spherical_to_pose(
            r=sample(self.radius_range),
            theta=sample(self.theta_range),
            phi=sample(self.theta_range),
            heading=self.heading,
        )
        sample_pose = se3_to_transform(
            T_obj_cam, rospy.Time.now(), self.base_frame, self.sample_name
        )
        sample_pose.transform.translation.x += self.object_position_estimate[0]
        sample_pose.transform.translation.y += self.object_position_estimate[1]
        sample_pose.transform.translation.z += self.object_position_estimate[2]
        self.pose_broadcaster.sendTransform(sample_pose)
        rospy.sleep(2.0)
        return "Completed"


class FOVSamplerState(StateRos):
    """
    Computes an observation point of view, that given a known object size (in terms of bounding box)
    and known camera intrinsics, makes sure that the object is in the field of view and camera is pointing
    at the object.
    Assuming the camera Z is pointing to the object, the minimum conservative distance to keep the object in the field of
    view is s * focal_distance / image_width. We approximate the numerator with max(kx * f, ky * f) and the denominator with
    min(width, height) as this information is readily available from the camera info topic.
    """

    def __init__(self, ns, outcomes=["Completed", "Failure"]):
        StateRos.__init__(self, ns=ns, outcomes=outcomes, input_keys=["get_next_pose"])

        # the name of the observation tf published
        self.sample_name = self.get_scoped_param("sample_name", "observation")

        # we use these names to resolve an available estimate in the current tf tree
        self.map_frame = self.get_scoped_param("map_frame", "world")
        self.object_frame = self.get_scoped_param("object_frame", "object")

        # Use these frame ids to find the offset for the transform, which is where
        # the controlled frame goes, not the camera frame.
        self.camera_frame = self.get_scoped_param(
            "camera_frame", "camera_optical_frame"
        )
        self.controlled_frame = self.get_scoped_param(
            "controlled_frame", "controlled_frame"
        )

        # the approximate lumped size of the object bounding box
        self.object_size = self.get_scoped_param("object_size", 1.0)

        # how big in the image the object should look like. 1.0 means getting the closest to the object
        self.image_fill_ratio_range = self.get_scoped_param(
            "image_fill_ratio_range", [1.0, 1.0]
        )

        # needed for FOV computation
        self.camera_info_topic = self.get_scoped_param(
            "camera_info_topic", "/camera_info"
        )

        self.heading = self.get_scoped_param("heading", "range")
        self.heading_range = np.deg2rad(
            self.get_scoped_param("heading_range_deg", [-20, 20])
        )
        self.theta_range = np.deg2rad(
            self.get_scoped_param("theta_range_deg", [-20, 20])
        )
        # how much to look laterally across all candidate observation poses (it is the phi angle in spherical coords)
        self.phi_range = np.deg2rad(self.get_scoped_param("phi_range_deg", [0, 20]))

        self.sample_poses = None
        self.num_samples = self.get_scoped_param("num_samples", 8)

        self.attempts = None

        self.camera_info = None
        self.sample_pose_array = None

        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._camera_info_cb
        )
        self.pose_array_pub = rospy.Publisher(
            "/sampled_viewpoints", PoseArray, queue_size=1, latch=True
        )
        self.pose_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def _camera_info_cb(self, msg):
        self.camera_info = msg

    def _init_sample_poses(self):
        self.sample_poses = []
        self.sample_pose_array = PoseArray()
        self.attempts = 0

        w = np.min([self.camera_info.height, self.camera_info.width])
        f = np.max([self.camera_info.K[0], self.camera_info.K[4]])

        T_map_obj = self.get_transform(self.map_frame, self.object_frame)
        T_cam_ctrl = self.get_transform(self.camera_frame, self.controlled_frame)

        for _ in range(self.num_samples):
            d = self.object_size * f / (w * sample(self.image_fill_ratio_range))
            theta = sample(self.theta_range)
            phi = sample(self.phi_range)
            heading = self.heading
            if self.heading == "random":
                heading = np.append(
                    tf.transformations.unit_vector(tf.transformations.random_vector(2)),
                    0.0,
                )
            elif self.heading == "range":
                heading_angle = sample(self.heading_range)
                heading = [np.cos(heading_angle), np.sin(heading_angle), 0.0]

            T_obj_cam = spherical_to_pose(d, theta, phi, heading)
            T_map_cam = T_map_obj.act(T_obj_cam)
            T_map_ctrl = T_map_cam.act(T_cam_ctrl)

            self.sample_pose_array.poses.append(se3_to_pose_ros(T_map_cam))
            self.sample_poses.append(
                se3_to_transform(
                    T_map_ctrl, rospy.Time.now(), self.map_frame, self.sample_name
                )
            )
        self.sample_pose_array.header.frame_id = self.map_frame
        self.sample_pose_array.header.stamp = rospy.get_rostime()
        self.pose_array_pub.publish(self.sample_pose_array)
        rospy.sleep(1.0)

    def run_with_userdata(self, userdata):
        if not self.camera_info:
            rospy.logerr(
                "No camera_info message received on {}".format(self.camera_info_topic)
            )
            return "Failure"

        if not userdata.get_next_pose:
            self._init_sample_poses()

        if self.attempts < len(self.sample_poses):
            self.pose_broadcaster.sendTransform(self.sample_poses[self.attempts])
            self.attempts += 1
            rospy.loginfo(f"Pose attempt {self.attempts} / {len(self.sample_poses)}")
            rospy.sleep(1.0)
            return "Completed"
        else:
            rospy.logwarn("No more point of views.")
            return "Failure"
