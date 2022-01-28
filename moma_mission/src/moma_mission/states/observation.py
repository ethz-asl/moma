import rospy
import tf2_ros
import tf
from geometry_msgs.msg import Pose, TransformStamped
import numpy as np

from moma_mission.core import StateRos

from object_keypoints_ros.srv import KeypointsPerception, KeypointsPerceptionResponse, KeypointsPerceptionRequest

class SphericalSamplerState(StateRos):
    """
    Observe an object from a pose normal to a bounded slice of a spherical surface centered around an object,
    given an initial position estimate of the object.
    The pose is calculated by random sampling on the surface.
    """

    def __init__(self, ns, outcomes=['Completed', 'Failure']):
        StateRos.__init__(self, ns=ns, outcomes=outcomes)

        self.sample_name = self.get_scoped_param("sample_name", "observation")
        self.base_frame = self.get_scoped_param("base_frame", "world")
        self.object_position_estimate = self.get_scoped_param("object_position_estimate", [0, 0, 0])
        self.radius_range = self.get_scoped_param("radius_range", [0.8, 1])
        self.theta_range = self.get_scoped_param("theta_range", [0, 2 * np.pi])
        self.phi_range = self.get_scoped_param("phi_range", [0, np.pi / 2]) # Use upper hemisphere by default
        self.heading = self.get_scoped_param("heading", [1, 0, 0])
        self.pose_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def _spherical_to_cartesian(self, r, theta, phi):
        r_sin_phi = r * np.sin(phi)
        return np.array([r_sin_phi * np.cos(theta),
                         r_sin_phi * np.sin(theta),
                         r * np.cos(phi)])

    def _sample(self, range):
        return np.random.uniform(range[0], range[1])

    def run(self):
        coords = self._spherical_to_cartesian(self._sample(self.radius_range),
                                              self._sample(self.theta_range),
                                              self._sample(self.phi_range))
        z_axis_dir = -tf.transformations.unit_vector(coords)
        x_axis_dir_desired = self.heading
        y_axis_dir = tf.transformations.unit_vector(np.cross(z_axis_dir, x_axis_dir_desired))
        x_axis_dir = np.cross(y_axis_dir, z_axis_dir)
        assert np.allclose(x_axis_dir, tf.transformations.unit_vector(x_axis_dir))
        mat = tf.transformations.identity_matrix()
        mat[0:3, 0] = x_axis_dir
        mat[0:3, 1] = y_axis_dir
        mat[0:3, 2] = z_axis_dir
        quat = tf.transformations.quaternion_from_matrix(mat)

        sample_pose = TransformStamped()
        sample_pose.transform.translation.x = coords[0] + self.object_position_estimate[0]
        sample_pose.transform.translation.y = coords[1] + self.object_position_estimate[1]
        sample_pose.transform.translation.z = coords[2] + self.object_position_estimate[2]
        sample_pose.transform.rotation.x = quat[0]
        sample_pose.transform.rotation.y = quat[1]
        sample_pose.transform.rotation.z = quat[2]
        sample_pose.transform.rotation.w = quat[3]
        sample_pose.header.stamp = rospy.Time.now()
        sample_pose.header.frame_id = self.base_frame
        sample_pose.child_frame_id = self.sample_name
        self.pose_broadcaster.sendTransform(sample_pose)

        rospy.sleep(2.0)
        return 'Completed'
