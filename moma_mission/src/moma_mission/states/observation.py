import rospy
import tf2_ros
import tf
import numpy as np
from scipy.spatial.transform import Rotation 
from geometry_msgs.msg import TransformStamped, PoseArray, Pose
from sensor_msgs.msg import CameraInfo

from moma_mission.core import StateRos

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

class FOVSamplerState(StateRos):
    """
    Computes an observation point of view, that given a known object size (in terms of bounding box)
    and known camera intrinsics, makes sure that the object is in the field of view and camera is pointig
    at the object.
    Assuming the camera Z is pointing to the object, the minimum conservative distance to keep the object in the field of 
    view is s * focal_distance / image_width. We approximate the numerator with max(kx * f, ky * f) and the denominator with
    min(width, height) as this information is readily available from the camera info topic. 
    """
    #TODO (giuseppe) this is computing the reference pose of the camera, but sending as reference pose for the tool
    # -> must account for offset

    def __init__(self, ns, outcomes=['Completed', 'Failure']):
        StateRos.__init__(self, ns=ns, outcomes=outcomes)

        # the name of the observation tf published
        self.sample_name = self.get_scoped_param("sample_name", "observation")

        # we use these names to resolve an available estimate in the current tf tree
        self.map_frame = self.get_scoped_param("map_frame", "world")
        self.object_frame = self.get_scoped_param("object_frame", "object")

        # the approximate lumped size of the object bounding box
        self.object_size = self.get_scoped_param("object_size", 1.0)

        # how big in the image the object should look like. 1.0 means getting the closest to the object
        self.image_fill_ratio = self.get_scoped_param("image_fill_ratio")

        # needed for FOV computation
        self.camera_info_topic = self.get_scoped_param("camera_info_topic", "/camera_info")
        self.heading = self.get_scoped_param("heading", [1, 0, 0])
        
        # how much to look lateraly across all candiate observation poses (it is the phi angle in sperical coords)
        self.lateral_view_angle = np.deg2rad(self.get_scoped_param("lateral_view_angle", 20))
        
        self.sample_poses = []
        self.num_samples = 1 + 8  # central  + intermediate angles
        
        self.attempts = 0
        self.first_run = True
        
        self.camera_info = None
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self._camera_info_cb)
        self.pose_array_pub = rospy.Publisher("/sampled_viewpoints", PoseArray, queue_size=1)
        self.pose_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def _spherical_to_cartesian(self, r, theta, phi):
        r_sin_phi = r * np.sin(phi)
        return np.array([r_sin_phi * np.cos(theta),
                         r_sin_phi * np.sin(theta),
                         r * np.cos(phi)])

    def _camera_info_cb(self, msg):
        self.camera_info = msg
        
    def run(self):
        if not self.camera_info:
            rospy.logerr("No camera_info message received on {}".format(self.camera_info_topic))
            return 'Failure'

        if self.first_run:
            w = np.min([self.camera_info.height, self.camera_info.width])
            f = np.max([self.camera_info.K[0], self.camera_info.K[4]])
            d = self.object_size * f / ( w * self.image_fill_ratio)

            T_map_obj = self.get_transform(self.map_frame, self.object_frame)
        
            thetas = [0.0] + [i * np.pi/4 for i  in range(8)]
            phis = [0.0] + [self.lateral_view_angle] * 8

            sample_pose_array = PoseArray()
            for theta, phi in zip(thetas, phis):
                coords = self._spherical_to_cartesian(d, theta, phi)

                z_axis_dir = -tf.transformations.unit_vector(coords)
                x_axis_dir_desired = self.heading
                y_axis_dir = tf.transformations.unit_vector(np.cross(z_axis_dir, x_axis_dir_desired))
                x_axis_dir = np.cross(y_axis_dir, z_axis_dir)
                assert np.allclose(x_axis_dir, tf.transformations.unit_vector(x_axis_dir))
                
                R_obs_obj = np.eye(3)
                R_obs_obj[:, 0] = x_axis_dir
                R_obs_obj[:, 1] = y_axis_dir
                R_obs_obj[:, 2] = z_axis_dir
                
                sample_pose_translation = T_map_obj.rotation @ coords + T_map_obj.translation
                sample_pose_orientation = Rotation.from_matrix(T_map_obj.rotation @ R_obs_obj.T).as_quat();
                
                sample_pose = Pose()
                sample_pose.position.x = sample_pose_translation[0]
                sample_pose.position.y = sample_pose_translation[1]
                sample_pose.position.z = sample_pose_translation[2]
                sample_pose.orientation.x = sample_pose_orientation[0]
                sample_pose.orientation.y = sample_pose_orientation[1]
                sample_pose.orientation.z = sample_pose_orientation[2]
                sample_pose.orientation.w = sample_pose_orientation[3]
                sample_pose_array.poses.append(sample_pose)
                
                sample_pose_tf = TransformStamped()
                sample_pose_tf.transform.translation = sample_pose.position
                sample_pose_tf.transform.rotation = sample_pose.orientation
                sample_pose_tf.header.stamp = rospy.Time.now()
                sample_pose_tf.header.frame_id = self.map_frame
                sample_pose_tf.child_frame_id = self.sample_name
                self.sample_poses.append(sample_pose_tf)
        
            sample_pose_array.header.frame_id = self.map_frame
            sample_pose_array.header.stamp = rospy.get_rostime()
            self.pose_array_pub.publish(sample_pose_array)
            rospy.sleep(1.0)
        
        if self.attempts < self.num_samples:
            self.pose_broadcaster.sendTransform(self.sample_poses[self.attempts])
            self.attempts += 1
            rospy.sleep(2.0)
            return 'Completed'
        else:
            rospy.logwarn("No more point of views.")
            return 'Failure'