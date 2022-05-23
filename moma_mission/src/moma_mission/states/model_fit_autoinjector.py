from typing import List

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from moma_mission.states.model_fit import ModelFitState


class ModelFitAutoinjectorState(ModelFitState):
    def _object_name(self) -> str:
        return "autoinjector"

    def _model_fit(
        self, keypoints_perception: List[Pose], frame: str
    ) -> TransformStamped:
        if len(keypoints_perception) != 2:
            rospy.logerr("Unexpected amount of keypoints")
            return None

        object_vector = [
            keypoints_perception[0].position.x - keypoints_perception[1].position.x,
            keypoints_perception[0].position.y - keypoints_perception[1].position.y,
            keypoints_perception[0].position.z - keypoints_perception[1].position.z,
        ]
        object_length = tf.transformations.vector_norm(object_vector)
        rospy.loginfo("Autoinjector length is {}".format(object_length))
        if not 0.12 <= object_length <= 0.19:
            rospy.logerr("Autoinjector length is not within the expected range")
            return None
        x_axis_dir = tf.transformations.unit_vector(object_vector)
        z_axis_dir_desired = [0, 0, -1]
        y_axis_dir = tf.transformations.unit_vector(
            np.cross(z_axis_dir_desired, x_axis_dir)
        )
        z_axis_dir = np.cross(x_axis_dir, y_axis_dir)
        assert np.allclose(z_axis_dir, tf.transformations.unit_vector(z_axis_dir))
        mat = tf.transformations.identity_matrix()
        mat[0:3, 0] = x_axis_dir
        mat[0:3, 1] = y_axis_dir
        mat[0:3, 2] = z_axis_dir
        quat = tf.transformations.quaternion_from_matrix(mat)

        object_pose = TransformStamped()
        object_pose.transform.translation.x = (
            keypoints_perception[0].position.x + keypoints_perception[1].position.x
        ) / 2
        object_pose.transform.translation.y = (
            keypoints_perception[0].position.y + keypoints_perception[1].position.y
        ) / 2
        object_pose.transform.translation.z = (
            keypoints_perception[0].position.z + keypoints_perception[1].position.z
        ) / 2
        object_pose.transform.rotation.x = quat[0]
        object_pose.transform.rotation.y = quat[1]
        object_pose.transform.rotation.z = quat[2]
        object_pose.transform.rotation.w = quat[3]
        return object_pose

    def _publish_marker(self, object_pose):
        pass
