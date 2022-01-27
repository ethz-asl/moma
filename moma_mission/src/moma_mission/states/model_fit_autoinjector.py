from typing import List
from geometry_msgs.msg import Pose, TransformStamped

from moma_mission.states.model_fit import ModelFitState


class ModelFitAutoinjectorState(ModelFitState):

    def _object_name(self) -> str:
        return 'autoinjector'

    def _model_fit(self, keypoints_perception: List[Pose]) -> TransformStamped:
        object_pose = TransformStamped()
        object_pose.transform.translation.x = (keypoints_perception[0].position.x + keypoints_perception[1].position.x) / 2
        object_pose.transform.translation.y = (keypoints_perception[0].position.y + keypoints_perception[1].position.y) / 2
        object_pose.transform.translation.z = (keypoints_perception[0].position.z + keypoints_perception[1].position.z) / 2
        object_pose.transform.rotation.w = 1.0
        return object_pose

