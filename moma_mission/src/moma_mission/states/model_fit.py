import rospy
import tf2_ros
from geometry_msgs.msg import Pose, TransformStamped
from typing import List

from moma_mission.core import StateRos

from object_keypoints_ros.srv import KeypointsPerception, KeypointsPerceptionResponse, KeypointsPerceptionRequest

class ModelFitState(StateRos):
    """
    Call a perception node and match a given model to the perceived keypoints, returning a single model pose
    """

    def __init__(self, ns, outcomes=['Completed', 'Failure']):
        StateRos.__init__(self, ns=ns, outcomes=outcomes)
        self.object_pose_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.perception_srv_client = rospy.ServiceProxy(self.get_scoped_param("detection_topic", "object_keypoints_ros/perceive"), KeypointsPerception)

    def _object_name(self) -> str:
        return 'object'

    def _model_fit(self, keypoints_perception: List[Pose]) -> TransformStamped:
        object_pose = TransformStamped()
        object_pose.transform.translation.x = keypoints_perception[0].position.x
        object_pose.transform.translation.y = keypoints_perception[0].position.y
        object_pose.transform.translation.z = keypoints_perception[0].position.z
        object_pose.transform.rotation.w = 1.0
        return object_pose

    def run(self):
        try:
            self.perception_srv_client.wait_for_service(timeout=3)
        except rospy.ROSException as exc:
            rospy.logwarn("Service {} not available yet".format(self.perception_srv_client.resolved_name))
            return 'Failure'

        req = KeypointsPerceptionRequest()
        res: KeypointsPerceptionResponse = self.perception_srv_client.call(req)

        object_pose = self._model_fit(res.keypoints.poses)
        if object_pose is None:
            return 'Failure'
        object_pose.header = res.keypoints.header
        object_pose.header.stamp = rospy.Time.now()
        object_pose.child_frame_id = self._object_name()

        self.object_pose_broadcaster.sendTransform(object_pose)

        rospy.sleep(2.0)
        return 'Completed'
