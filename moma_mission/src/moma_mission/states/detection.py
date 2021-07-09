import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

from moma_mission.missions.piloting.frames import Frames
from moma_mission.core import StateRos
from moma_mission.utils.rotation import CompatibleRotation as R
import moma_mission.utils.transforms as transforms


class DetectionState(StateRos):
    """
    Subscribe to a pose stream coming from a detector and publish a static tf corresponding to the object
    """

    def __init__(self, ns, outcomes=['Completed', 'Failure']):
        StateRos.__init__(self, ns=ns, outcomes=outcomes)
        self.detection_pose_topic = self.get_scoped_param("detection_topic")
        self.base_frame_id = Frames.base_frame
        self.object_frame_id = Frames.valve_frame

        self.detection_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.detection_pose = None
        self.detection_sub = rospy.Subscriber(self.detection_pose_topic, PoseStamped, self.pose_callback)

    def pose_callback(self, msg):
        if not self.detection_pose:
            self.detection_pose = msg

    def run(self):
        rospy.loginfo("Sleeping 2 sec before recording the detection")
        rospy.sleep(2.0)

        while not self.detection_pose:
            if rospy.is_shutdown():
                return 'Failure'
            rospy.loginfo_throttle(1.0, "Waiting for the detection pose on: [{}]".format(self.detection_pose_topic))

        # B = base frame
        # D = detection frame
        # V = valve frame
        T_BD = transforms.get_transform(target=self.base_frame_id,
                                        source=self.detection_pose.header.frame_id)
        T_DV = transforms.pose_to_se3(self.detection_pose.pose)
        T_BV = T_BD.act(T_DV)

        object_pose = TransformStamped()
        object_pose.header.stamp = rospy.Time.now()
        object_pose.header.frame_id = self.base_frame_id
        object_pose.child_frame_id = self.object_frame_id
        object_pose.transform.translation.x = T_BV.translation[0]
        object_pose.transform.translation.y = T_BV.translation[1]
        object_pose.transform.translation.z = T_BV.translation[2]
        q = R.from_dcm(T_BV.rotation).as_quat()
        object_pose.transform.rotation.x = q[0]
        object_pose.transform.rotation.y = q[1]
        object_pose.transform.rotation.z = q[2]
        object_pose.transform.rotation.w = q[3]

        self.detection_broadcaster.sendTransform(object_pose)
        rospy.sleep(2.0)
        return 'Completed'
