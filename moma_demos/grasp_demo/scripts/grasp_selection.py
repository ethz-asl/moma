import rospy
from sensor_msgs.msg import PointCloud2
from gpd_ros.msg import GraspConfigList
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped

class GraspSelection(object):
    def __init__(self):
        rospy.init_node("grasp_planner")
        self.use_vbpp = rospy.get_param("use_vbpp", False)
        self.stitched_cloud_sub = rospy.Subscriber('scan_action_node/cloud', PointCloud2, self._cloud_cb)
        self.grasp_planner_pub = rospy.Publisher('/cloud_stitched', PointCloud2)

    def _cloud_cb(self, msg):
        if self.use_vbpp:
            # Should probably do nothing. The grasp planning should be triggered when the
            # object to be picked is selected.
            raise NotImplementedError()
        else:
            self._plan_grasp(msg)

    def _plan_grasp(self, msg):
        self.grasp_planner_pub.publish(msg)
        try:
            grasp_candidates = rospy.wait_for_message(
                "/detect_grasps/clustered_grasps", GraspConfigList, timeout=30
            )
        except rospy.ROSException:
            rospy.loginfo("GPD server timed out")
            self._as.set_aborted(result)
            return

        if len(grasp_candidates.grasps) == 0:
            rospy.loginfo("No grasps detected")
            self._as.set_aborted(result)
            return

        grasp_pose = self.select_grasp_pose(grasp_candidates)
        self.selected_grasp_pub.publish(grasp_pose)

        rospy.loginfo("Scanning action succeeded")
        result.selected_grasp_pose = grasp_pose

    def select_grasp_pose(self, grasp_config_list):
        grasp = grasp_config_list.grasps[0]

        x_axis = np.r_[grasp.axis.x, grasp.axis.y, grasp.axis.z]
        y_axis = np.r_[grasp.binormal.x, grasp.binormal.y, grasp.binormal.z]
        z_axis = np.r_[grasp.approach.x, grasp.approach.y, grasp.approach.z]
        rot_mat = np.vstack([x_axis, y_axis, z_axis]).T

        if np.linalg.det(rot_mat) < 0:
            rospy.loginfo(
                "Grasp pose vectors not a right-handed system. Flipping y-axis."
            )
            y_axis *= -1
            rot_mat = np.vstack([x_axis, y_axis, z_axis]).T

        rot = Rotation.from_dcm(rot_mat)

        if x_axis[0] < 0:
            rospy.loginfo(
                "Flipped grasp pose. x-axis was pointing in negative direction"
            )
            rot = rot * Rotation.from_euler("z", 180, degrees=True)

        offset = rot.apply(
            [0.0, 0.0, 0.04]
        )  # GPD defines points at the hand palm, not the fingertip
        quat = rot.as_quat()

        pose = Pose()
        pose.position.x = grasp.position.x + offset[0]
        pose.position.y = grasp.position.y + offset[1]
        pose.position.z = grasp.position.z + offset[2]

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "panda_link0"

        return pose_stamped


def main():
    node = GraspSelection()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()

