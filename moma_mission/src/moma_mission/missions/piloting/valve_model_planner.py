import numpy as np
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from moma_mission.missions.piloting.frames import Frames
from moma_mission.missions.piloting.valve_fitting import ValveModel
from moma_mission.utils.rotation import CompatibleRotation as R


class ValveModelPlanner:
    """
    Generate graps and paths given a valve model

    @param robot_base_pose: pose of the robot base, should be in the same frame as valve_model
    """

    def __init__(self, valve_model=ValveModel(), robot_base_pose=None):
        self.valve_model = valve_model
        self.robot_base_heading = None

        if robot_base_pose is not None:
            # assert robot_base_pose.header.frame_id == valve_model.frame
            # self.robot_base_heading = robot_base_pose.rotation[:, 0]  # trivial approach
            self.robot_base_heading = valve_model.center - robot_base_pose.translation
            self.robot_base_heading /= np.linalg.norm(self.robot_base_heading)

    def _get_all_grasping_poses(self, inverted=False, samples=100):
        """
        Get all potential grasping poses

        @param inverted: whether the gripper heading should be inverted
        """
        thetas = [i * 2 * np.pi / samples for i in range(samples)]
        points = [self.valve_model.get_point_on_wheel(theta) for theta in thetas]

        zdes = np.array([0.0, 0.0, -1.0])
        xs = [self.valve_model.get_tangent_on_wheel(theta) for theta in thetas]
        if inverted:
            xs = [-x for x in xs]
        zs = [zdes - np.dot(zdes, x) * x for x in xs]
        zs = [z / np.linalg.norm(z) for z in zs]
        ys = [np.cross(z, x) for x, z in zip(xs, zs)]
        rot = [np.array([x, y, z]).transpose() for x, y, z, in zip(xs, ys, zs)]
        quat = [R.from_matrix(r).as_quat() for r in rot]
        # Also store the index and the total count to be able
        # to restore continuous sequences after filtering grasps
        poses = [
            {
                "index": i,
                "samples": samples,
                "angle": t,
                "position": p,
                "orientation": q,
            }
            for i, (p, q, t) in enumerate(zip(points, quat, thetas))
        ]
        return poses

    def _is_radial_grasp(self, grasp):
        """
        Check if grasp is pointing to the center
        """
        radial = self.valve_model.wheel_center - grasp["position"]
        z_axis = R.from_quat(grasp["orientation"]).as_matrix()[:, 2]
        return np.dot(z_axis, radial) >= 0

    def _is_non_singular_grasp(self, grasp, threshold=0.2):
        """
        Check if the tangential direction of the grasp is basically parallel to the z axis and would
        force a very unnatural and difficult grasp

        :param threshold: Projection to z-axis threshold
        """
        z_axis = R.from_quat(grasp["orientation"]).as_matrix()[:, 2]
        return np.dot(z_axis, np.array([0, 0, -1])) > threshold

    def _is_non_obstructed_grasp(self, grasp, safety_distance=-1):
        """
        Check if grasp is non obstructed, for example obstruction by spokes

        :param safety_distance: Minimum distance to spokes
        """
        if safety_distance < 0:
            safety_distance = 4 * self.valve_model.spoke_radius

        pos = self.valve_model.spokes_positions
        rad = self.valve_model.spoke_radius
        return np.all(
            (np.linalg.norm(grasp["position"] - pos, axis=1) - rad) > safety_distance
        )

    def _get_grasp_score(self, grasp):
        """
        Get a score for a particular grasping pose - the higher the better
        """
        z_axis = R.from_quat(grasp["orientation"]).as_matrix()[:, 2]
        z_score = np.dot(z_axis, np.array([0, 0, -1]))

        # Avoid entangled robot configurations by considering the robot base heading
        x_score = self._get_grasp_heading_score(grasp)

        return z_score + x_score

    def _get_grasp_heading_score(self, grasp):
        """
        Get a score for a particular grasping pose considering the heading w.r.t. the robot base
        """
        x_score = 0
        if self.robot_base_heading is not None:
            x_axis = R.from_quat(grasp["orientation"]).as_matrix()[:, 0]
            x_score = np.dot(x_axis, self.robot_base_heading)
        return x_score

    def _get_valid_paths(
        self, grasps_start, grasps, angle_max=2 * np.pi, inverted=False
    ):
        """
        Get a list of valid paths

        :param grasps_start: grasps that are valid for starting the path
        :param grasps: grasps that are valid during path execution, while the gripper is closed
        :param angle_max: desired turning angle (positive means forwards turning, negative means backwards turning)
        :return: list of dicts, containing paths and their corresponding scores
        """
        step = 1 if angle_max > 0 else -1

        # Longest chain of consecutive grasps
        paths = []
        for grasp_start in grasps_start:
            poses = []
            angle = 0
            score = 0
            index = grasp_start["index"]
            while True:
                # Find next pose in this path
                pose_next = [grasp for grasp in grasps if grasp["index"] == index]
                if len(pose_next) > 0:
                    poses.append(pose_next[0])
                else:
                    # No more poses in this path
                    break
                # Pay attention to wrap-around
                index = (index + step) % grasps[0]["samples"]

                score += self._get_grasp_score(pose_next[0])

                if len(poses) > 1:
                    # Shortest angular distance
                    angle += min(
                        (poses[-1]["angle"] - poses[-2]["angle"]) % (2 * np.pi),
                        (poses[-2]["angle"] - poses[-1]["angle"]) % (2 * np.pi),
                    )
                if angle >= abs(angle_max):
                    break

            # Store path metadata for analysis
            # Note that due to different starting poses, paths for the same angle_max parameter
            # may have different actual turning angles and pose counts
            # Thus it is important to average the final score
            # Note that the angle is always positive, independent of turning direction
            # to simplify evaluation and avoid taking abs() in comparisons
            paths.append(
                {
                    "angle": angle,
                    "score": score / len(poses),
                    "inverted": inverted,
                    "poses": poses,
                }
            )

        return paths

    def _get_all_valid_paths(self, angle_max, inverted=False):
        """
        Given a maximum angle that we want to achieve withing a single manipulation step
        extract a path with the following properties
        1. the first grasp does not intersect with a spoke
        2. all the poses along the path points toward the center (if possible)
        3. all poses along the path are continuous
        4. the path meets the turning angle (otherwise, use longest available one)

        @param angle_max: maximum turning angle (sign determines turning direction)
        @param inverted: whether the gripper heading should be inverted
        """
        grasps = self._get_all_grasping_poses(inverted=inverted)
        grasps = filter(self._is_radial_grasp, grasps)
        grasps = list(filter(self._is_non_singular_grasp, grasps))
        grasps_start = filter(self._is_non_obstructed_grasp, grasps)

        paths = self._get_valid_paths(
            grasps_start=grasps_start,
            grasps=grasps,
            angle_max=angle_max,
            inverted=inverted,
        )
        return paths

    def get_path(self, angle_max=2 * np.pi):
        """
        Given a maximum angle that we want to achieve withing a single manipulation step
        extract a path with the following properties
        1. [...] see description of _get_all_valid_paths
        2. prefer motion with a high score (if possible)

        @param angle_max: maximum turning angle (sign determines turning direction)
        """

        all_paths = self._get_all_valid_paths(angle_max=angle_max, inverted=False)
        if self.robot_base_heading is not None:
            all_paths += self._get_all_valid_paths(angle_max=angle_max, inverted=True)
        valid_paths = [path for path in all_paths if path["angle"] >= abs(angle_max)]

        if len(all_paths) == 0:
            rospy.logerr("No valid path found")
            return None

        # If no path meets angle specification, choose longest path
        if len(valid_paths) == 0:
            rospy.logdebug_throttle(
                1.0, "No path meets max angle specification, using longest one"
            )
            return max(all_paths, key=lambda path: path["angle"])

        # Otherwise choose path with highest score
        rospy.logdebug_throttle(1.0, "Path with highest score is chosen")
        return max(valid_paths, key=lambda path: path["score"])

    def get_path_approach_poses(self, path):
        """
        Get the optimal approaching poses for a given path, such that the robot heading is optimized,
        avoiding entangled configurations
        """
        best_pose = max(
            path["poses"], key=lambda pose: self._get_grasp_heading_score(pose)
        )
        best_pose_index = path["poses"].index(best_pose)
        approach_poses = path["poses"][best_pose_index::-1]
        # Better do the offset in the path_visitor
        # approach_poses = copy.deepcopy(path["poses"][best_pose_index::-1])
        # for pose in approach_poses:
        #     pose["position"][2] += z_offset
        return approach_poses

    def poses_to_ros(self, poses):
        posesa = PoseArray()
        posesa.header.frame_id = self.valve_model.frame
        posesa.header.stamp = rospy.get_rostime()
        for p in poses:
            pose = Pose()
            pose.position.x = p["position"][0]
            pose.position.y = p["position"][1]
            pose.position.z = p["position"][2]
            pose.orientation.x = p["orientation"][0]
            pose.orientation.y = p["orientation"][1]
            pose.orientation.z = p["orientation"][2]
            pose.orientation.w = p["orientation"][3]
            posesa.poses.append(pose)
        return posesa


if __name__ == "__main__":
    rospy.init_node("valve_planner_test")

    poses_pub = rospy.Publisher("/plan", PoseArray, queue_size=1, latch=True)
    approach_poses_pub = rospy.Publisher(
        "/plan_approach", PoseArray, queue_size=1, latch=True
    )

    # Robot mockup
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    from moma_mission.utils.transforms import tf_to_se3

    robot_pose_broadcaster = tf2_ros.StaticTransformBroadcaster()
    robot_pose = TransformStamped()
    robot_pose.header.stamp = rospy.Time.now()
    robot_pose.header.frame_id = "map"
    robot_pose.child_frame_id = "robot"
    robot_pose.transform.translation.x = 0.5
    robot_pose.transform.translation.y = 1.0
    robot_pose.transform.translation.z = 0.0
    robot_pose.transform.rotation.w = 1.0
    robot_pose_broadcaster.sendTransform(robot_pose)
    rospy.loginfo("Publishing mockup robot pose")

    valve_model = ValveModel(center=[0.5, 0.5, 0.5], depth=0.1)
    valve_planner = ValveModelPlanner(valve_model, tf_to_se3(robot_pose))

    # valve_model.transform(pitch_deg=45)
    # valve_model.turn(45)
    while not rospy.is_shutdown():
        valve_model.transform(pitch_deg=1)
        valve_model.turn(1)

        # grasps = valve_planner._get_all_grasping_poses()
        # grasps = filter(valve_planner._is_radial_grasp, grasps)
        # grasps = filter(valve_planner._is_non_singular_grasp, grasps)
        # grasps = filter(valve_planner._is_non_obstructed_grasp, grasps)

        # path = valve_planner.get_path(angle_max=np.pi/2) # turn forwards
        path = valve_planner.get_path(angle_max=-np.pi / 2)  # turn backwards
        if path is None:
            continue
        rospy.loginfo_throttle(
            1.0, f"Turning angle: {path['angle']}, score: {path['score']}"
        )
        grasps = path["poses"]

        poses_ros = valve_planner.poses_to_ros(grasps)
        poses_pub.publish(poses_ros)

        approach_poses = valve_planner.get_path_approach_poses(path)
        approach_poses_ros = valve_planner.poses_to_ros(approach_poses)
        approach_poses_pub.publish(approach_poses_ros)

        # rospy.sleep(0.02)
