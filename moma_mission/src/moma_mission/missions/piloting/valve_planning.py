import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Path

from moma_mission.missions.piloting.frames import Frames
from moma_mission.utils.rotation import CompatibleRotation as R
from moma_mission.missions.piloting.valve_fitting import ValveModel


class ValvePlanner:
    """
    Utilities class to generate graps and paths given a valve model
    """

    def __init__(self, valve_model=ValveModel()):
        self.valve_model = valve_model

    def _get_all_grasping_poses(self, samples=100):
        """
        Get all potential grasping poses
        """
        thetas = [i * 2 * np.pi / samples for i in range(samples)]
        points = [self.valve_model.get_point_on_wheel(theta) for theta in thetas]

        zdes = np.array([0.0, 0.0, -1.0])
        xs = [self.valve_model.get_tangent_on_wheel(theta) for theta in thetas]
        zs = [zdes - np.dot(zdes, x) * x for x in xs]
        zs = [z / np.linalg.norm(z) for z in zs]
        ys = [np.cross(z, x) for x, z in zip(xs, zs)]
        rot = [np.array([x, y, z]).transpose() for x, y, z, in zip(xs, ys, zs)]
        quat = [R.from_matrix(r).as_quat() for r in rot]
        # Also store the index and the total count to be able
        # to restore continuous sequences after filtering grasps
        poses = [{"index": i, "samples": samples, "angle": t, "position": p, "orientation": q} for i, (p, q, t) in enumerate(zip(points, quat, thetas))]
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
            safety_distance = self.valve_model.spoke_radius

        pos = self.valve_model.spokes_positions
        rad = self.valve_model.spoke_radius
        return np.all((np.linalg.norm(grasp["position"] - pos, axis=1) - rad) > safety_distance)

    def _get_grasp_score(self, grasp):
        z_axis = R.from_quat(grasp["orientation"]).as_matrix()[:, 2]
        return np.dot(z_axis, np.array([0, 0, -1]))

    def _get_valid_paths(self, grasps_start, grasps, angle_max=2*np.pi):
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
                    angle += min((poses[-1]["angle"] - poses[-2]["angle"]) % (2 * np.pi),
                                 (poses[-2]["angle"] - poses[-1]["angle"]) % (2 * np.pi))
                if angle >= abs(angle_max):
                    break

            # Store path metadata for analysis
            # Note that due to different starting poses, paths for the same angle_max parameter
            # may have different actual turning angles and pose counts
            # Thus it is important to average the final score
            # Note that the angle is always positive, independent of turning direction
            # to simplify evaluation and avoid taking abs() in comparisons
            paths.append({"angle": angle, "score": score / len(poses), "poses": poses})

        return paths

    def get_path(self, angle_max=2*np.pi):
        """
        Given a maximum angle that we want to achieve withing a single manipulation step
        extrct a path with the following properties
        1. the first grasp does not intersect with a spoke
        2. all the poses along the path points toward the center (if possible)
        3. all poses along the path are continuous
        4. the path meets the turning angle (otherwise, use longest available one)
        5. prefer motion with a high score (if possible)

        @param angle_max: maximum turning angle (sign determines turning direction)
        """

        grasps = valve_planner._get_all_grasping_poses()
        grasps = filter(valve_planner._is_radial_grasp, grasps)
        grasps = list(filter(valve_planner._is_non_singular_grasp, grasps))
        grasps_start = filter(valve_planner._is_non_obstructed_grasp, grasps)

        all_paths = self._get_valid_paths(grasps_start, grasps, angle_max)
        valid_paths = [path for path in all_paths if path["angle"] >= abs(angle_max)]

        if len(all_paths) == 0:
            rospy.logerr("No valid path found")
            return None

        # If no path meets angle specification, choose longest path
        if len(valid_paths) == 0:
            rospy.logdebug_throttle(1.0, "No path meets max angle specification, using longest one")
            return max(all_paths, key=lambda path: path["angle"])

        rospy.logdebug_throttle(1.0, "Path with highest score is chosen")
        # Otherwise choose path with highest score
        return max(valid_paths, key=lambda path: path["score"])

    def poses_to_ros(self, poses, frame=Frames.map_frame):
        posesa = PoseArray()
        posesa.header.frame_id = Frames.map_frame
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

    def poses_to_ros_path(self, poses, frame=Frames.map_frame, speed=1.0):
        path = Path()
        path.header.frame_id = frame
        poses = self.poses_to_ros(poses, frame).poses
        t0 = rospy.get_rostime()
        dt = 1 / (speed * len(poses))
        for i, pose in enumerate(poses):
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose
            pose_stamped.header.frame_id = frame
            pose_stamped.header.stamp = t0 + rospy.Duration.from_sec(i * dt)


            
if __name__ == "__main__":
    rospy.init_node("valve_planner_test")
    
    poses_pub = rospy.Publisher("/plan", PoseArray, queue_size=1)
    
    valve_model = ValveModel(center=[0.5,0.5,0.5], depth=0.1)
    valve_planner = ValvePlanner(valve_model)

    #valve_model.transform(pitch_deg=45)
    #valve_model.turn(45)
    while not rospy.is_shutdown():
        valve_model.transform(pitch_deg=1)
        valve_model.turn(1)

        grasps = valve_planner._get_all_grasping_poses()
        grasps = filter(valve_planner._is_radial_grasp, grasps)
        grasps = filter(valve_planner._is_non_singular_grasp, grasps)
        grasps = filter(valve_planner._is_non_obstructed_grasp, grasps)
        path = valve_planner.get_path(angle_max=-np.pi/2)
        if path is None:
            continue
        rospy.loginfo_throttle(1.0, f"Turning angle: {path['angle']}, score: {path['score']}")
        grasps = path["poses"]

        poses_ros = valve_planner.poses_to_ros(grasps)
        poses_pub.publish(poses_ros)
    
        rospy.sleep(0.02)
    