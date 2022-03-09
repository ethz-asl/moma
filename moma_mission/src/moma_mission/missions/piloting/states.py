
import rospy
import numpy as np
import tf2_ros
from typing import List
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, PoseArray

from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from object_keypoints_ros.srv import KeypointsPerception, KeypointsPerceptionRequest

from moma_mission.utils.transforms import se3_to_pose_ros
from moma_mission.utils.trajectory import get_timed_path_to_target
from moma_mission.utils.robot import Robot
from moma_mission.states.navigation import SingleNavGoalState
from moma_mission.states.model_fit import ModelFitState

from moma_mission.core import StateRosControl, StateRos
from moma_mission.utils.transforms import numpy_to_pose_stamped

from moma_mission.missions.piloting.frames import Frames
from moma_mission.missions.piloting.valve import Valve
from moma_mission.missions.piloting.valve_fitting import ValveFitter, ValveModel, RansacMatcher, Camera
from moma_mission.missions.piloting.valve_urdf_planner import ValveUrdfPlanner
from moma_mission.missions.piloting.valve_model_planner import ValveModelPlanner
from moma_mission.missions.piloting.grasping import GraspPlanner
from moma_mission.missions.piloting.trajectory import ValveTrajectoryGenerator
from moma_mission.missions.piloting.rcs_bridge import RCSBridge


gRCS = RCSBridge()


class SetUp(StateRos):
    def __init__(self, ns):
        StateRos.__init__(self, ns=ns)
        self.waypoints_file = self.get_scoped_param("waypoints_file_path")

    def run(self):
        global gRCS
        if not gRCS.read_params():
            rospy.logerr("Failed to read gRCS params")
            return 'Failure'

        if not gRCS.init_ros():
            rospy.logerr('Failed to initialize RCSBridge ros')
            return 'Failure'

        if not gRCS.read_waypoints_from_file(self.waypoints_file):
            rospy.logerr("Failed to read waypoints from file")
            return 'Failure'

        if not gRCS.upload_waypoints():
            rospy.logerr("Failed to upload waypoints.")
            return 'Failure'

        if not gRCS.upload_hl_action():
            rospy.logerr("Failed to upload high level actions")
            return 'Failure'
        return 'Completed'


class Idle(StateRos):
    """
    Parses the command from the gRCS and start the execution of the mission.
    Otherwise it idles
    """

    def __init__(self, ns):
        StateRos.__init__(self, ns=ns, outcomes=[
                          'ExecuteInspectionPlan', 'ExecuteManipulationPlan', 'Failure'])

    def run(self):
        while True:
            command_id, command = gRCS.get_current_hl_command()
            if command_id == 0:
                return 'ExecuteManipulationPlan'
            elif command_id == 1:
                rospy.loginfo("Received high level command {}: {}".format(
                    command_id, command))
                return 'ExecuteInspectionPlan'
            elif command_id == 2:
                return 'Failure'
            rospy.sleep(1.0)
            rospy.loginfo_throttle(
                3.0, "In [IDLE] state... Waiting from gRCS commands.")


class HomePose(StateRosControl):
    """
    Go to some home pose
    """

    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)
        path_topic_name = self.get_scoped_param("path_topic_name")

        self.path_publisher = rospy.Publisher(
            path_topic_name, Path, queue_size=1)

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        # !!! The following poses are all referenced to the arm base frame
        # home pose
        home_pose_position = np.array([0.096, 0.266, 0.559])
        home_pose_orientation = np.array([0.011, 0.751, 0.660, 0.010])

        home_pose = numpy_to_pose_stamped(
            home_pose_position, home_pose_orientation, frame_id=Frames.base_frame)

        target_pose = PoseStamped()
        target_pose.header.frame_id = Frames.base_frame
        target_pose.pose = se3_to_pose_ros(self.get_transform(target=Frames.base_frame,
                                                         source=Frames.tool_frame))

        path = get_timed_path_to_target(start_pose=home_pose,
                                        target_pose=target_pose,
                                        linear_velocity=0.25, angular_velocity=0.25)
        self.path_publisher.publish(path)
        if not self.wait_until_reached(target_frame=Frames.tool_frame,
                                       quiet=True):
            return 'Failure'
        else:
            return 'Completed'


class WaypointNavigationState(SingleNavGoalState):
    """
    Depends on a service to provide a goal for the base
    """

    def __init__(self, ns):
        SingleNavGoalState.__init__(self, ns=ns, outcomes=['Completed',
                                                           'Failure',
                                                           'NextWaypoint'])

    def run(self):
        waypoint = gRCS.next_waypoint()

        if waypoint is None:
            return 'Completed'

        goal = PoseStamped()
        goal.header.frame_id = Frames.odom_frame
        goal.header.stamp = rospy.get_rostime()
        goal.pose.position.x = waypoint.x
        goal.pose.position.y = waypoint.y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = np.sin(waypoint.orientation/2.0)
        goal.pose.orientation.w = np.cos(waypoint.orientation/2.0)

        rospy.loginfo("Reaching goal at {}, {}".format(
            goal.pose.position.x, goal.pose.position.y))
        success = self.reach_goal(goal, action=True)
        if not success:
            rospy.logerr("Failed to reach base goal.")
            return 'Failure'

        gRCS.set_waypoint_done()
        return 'NextWaypoint'


class NavigationState(SingleNavGoalState):
    """
    Depends on a service to provide a goal for the base
    """

    def __init__(self, ns):
        SingleNavGoalState.__init__(self, ns=ns)
        self.target_frame = self.get_scoped_param("target_frame")

    def run(self):
        T_map_target = self.get_transform(
            target=Frames.map_frame, source=self.target_frame)
        if T_map_target is None:
            return 'Failure'

        goal = PoseStamped()
        goal.header.frame_id = Frames.map_frame
        goal.pose = se3_to_pose_ros(T_map_target)

        rospy.loginfo("Reaching goal at {}, {}".format(
            goal.pose.position.x, goal.pose.position.y))
        success = self.reach_goal(goal)
        if not success:
            rospy.logerr("Failed to reach base goal.")
            return 'Failure'

        return 'Completed'


class DetectionPosesVisitor(StateRosControl):
    """
    Go to some home pose
    """

    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)
        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(
            path_topic_name, Path, queue_size=1)

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        poses = Valve.get_detection_poses()
        rospy.loginfo("Moving to {} different viewpoints".format(len(poses)))
        for pose in poses:
            start_pose = PoseStamped()
            start_pose.header.frame_id = Frames.base_frame
            start_pose.pose = se3_to_pose_ros(self.get_transform(target=Frames.base_frame,
                                                                 source=Frames.tool_frame))
            path = get_timed_path_to_target(start_pose=start_pose,
                                            target_pose=pose,
                                            linear_velocity=0.25, angular_velocity=0.25)
            rospy.loginfo("Moving to the next viewpoint")
            self.path_publisher.publish(path)
            if not self.wait_until_reached(target_frame=Frames.tool_frame, target_pose=pose, quiet=True):
                return 'Failure'
            else:
                rospy.loginfo("Viewpoint reached.")

            rospy.loginfo("Sleeping 3.0 sec before moving to next viewpoint.")
            rospy.sleep(3.0)
        return 'Completed'


class ModelFitValveState(StateRos):
    """
    Call a detection service to fit a valve model
    """

    def __init__(self, ns):
        StateRos.__init__(self, ns=ns, outcomes=[
                          "Completed", "NextDetection", "Failure"])
        self.object_pose_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.perception_srv_client = rospy.ServiceProxy(self.get_scoped_param(
            "detection_topic", "object_keypoints_ros/perceive"), KeypointsPerception)

        # Way to specify dummy valve, to run without the detection call
        self.dummy = self.get_scoped_param("dummy")
        self.dummy_position = self.get_scoped_param("dummy_position")
        self.dummy_orientation = self.get_scoped_param("dummy_orientation")

        self.k = 3 # TODO move to params or constants
        self.min_successful_detections = self.get_scoped_param("min_successful_detections")
        self.acceptance_ratio = self.get_scoped_param("feature_matcher_acceptance_ratio")
        self.min_consensus = self.get_scoped_param("ransac_min_consensus")
        if self.min_consensus > self.min_successful_detections:
            raise NameError(f"min consensor < min successful detections {self.min_consensus} < {self.min_successful_detections}")

        self.frame_id = None
        self.successful_detections = 0
        self.valve_fitter = ValveFitter(k=self.k)
        self.ransac_matcher = RansacMatcher(acceptance_ratio=self.acceptance_ratio, min_consensus=self.min_consensus)
        self.marker_publisher = rospy.Publisher("/detected_valve/marker", Marker, queue_size=1)

    def _object_name(self) -> str:
        return Frames.valve_frame

    @staticmethod
    def reject_outliers(data, m = 2.):
        """ Returns the filtered array and the index of the corresponding entries """
        d = np.abs(data - np.median(data))
        mdev = np.median(d)
        s = d/mdev if mdev else 0.
        return data[s<m], s<m
    
    def _make_default_marker(self) -> Marker:
        marker = Marker()
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        return marker

    def _model_fit(self, points3d: np.ndarray, frame: str) -> TransformStamped:

        valve: ValveModel
        valve = self.valve_fitter.estimate_from_3d_points(points3d, frame)
        if valve is None:
            return None
        self.set_context('valve_model', valve)

        marker = self._make_default_marker()
        marker.header.frame_id = frame
        marker.scale.x = 2 * valve.radius
        marker.scale.y = 2 * valve.radius
        marker.scale.z = 0.03

        rot = np.zeros((3, 3))
        rot[:, 0] = valve.axis_1
        rot[:, 1] = valve.axis_2
        rot[:, 2] = valve.normal
        q = Rotation.from_matrix(rot).as_quat()

        center = valve.wheel_center
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]

        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        self.marker_publisher.publish(marker)

        object_pose = TransformStamped()
        object_pose.transform.translation = marker.pose.position
        object_pose.transform.rotation = marker.pose.orientation
        return object_pose

    def _object_name(self) -> str:
        return 'object'

    def _request_keypoints(self):
        try:
            self.perception_srv_client.wait_for_service(timeout=10)
        except rospy.ROSException as exc:
            rospy.logwarn("Service {} not available yet".format(self.perception_srv_client.resolved_name))
            return False
        
        req = KeypointsPerceptionRequest()
        res = self.perception_srv_client.call(req)
        self.frame_id = res.keypoints.header.frame_id
        camera = Camera()
        camera.set_intrinsics_from_camera_info(res.camera_info)
        camera.set_extrinsics_from_pose(res.camera_pose)
        keypoints2d = np.zeros((len(res.keypoints2d), 2))
        keypoints3d = np.zeros((len(res.keypoints.poses), 3))
        for i, (kpt2d, kpt3d) in enumerate(zip(res.keypoints2d, res.keypoints.poses)):
            keypoints2d[i, 0] = kpt2d.x
            keypoints2d[i, 1] = kpt2d.y
            keypoints3d[i, 0] = kpt3d.position.x
            keypoints3d[i, 1] = kpt3d.position.y
            keypoints3d[i, 2] = kpt3d.position.z
        self.ransac_matcher.add_observation(camera, keypoints2d, keypoints3d)
        return True

    @staticmethod
    def _filter_3d_observations(points, method='average'):
        """
        # TODO more methods, probably this can be outsourced to the ValveFitter
        Filter a list of [n x 3] representing multiple observations of 
        the same 3d points. Supported methods are:
        - average: just compute the average over all observations 
        """
        if method == 'average':
            n_points = len(points)
            points = np.sum(points, axis=0) / n_points
            return points
        else:   
            raise NameError(f"Unrecognized method {method}")

    def run(self):
        object_pose = TransformStamped()
        if self.dummy:
            rospy.logwarn("[ModelFitState]: running dummy detection")
            object_pose.header.frame_id = "world"
            object_pose.header.stamp = rospy.get_rostime()
            object_pose.child_frame_id = self._object_name()
            object_pose.transform.translation.x = self.dummy_position[0]
            object_pose.transform.translation.y = self.dummy_position[1]
            object_pose.transform.translation.z = self.dummy_position[2]
            object_pose.transform.rotation.x = self.dummy_orientation[0]
            object_pose.transform.rotation.y = self.dummy_orientation[1]
            object_pose.transform.rotation.z = self.dummy_orientation[2]
            object_pose.transform.rotation.w = self.dummy_orientation[3]
        else:
            if self._request_keypoints():
                self.successful_detections += 1
            else:
                rospy.logwarn("Failed to detect keypoints in the image")
                rospy.logwarn(f"Current number of successful detections:  {self.successful_detections}")

            if self.successful_detections < self.min_successful_detections:
                return 'NextDetection'
            
            # keyoints3d is a list [ (num_kpts x 3), (num_kpts x 3), None, ... ] num_observations long
            # and None if the observation was rejected
            success, keypoints2d, keypoints3d = self.ransac_matcher.filter()
            if not success:
                rospy.logerr("Failed to match and filter detections.")
                return 'Failure'

            # filter only valid and matched observations
            valid_observations = [ kpts3d for kpts3d in keypoints3d if kpts3d is not None]
            points = self._filter_3d_observations(valid_observations, method='average')

            object_pose = self._model_fit(points.T, self.frame_id)
            if object_pose is None:
                return 'Failure'
            object_pose.header.frame_id = self.frame_id
            object_pose.header.stamp = rospy.get_rostime()
            object_pose.child_frame_id = self._object_name()

        self.object_pose_broadcaster.sendTransform(object_pose)
        rospy.sleep(2.0)
        return 'Completed'

class ValveManipulationUrdfState(StateRos):
    def __init__(self, ns):
        StateRos.__init__(self, ns=ns)

        self.turning_angle = np.deg2rad(
            self.get_scoped_param("turning_angle_deg", 45.0))

        valve_description_name = self.get_scoped_param(
            "valve_description_name", "valve_description")

        poses_topic = self.get_scoped_param("poses_topic", "/valve_poses")
        self.poses_publisher = rospy.Publisher(poses_topic, PoseArray, queue_size=1, latch=True)

        self.valve_urdf_planner = ValveUrdfPlanner(robot=Robot(valve_description_name),
                                                   world_frame=self.get_scoped_param("world_frame", "world"),
                                                   grasp_frame=self.get_scoped_param("grasp_frame", "grasp_point"),
                                                   grasp_orientation=self.get_scoped_param("grasp_orientation",
                                                                                           [0.0, 180.0, -90.0]))

    def run(self):
        poses = self.generate_valve_turning_poses(self.turning_angle)
        self.poses_publisher.publish(poses)

        return 'Completed'


class ValveManipulationModelState(StateRos):
    def __init__(self, ns):
        StateRos.__init__(self, ns=ns)

        self.turning_angle = np.deg2rad(
            self.get_scoped_param("turning_angle_deg", 45.0))

        poses_topic = self.get_scoped_param("poses_topic", "/valve_poses")
        self.poses_publisher = rospy.Publisher(poses_topic, PoseArray, queue_size=1, latch=True)

    def run(self):
        valve_model = self.global_context.ctx.valve_model
        valve_planner = ValveModelPlanner(valve_model)
        path = valve_planner.get_path(angle_max=self.turning_angle)
        if path is None:
            rospy.logerr("Could not obtain a valid valve manipulation path")
            return 'Failure'

        self.poses_publisher.publish(valve_planner.poses_to_ros(path['poses']))

        return 'Completed'


class LateralGraspState(StateRosControl):
    """
    Switch and send target pose to the controller
    """

    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)
        path_topic_name = self.get_scoped_param("path_topic_name")

        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)
        self.candidate_poses_publisher = rospy.Publisher("/candidate_poses", Path, queue_size=1)
        self.grasp_planner = GraspPlanner()

        self.first_run = True  # select the candidate grasp only at the first run
        self.pre_grasp = None
        self.grasp = None

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        if self.first_run:
            # For debugging only
            candidates = self.grasp_planner.compute_candidate_lateral_grasps()
            self.candidate_poses_publisher.publish(candidates)

            # Goal 0: get close to the grasping pose, not yet around the valve
            #         this preliminary pose is meant to avoid collisions
            approach_pose = self.grasp_planner.compute_lateral_approach_pose()
            start_pose = PoseStamped()
            start_pose.header.frame_id = Frames.base_frame
            start_pose.pose = se3_to_pose_ros(self.get_transform(target=Frames.base_frame, source=Frames.tool_frame))
            path = get_timed_path_to_target(start_pose=start_pose,
                                            target_pose=approach_pose,
                                            linear_velocity=0.25, angular_velocity=0.25)
            self.path_publisher.publish(path)
            if not self.wait_until_reached(target_frame=Frames.tool_frame, target_pose=approach_pose, quiet=True):
                return 'Failure'

            # Goal 1: move tool to the valve plane, not yet at the handle
            self.pre_grasp = self.grasp_planner.compute_lateral_pre_grasp_pose()

            # Goal 2: move tool forward to grasp the handle
            self.grasp = self.grasp_planner.compute_lateral_grasp_pose()

            # Next run the same positions will be used
            self.first_run = False

        start_pose = PoseStamped()
        start_pose.header.frame_id = Frames.base_frame

        start_pose.pose = se3_to_pose_ros(self.get_transform(target=Frames.base_frame, source=Frames.tool_frame))
        path = get_timed_path_to_target(start_pose=start_pose,
                                        target_pose=self.pre_grasp,
                                        linear_velocity=0.5, angular_velocity=0.5)
        self.path_publisher.publish(path)
        if not self.wait_until_reached(Frames.tool_frame, self.pre_grasp, quiet=True):
            return 'Failure'

        # Goal 2: move forward to surround the valve
        start_pose.pose = se3_to_pose_ros(self.get_transform(target=Frames.base_frame, source=Frames.tool_frame))
        path = get_timed_path_to_target(start_pose=start_pose,
                                        target_pose=self.grasp,
                                        linear_velocity=0.1, angular_velocity=0.1)
        self.path_publisher.publish(path)
        if not self.wait_until_reached(Frames.tool_frame, self.grasp, quiet=True):
            return 'Failure'

        return 'Completed'


class ValveManipulation(StateRosControl):
    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)

        self.angle_start_deg = 0.0
        self.angle_step_deg = self.get_scoped_param("angle_step_deg")
        self.angle_end_deg = self.get_scoped_param("angle_end_deg")
        self.angle_delta_deg = self.get_scoped_param("angle_delta_deg")
        self.total_angle = 0
        self.speed_deg = self.get_scoped_param("speed_deg")

        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)

        self.trajectory_generator = ValveTrajectoryGenerator()
        self.theta_current = 0.0
        self.set_context('full_rotation_done', False)

    def step(self):
        path = self.trajectory_generator.get_path(angle_start_deg=0.0,
                                                  angle_end_deg=self.angle_step_deg,
                                                  speed_deg=self.speed_deg,
                                                  angle_delta_deg=self.angle_delta_deg)

        self.path_publisher.publish(path)
        if not self.wait_until_reached(Frames.tool_frame, path.poses[-1], quiet=True, linear_tolerance=0.02):
            return 'Failure'

        else:
            rospy.loginfo("Total angle is: {} (target angle={})".format(self.total_angle, self.angle_end_deg))
            self.total_angle += self.angle_step_deg  # compute the absolute total angle displacement

            if abs(self.total_angle) > abs(self.angle_end_deg):
                self.set_context("full_rotation_done", True, overwrite=True)
                rospy.loginfo("Valve has been successfully operated.")
            return 'Completed'


class LateralManipulation(ValveManipulation):
    """
    Switch and send target poses to the controller manager
    """

    def __init__(self, ns):
        ValveManipulation.__init__(self, ns=ns)

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        self.trajectory_generator.estimate_valve_from_lateral_grasp()
        return self.step()


class PostLateralGraspState(StateRosControl):
    """
    Move away from the valve to restart the grasping in the same pose
    """

    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns,
                                    outcomes=['Completed', 'Failure', 'FullRotationDone'])
        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)
        self.grasp_planner = GraspPlanner()

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        # Goal 1: move away from the valve in the radial direction
        # Assumption is that we are in a grasp state
        target_pose = self.grasp_planner.compute_post_lateral_grasp()
        start_pose = PoseStamped()
        start_pose.header.frame_id = Frames.base_frame
        start_pose.pose = se3_to_pose_ros(self.get_transform(target=Frames.base_frame, source=Frames.tool_frame))
        path = get_timed_path_to_target(start_pose=start_pose,
                                        target_pose=target_pose,
                                        linear_velocity=0.1, angular_velocity=0.1)
        self.path_publisher.publish(path)
        if not self.wait_until_reached(Frames.tool_frame, 
                                       target_pose, 
                                       linear_tolerance=0.02, 
                                       angular_tolerance=0.2,
                                       quiet=True):
            return 'Failure'

        if self.global_context.ctx.full_rotation_done:
            return 'FullRotationDone'

        return 'Completed'

# Unsupported for now
# class FrontalGraspState(RosControlPoseReaching):
#     """
#     Switch and send target pose to the controller
#     """
#
#     def __init__(self, ns):
#         RosControlPoseReaching.__init__(self, ns=ns)
#         pose_topic_name = self.get_scoped_param("pose_topic_name")
#         self.pose_goal_publisher = rospy.Publisher(pose_topic_name, PoseStamped, queue_size=1)
#
#     def execute(self, ud):
#         controller_switched = self.do_switch()
#         if not controller_switched:
#             return 'Failure'
#
#         # Goal 1: get close to the grasping pose, not yet around the valve
#         target_pose = compute_pre_frontal_grasp()
#         self.pose_goal_publisher.publish(target_pose)
#         if not wait_until_reached(target_pose):
#             return 'Failure'
#
#         # Goal 2: move forward to surround the valve
#         target_pose = compute_frontal_grasp()
#         self.pose_goal_publisher.publish(target_pose)
#         if not wait_until_reached(target_pose):
#             return 'Failure'
#
#         return 'Completed'
#
# class FrontalManipulation(ValveManipulation):
#     """
#     Switch and send target poses to the controller manager
#     """
#
#     def __init__(self, ns):
#         ValveManipulation.__init__(self, ns=ns)
#
#     def execute(self, ud):
#         if self.default_outcome:
#             return self.default_outcome
#
#         controller_switched = self.do_switch()
#         if not controller_switched:
#             return 'Failure'
#
#         self.trajectory_generator.estimate_valve_from_frontal_grasp()
#         return self.run()
#
# class PostFrontalGraspState(RosControlPoseReaching):
#     """
#     Move away from the valve to restart the grasping in the same pose
#     """
#
#     def __init__(self, ns):
#         RosControlPoseReaching.__init__(self, ns=ns,
#                                         outcomes=['Completed', 'Failure', 'FullRotationDone'])
#         pose_topic_name = self.get_scoped_param("pose_topic_name")
#         self.pose_goal_publisher = rospy.Publisher(pose_topic_name, PoseStamped, queue_size=1)
#
#     def execute(self, ud):
#         controller_switched = self.do_switch()
#         if not controller_switched:
#             return 'Failure'
#
#         # Goal 1: move away from the valve in the radial direction
#         # Assumption is that we are in a grasp state
#         target_pose = compute_post_frontal_grasp()
#         self.pose_goal_publisher.publish(target_pose)
#         if not wait_until_reached(target_pose):
#             return 'Failure'
#
#         if self.get_context_data('full_rotation_done'):
#             return 'FullRotationDone'
#
#         return 'Completed'
