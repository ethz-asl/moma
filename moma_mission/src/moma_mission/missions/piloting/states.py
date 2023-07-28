import rospy
import numpy as np
import tf
import tf2_ros
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, PoseArray

from std_msgs.msg import Bool, Float64
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from object_keypoints_ros.srv import KeypointsPerception, KeypointsPerceptionRequest

from moma_mission.utils.transforms import se3_to_pose_ros, tf_to_se3
from moma_mission.utils.trajectory import get_timed_path_to_target
from moma_mission.utils.robot import Robot
from moma_mission.states.navigation import SingleNavGoalState

from moma_mission.core import StateRosControl, StateRos
from moma_mission.utils.transforms import numpy_to_pose_stamped

from moma_mission.missions.piloting.frames import Frames
from moma_mission.missions.piloting.valve import Valve
from moma_mission.missions.piloting.valve_fitting import (
    ValveFitter,
    ValveModel,
    RansacMatcher,
    Camera,
)
from moma_mission.missions.piloting.valve_urdf_planner import ValveUrdfPlanner
from moma_mission.missions.piloting.valve_model_planner import ValveModelPlanner
from moma_mission.missions.piloting.grasping import GraspPlanner
from moma_mission.missions.piloting.trajectory import ValveTrajectoryGenerator
from moma_mission.missions.piloting.rcs_bridge import RCSBridge


class SetUp(StateRos):
    def __init__(self, ns):
        StateRos.__init__(self, ns=ns)

    def run(self):
        self.set_context("gRCS", RCSBridge())
        gRCS = self.global_context.ctx.gRCS
        init_state = gRCS.init_all()
        return "Completed" if init_state else "Failure"


class Idle(StateRos):
    """
    Parses the command from the gRCS and start the execution of the mission.
    Otherwise it idles
    """

    def __init__(self, ns):
        StateRos.__init__(
            self,
            ns=ns,
            outcomes=[
                "ExecuteInspectionPlan",
                "ExecuteDummyPlan",
                "ManipulateValve",
                "ReadGauge",
                "Failure",
            ],
        )
        self.valve_pose_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.gauge_pose_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.map_frame = self.get_scoped_param("map_frame", Frames.map_frame)
        self.valve_frame = self.get_scoped_param("valve_frame", "valve_gt")
        self.gauge_frame = self.get_scoped_param("gauge_frame", "gauge")

    def run(self):
        gRCS = self.global_context.ctx.gRCS
        while True:
            if gRCS.is_waypoints_available:
                if gRCS.is_continuable:
                    return "ExecuteInspectionPlan"
                else:
                    rospy.loginfo(
                        "Waiting for continuation request until reaching next waypoint."
                    )

            command, info = gRCS.get_current_hl_command()
            if command == "PLACE_OBJECT":
                rospy.loginfo("Placing an object...")
            elif command == "READ_GAUGE":
                rospy.loginfo("Reading gauge...")

                # TODO Hack to get some orientation of the gauge from a "reserved" parameter (undocumented)
                yaw_rad = info.param3
                quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_rad)

                gauge_pose = TransformStamped()
                gauge_pose.header.frame_id = self.map_frame
                gauge_pose.header.stamp = rospy.get_rostime()
                gauge_pose.transform.translation.x = info.param5
                gauge_pose.transform.translation.y = info.param6
                gauge_pose.transform.translation.z = info.param7
                gauge_pose.transform.rotation.x = quaternion[0]
                gauge_pose.transform.rotation.y = quaternion[1]
                gauge_pose.transform.rotation.z = quaternion[2]
                gauge_pose.transform.rotation.w = quaternion[3]
                gauge_pose.child_frame_id = self.gauge_frame
                self.gauge_pose_broadcaster.sendTransform(gauge_pose)
                rospy.sleep(2.0)

                return "ReadGauge"
            elif command == "MANIPULATE_VALVE":
                rospy.loginfo("Manipulating valve...")
                desired_angle = info.param1
                if (info.param1 < 0 and info.param2 < 0) or (
                    info.param1 > 0 and info.param2 > 0
                ):
                    desired_angle = -desired_angle
                rospy.loginfo(f"Turning valve by angle {desired_angle} rad.")
                self.set_context("valve_desired_angle", desired_angle)

                # TODO Hack to get some orientation of the valve from a "reserved" parameter (undocumented)
                yaw_rad = info.param3
                quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw_rad)

                valve_pose = TransformStamped()
                valve_pose.header.frame_id = self.map_frame
                valve_pose.header.stamp = rospy.get_rostime()
                valve_pose.transform.translation.x = info.param5
                valve_pose.transform.translation.y = info.param6
                valve_pose.transform.translation.z = info.param7
                valve_pose.transform.rotation.x = quaternion[0]
                valve_pose.transform.rotation.y = quaternion[1]
                valve_pose.transform.rotation.z = quaternion[2]
                valve_pose.transform.rotation.w = quaternion[3]
                valve_pose.child_frame_id = self.valve_frame
                self.valve_pose_broadcaster.sendTransform(valve_pose)
                rospy.sleep(2.0)

                return "ManipulateValve"
            elif command is not None:
                rospy.logerr(f"Command {command} not understood by the state machine.")
            rospy.sleep(1.0)
            rospy.loginfo_throttle(
                3.0, "In [IDLE] state... Waiting for commands from gRCS."
            )


class HomePose(StateRosControl):
    """
    Go to some home pose
    """

    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)
        path_topic_name = self.get_scoped_param("path_topic_name")

        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return "Failure"

        # !!! The following poses are all referenced to the arm base frame
        # home pose
        home_pose_position = np.array([0.096, 0.266, 0.559])
        home_pose_orientation = np.array([0.011, 0.751, 0.660, 0.010])

        home_pose = numpy_to_pose_stamped(
            home_pose_position, home_pose_orientation, frame_id=Frames.base_frame
        )

        target_pose = PoseStamped()
        target_pose.header.frame_id = Frames.base_frame
        target_pose.pose = se3_to_pose_ros(
            self.get_transform(target=Frames.base_frame, source=Frames.tool_frame)
        )

        path = get_timed_path_to_target(
            start_pose=home_pose,
            target_pose=target_pose,
            linear_velocity=0.25,
            angular_velocity=0.25,
        )
        self.path_publisher.publish(path)
        if not self.wait_until_reached(target_frame=Frames.tool_frame, quiet=True):
            return "Failure"
        else:
            return "Completed"


class WaypointNavigationState(SingleNavGoalState):
    """
    Depends on a service to provide a goal for the base
    """

    def __init__(self, ns):
        SingleNavGoalState.__init__(
            self, ns=ns, outcomes=["Completed", "Failure", "NextWaypoint"]
        )

    def run(self):
        waypoint = gRCS.next_waypoint()

        if waypoint is None:
            return "Completed"

        goal = PoseStamped()
        goal.header.frame_id = Frames.odom_frame
        goal.header.stamp = rospy.get_rostime()
        goal.pose.position.x = waypoint.x
        goal.pose.position.y = waypoint.y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = np.sin(waypoint.orientation / 2.0)
        goal.pose.orientation.w = np.cos(waypoint.orientation / 2.0)

        rospy.loginfo(
            "Reaching goal at {}, {}".format(goal.pose.position.x, goal.pose.position.y)
        )
        success = self.reach_goal(goal, action=True)
        if not success:
            rospy.logerr("Failed to reach base goal.")
            return "Failure"

        gRCS.set_waypoint_done()
        return "NextWaypoint"


class NavigationState(SingleNavGoalState):
    """
    Depends on a service to provide a goal for the base
    """

    def __init__(self, ns):
        SingleNavGoalState.__init__(self, ns=ns)
        self.target_frame = self.get_scoped_param("target_frame")

    def run(self):
        T_map_target = self.get_transform(
            target=Frames.map_frame, source=self.target_frame
        )
        if T_map_target is None:
            return "Failure"

        goal = PoseStamped()
        goal.header.frame_id = Frames.map_frame
        goal.pose = se3_to_pose_ros(T_map_target)

        rospy.loginfo(
            "Reaching goal at {}, {}".format(goal.pose.position.x, goal.pose.position.y)
        )
        success = self.reach_goal(goal, action=True)
        if not success:
            rospy.logerr("Failed to reach base goal.")
            return "Failure"

        return "Completed"


class DetectionPosesVisitor(StateRosControl):
    """
    Go to some home pose
    """

    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)
        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return "Failure"

        poses = Valve.get_detection_poses()
        rospy.loginfo("Moving to {} different viewpoints".format(len(poses)))
        for pose in poses:
            start_pose = PoseStamped()
            start_pose.header.frame_id = Frames.base_frame
            start_pose.pose = se3_to_pose_ros(
                self.get_transform(target=Frames.base_frame, source=Frames.tool_frame)
            )
            path = get_timed_path_to_target(
                start_pose=start_pose,
                target_pose=pose,
                linear_velocity=0.25,
                angular_velocity=0.25,
            )
            rospy.loginfo("Moving to the next viewpoint")
            self.path_publisher.publish(path)
            if not self.wait_until_reached(
                target_frame=Frames.tool_frame, target_pose=pose, quiet=True
            ):
                return "Failure"
            else:
                rospy.loginfo("Viewpoint reached.")

            rospy.loginfo("Sleeping 3.0 sec before moving to next viewpoint.")
            rospy.sleep(3.0)
        return "Completed"


class ModelFitValveState(StateRos):
    """
    Call a detection service to fit a valve model
    """

    def __init__(self, ns):
        StateRos.__init__(
            self,
            ns=ns,
            outcomes=["Completed", "NextDetection", "Failure"],
            input_keys=["continue_valve_fitting"],
        )
        self.object_pose_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.perception_srv_client = rospy.ServiceProxy(
            self.get_scoped_param("detection_topic", "object_keypoints_ros/perceive"),
            KeypointsPerception,
        )

        # Way to specify dummy valve, to run without the detection call
        self.dummy = self.get_scoped_param("dummy")
        self.dummy_position = self.get_scoped_param("dummy_position")
        self.dummy_orientation = self.get_scoped_param("dummy_orientation")

        self.num_spokes = self.get_scoped_param("num_spokes")
        self.spoke_radius = self.get_scoped_param("spoke_radius")
        self.handle_radius = self.get_scoped_param("handle_radius")
        self.error_threshold = self.get_scoped_param("error_threshold")
        self.min_successful_detections = self.get_scoped_param(
            "min_successful_detections"
        )
        self.acceptance_ratio = self.get_scoped_param(
            "feature_matcher_acceptance_ratio"
        )
        self.min_consensus = self.get_scoped_param("ransac_min_consensus")
        if self.min_consensus > self.min_successful_detections:
            raise NameError(
                f"min consensor < min successful detections {self.min_consensus} < {self.min_successful_detections}"
            )

        self.frame_id = None
        self.successful_detections = None
        self.detections = None
        self.camera_pose = None
        self.valve_fitter = ValveFitter(num_spokes=self.num_spokes)
        self.ransac_matcher = RansacMatcher(
            acceptance_ratio=self.acceptance_ratio,
            min_consensus=self.min_consensus,
            mask=[0],
        )
        self.matches_publisher = rospy.Publisher(
            "/matched_keypoints", MarkerArray, queue_size=1
        )

    def init(self):
        self.successful_detections = 0
        self.detections = []

    def _object_name(self) -> str:
        return Frames.valve_frame

    @staticmethod
    def reject_outliers(data, m=2.0):
        """Returns the filtered array and the index of the corresponding entries"""
        d = np.abs(data - np.median(data))
        mdev = np.median(d)
        s = d / mdev if mdev else 0.0
        return data[s < m], s < m

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

    @staticmethod
    def _markers_from_keypoints(points, frame) -> MarkerArray:
        colors = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.5, 0.0, 0.5],
            [0.5, 0.5, 0.0],
            [1.0, 0.0, 1.0],
        ]
        markers = MarkerArray()
        for i in range(points.shape[0]):
            marker = Marker()
            marker.header.stamp = rospy.get_rostime()
            marker.header.frame_id = frame
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = colors[i][0]
            marker.color.g = colors[i][1]
            marker.color.b = colors[i][2]
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = points[i][0]
            marker.pose.position.y = points[i][1]
            marker.pose.position.z = points[i][2]
            markers.markers.append(marker)
        return markers

    def _model_fit(self, points3d: np.ndarray) -> TransformStamped:
        valve: ValveModel
        residual, valve = self.valve_fitter.estimate_from_3d_points(
            points_3d=points3d,
            camera_pose=self.camera_pose,
            frame=self.frame_id,
            spoke_radius=self.spoke_radius,
            handle_radius=self.handle_radius,
            error_threshold=self.error_threshold,
        )
        rospy.loginfo(f"Finally fit valve with residual {residual}")
        if valve is None:
            return None
        rospy.loginfo(valve)
        self.set_context("valve_model", valve)

        rot = np.zeros((3, 3))
        rot[:, 0] = valve.axis_1
        rot[:, 1] = valve.axis_2
        rot[:, 2] = valve.normal
        q = Rotation.from_matrix(rot).as_quat()

        object_pose = TransformStamped()
        object_pose.transform.translation.x = valve.wheel_center[0]
        object_pose.transform.translation.y = valve.wheel_center[1]
        object_pose.transform.translation.z = valve.wheel_center[2]
        object_pose.transform.rotation.x = q[0]
        object_pose.transform.rotation.y = q[1]
        object_pose.transform.rotation.z = q[2]
        object_pose.transform.rotation.w = q[3]
        return object_pose

    def _object_name(self) -> str:
        return "object"

    def _request_keypoints(self):
        try:
            self.perception_srv_client.wait_for_service(timeout=10)
        except rospy.ROSException as exc:
            rospy.logwarn(
                "Service {} not available yet".format(
                    self.perception_srv_client.resolved_name
                )
            )
            return False

        req = KeypointsPerceptionRequest()
        try:
            res = self.perception_srv_client.call(req)
        except Exception as e:
            rospy.logerr(e)
            return False

        if len(res.keypoints.poses) != self.num_spokes + 1:
            return False

        self.frame_id = res.keypoints.header.frame_id
        self.camera_pose = res.camera_pose
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

        rospy.loginfo(f"New detection is\n{keypoints3d}")

        if np.isnan(keypoints2d).any() or np.isnan(keypoints3d).any():
            rospy.logerr(
                "Discarding keypoints perception due to NaN points, likely due to missing depth information"
            )
            return False

        self.detections.append(keypoints3d)
        rospy.loginfo(f"{len(self.detections)} successful detections")
        self.ransac_matcher.add_observation(camera, keypoints2d, keypoints3d)
        return True

    def _filter_3d_observations(self, points, method="average"):
        """
        # TODO more methods, probably this can be outsourced to the ValveFitter
        Filter a list of [n x 3] representing multiple observations of
        the same 3d points. Supported methods are:
        - average: just compute the average over all observations
        - geometric: tries to fit a valve to each observation of 3d points, if the residual is too high
          discards observation before final averaging
        """
        if method == "average":
            n_points = len(points)
            points = np.sum(points, axis=0) / n_points
            return points
        elif method == "geometric":
            min_residual = np.inf
            points_new = []
            for p in points:
                rospy.loginfo(f"Fitting to\n{p.T} with shape\n{p.T.shape}")
                residual, valve_fit = self.valve_fitter.estimate_from_3d_points(
                    points_3d=p.T,
                    camera_pose=self.camera_pose,
                    frame=self.frame_id,
                    spoke_radius=self.spoke_radius,
                    handle_radius=self.handle_radius,
                    error_threshold=self.error_threshold,
                )
                rospy.loginfo(f"Residual is {residual}")

                # if valve_fit is not None:
                #     points_new.append(p)
                if valve_fit is not None and residual < min_residual:
                    points_new = [p]
                    min_residual = residual
                else:
                    rospy.loginfo("Observation did not pass geometric validation step.")

            n_points = len(points_new)
            if n_points == 0:
                rospy.logerr("After filtering no points left.")
                return None
            points_new = np.sum(points_new, axis=0) / n_points
            return points_new
        else:
            raise NameError(f"Unrecognized method {method}")

    def run_with_userdata(self, userdata):
        if not userdata.continue_valve_fitting:
            self.init()

        object_pose = TransformStamped()
        if self.dummy:
            rospy.logwarn("[ModelFitValveState]: running dummy detection")
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
                rospy.logwarn(
                    f"Current number of successful detections:  {self.successful_detections}"
                )

            if self.successful_detections < self.min_successful_detections:
                return "NextDetection"

            # trying to match keypoints and filtering based on best epipolar match
            # keyoints3d is a list [ (num_kpts x 3), (num_kpts x 3), None, ... ] num_observations long
            # and None if the observation was rejected
            # try:
            #     (
            #         success,
            #         keypoints2d,
            #         keypoints3d,
            #         cameras,
            #     ) = self.ransac_matcher.filter()
            # except:
            #     rospy.logerr("Exception while matching and filtering detections.")
            #     return "Failure"

            # if not success:
            #     rospy.logerr("Failed to match and filter detections.")
            #     return "Failure"

            # visualize all the matches
            # valid_observations = [
            #     kpts3d for kpts3d in keypoints3d if kpts3d is not None
            # ]
            # for points in self.detections:
            #    rospy.loginfo("Visualizing keypoints matches")
            #    self.matches_publisher.publish(
            #        self._markers_from_keypoints(points, self.frame_id)
            #    )
            #    rospy.sleep(5.0)

            # filter only valid and matched observations
            points = self._filter_3d_observations(self.detections, method="geometric")
            if points is None:
                rospy.logerr("Failed to fit 3d observations, collecting more data.")
                return "NextDetection"

            object_pose = self._model_fit(points.T)
            if object_pose is None:
                return "Failure"
            object_pose.header.frame_id = self.frame_id
            object_pose.header.stamp = rospy.get_rostime()
            object_pose.child_frame_id = self._object_name()

        self.object_pose_broadcaster.sendTransform(object_pose)
        rospy.sleep(2.0)
        return "Completed"


class ValveManipulationUrdfState(StateRos):
    def __init__(self, ns):
        StateRos.__init__(self, ns=ns)

        self.turning_angle = np.deg2rad(
            self.get_scoped_param("turning_angle_deg", 45.0)
        )

        valve_description_name = self.get_scoped_param(
            "valve_description_name", "valve_description"
        )

        poses_topic = self.get_scoped_param("poses_topic", "/valve_poses")
        self.poses_publisher = rospy.Publisher(
            poses_topic, PoseArray, queue_size=1, latch=True
        )

        self.valve_urdf_planner = ValveUrdfPlanner(
            robot=Robot(valve_description_name),
            world_frame=self.get_scoped_param("world_frame", "world"),
            grasp_frame=self.get_scoped_param("grasp_frame", "grasp_point"),
            grasp_orientation=self.get_scoped_param(
                "grasp_orientation", [0.0, 180.0, -90.0]
            ),
        )

    def run(self):
        poses = self.valve_urdf_planner.generate_valve_turning_poses(self.turning_angle)
        self.poses_publisher.publish(poses)

        return "Completed"


class ValveManipulationPrePostState(StateRos):
    def __init__(self, ns):
        StateRos.__init__(self, ns=ns)

        angle_topic = self.get_scoped_param("angle_topic", "/valve_angle")
        self.angle_publisher = rospy.Publisher(
            angle_topic, Float64, queue_size=1, latch=True
        )
        valve_continue_topic = self.get_scoped_param("angle_topic", "/valve_continue")
        self.valve_continue_publisher = rospy.Publisher(
            valve_continue_topic, Bool, queue_size=1, latch=True
        )

    def init(self):
        self.set_context("valve_total_angle", 0)
        self._publish_angle()
        self._publish_continue_valve(False)

    def _publish_angle(self):
        angle = Float64()
        angle.data = self.global_context.ctx.valve_total_angle
        self.angle_publisher.publish(angle)

    def _publish_continue_valve(self, do_continue):
        """
        Whether the valve manipulation is continued on the same valve as before
        """
        valve_continue = Bool()
        valve_continue.data = do_continue
        self.valve_continue_publisher.publish(valve_continue)

    def run(self):
        if not self.get_scoped_param("post"):
            self.init()
            return "Completed"

        valve_total_angle = (
            self.global_context.ctx.valve_total_angle
            + self.global_context.ctx.valve_angle_step
        )
        self.set_context("valve_total_angle", valve_total_angle)
        self._publish_angle()
        self._publish_continue_valve(True)

        return "Completed"


class ValveAngleControllerState(StateRos):
    def run(self):
        if "valve_desired_angle" not in self.global_context.ctx:
            rospy.loginfo(
                "Dynamic valve_desired_angle was not found in global context. Falling back to static one."
            )
            self.set_context(
                "valve_desired_angle",
                np.deg2rad(self.get_scoped_param("turning_angle_deg", 140.0)),
            )
        valve_desired_angle = self.global_context.ctx.valve_desired_angle
        valve_total_angle = self.global_context.ctx.valve_total_angle
        tolerance = np.deg2rad(self.get_scoped_param("turning_angle_tolerance_deg", 10))

        rospy.loginfo(
            f"Valve is at angle {np.rad2deg(valve_total_angle)}° and should be at angle {np.rad2deg(valve_desired_angle)}°. Tolerance is {np.rad2deg(tolerance)}°."
        )

        if (
            abs(valve_desired_angle) - tolerance
            <= abs(valve_total_angle)
            <= abs(valve_desired_angle) + tolerance
        ):
            return "Completed"

        return "Failure"


class ValveManipulationModelState(StateRos):
    def __init__(self, ns):
        StateRos.__init__(self, ns=ns)

        self.turning_angle_step = np.deg2rad(
            self.get_scoped_param("turning_angle_step_deg", 360.0)
        )
        self.safety_distance_factor = self.get_scoped_param(
            "safety_distance_factor", -1
        )

        poses_topic = self.get_scoped_param("poses_topic", "/valve_poses")
        approach_poses_topic = self.get_scoped_param(
            "approach_poses_topic", "/valve_approach_poses"
        )
        self.poses_publisher = rospy.Publisher(
            poses_topic, PoseArray, queue_size=1, latch=True
        )
        self.approach_poses_publisher = rospy.Publisher(
            approach_poses_topic, PoseArray, queue_size=1, latch=True
        )
        self.robot_base_frame = self.get_scoped_param("robot_base_frame", None)

        path_inverted_topic = self.get_scoped_param(
            "path_inverted_topic", "/valve_path_inverted"
        )
        self.path_inverted_publisher = rospy.Publisher(
            path_inverted_topic, Bool, queue_size=1, latch=True
        )

    def run(self):
        valve_model = self.global_context.ctx.valve_model
        robot_base_pose = None
        if self.robot_base_frame is not None:
            robot_base_pose = tf_to_se3(
                self.tf_buffer.lookup_transform(
                    valve_model.frame,
                    self.robot_base_frame,
                    rospy.Time(0),  # tf at first available time
                    rospy.Duration(3),
                )
            )
            rospy.loginfo(f"Robot base pose is {robot_base_pose}")
        valve_planner = ValveModelPlanner(
            valve_model=valve_model,
            robot_base_pose=robot_base_pose,
            safety_distance_factor=self.safety_distance_factor,
        )
        turning_angle = (
            self.global_context.ctx.valve_desired_angle
            - self.global_context.ctx.valve_total_angle
        )
        turning_angle = max(
            min(turning_angle, self.turning_angle_step), -self.turning_angle_step
        )
        rospy.loginfo(
            f"Valve is at angle {np.rad2deg(self.global_context.ctx.valve_total_angle)}° and should be at angle {np.rad2deg(self.global_context.ctx.valve_desired_angle)}°. Step size is {np.rad2deg(self.turning_angle_step)}°. Turning valve by {np.rad2deg(turning_angle)}°."
        )
        path = valve_planner.get_path(angle_max=turning_angle)
        if path is None:
            rospy.logerr("Could not obtain a valid valve manipulation path")
            return "Failure"

        path_inverted = Bool()
        path_inverted.data = path["inverted"]
        self.path_inverted_publisher.publish(path_inverted)
        self.poses_publisher.publish(valve_planner.poses_to_ros(path["poses"]))
        self.approach_poses_publisher.publish(
            valve_planner.poses_to_ros(valve_planner.get_path_approach_poses(path))
        )

        self.set_context("valve_angle_step", path["angle_signed"])

        return "Completed"


class LateralGraspState(StateRosControl):
    """
    Switch and send target pose to the controller
    """

    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)
        path_topic_name = self.get_scoped_param("path_topic_name")

        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)
        self.candidate_poses_publisher = rospy.Publisher(
            "/candidate_poses", Path, queue_size=1
        )
        self.grasp_planner = GraspPlanner()

        self.first_run = True  # select the candidate grasp only at the first run
        self.pre_grasp = None
        self.grasp = None

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return "Failure"

        if self.first_run:
            # For debugging only
            candidates = self.grasp_planner.compute_candidate_lateral_grasps()
            self.candidate_poses_publisher.publish(candidates)

            # Goal 0: get close to the grasping pose, not yet around the valve
            #         this preliminary pose is meant to avoid collisions
            approach_pose = self.grasp_planner.compute_lateral_approach_pose()
            start_pose = PoseStamped()
            start_pose.header.frame_id = Frames.base_frame
            start_pose.pose = se3_to_pose_ros(
                self.get_transform(target=Frames.base_frame, source=Frames.tool_frame)
            )
            path = get_timed_path_to_target(
                start_pose=start_pose,
                target_pose=approach_pose,
                linear_velocity=0.25,
                angular_velocity=0.25,
            )
            self.path_publisher.publish(path)
            if not self.wait_until_reached(
                target_frame=Frames.tool_frame, target_pose=approach_pose, quiet=True
            ):
                return "Failure"

            # Goal 1: move tool to the valve plane, not yet at the handle
            self.pre_grasp = self.grasp_planner.compute_lateral_pre_grasp_pose()

            # Goal 2: move tool forward to grasp the handle
            self.grasp = self.grasp_planner.compute_lateral_grasp_pose()

            # Next run the same positions will be used
            self.first_run = False

        start_pose = PoseStamped()
        start_pose.header.frame_id = Frames.base_frame

        start_pose.pose = se3_to_pose_ros(
            self.get_transform(target=Frames.base_frame, source=Frames.tool_frame)
        )
        path = get_timed_path_to_target(
            start_pose=start_pose,
            target_pose=self.pre_grasp,
            linear_velocity=0.5,
            angular_velocity=0.5,
        )
        self.path_publisher.publish(path)
        if not self.wait_until_reached(Frames.tool_frame, self.pre_grasp, quiet=True):
            return "Failure"

        # Goal 2: move forward to surround the valve
        start_pose.pose = se3_to_pose_ros(
            self.get_transform(target=Frames.base_frame, source=Frames.tool_frame)
        )
        path = get_timed_path_to_target(
            start_pose=start_pose,
            target_pose=self.grasp,
            linear_velocity=0.1,
            angular_velocity=0.1,
        )
        self.path_publisher.publish(path)
        if not self.wait_until_reached(Frames.tool_frame, self.grasp, quiet=True):
            return "Failure"

        return "Completed"


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
        self.set_context("full_rotation_done", False)

    def step(self):
        path = self.trajectory_generator.get_path(
            angle_start_deg=0.0,
            angle_end_deg=self.angle_step_deg,
            speed_deg=self.speed_deg,
            angle_delta_deg=self.angle_delta_deg,
        )

        self.path_publisher.publish(path)
        if not self.wait_until_reached(
            Frames.tool_frame, path.poses[-1], quiet=True, linear_tolerance=0.02
        ):
            return "Failure"

        else:
            rospy.loginfo(
                "Total angle is: {} (target angle={})".format(
                    self.total_angle, self.angle_end_deg
                )
            )
            self.total_angle += (
                self.angle_step_deg
            )  # compute the absolute total angle displacement

            if abs(self.total_angle) > abs(self.angle_end_deg):
                self.set_context("full_rotation_done", True, overwrite=True)
                rospy.loginfo("Valve has been successfully operated.")
            return "Completed"


class LateralManipulation(ValveManipulation):
    """
    Switch and send target poses to the controller manager
    """

    def __init__(self, ns):
        ValveManipulation.__init__(self, ns=ns)

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return "Failure"

        self.trajectory_generator.estimate_valve_from_lateral_grasp()
        return self.step()


class PostLateralGraspState(StateRosControl):
    """
    Move away from the valve to restart the grasping in the same pose
    """

    def __init__(self, ns):
        StateRosControl.__init__(
            self, ns=ns, outcomes=["Completed", "Failure", "FullRotationDone"]
        )
        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)
        self.grasp_planner = GraspPlanner()

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return "Failure"

        # Goal 1: move away from the valve in the radial direction
        # Assumption is that we are in a grasp state
        target_pose = self.grasp_planner.compute_post_lateral_grasp()
        start_pose = PoseStamped()
        start_pose.header.frame_id = Frames.base_frame
        start_pose.pose = se3_to_pose_ros(
            self.get_transform(target=Frames.base_frame, source=Frames.tool_frame)
        )
        path = get_timed_path_to_target(
            start_pose=start_pose,
            target_pose=target_pose,
            linear_velocity=0.1,
            angular_velocity=0.1,
        )
        self.path_publisher.publish(path)
        if not self.wait_until_reached(
            Frames.tool_frame,
            target_pose,
            linear_tolerance=0.02,
            angular_tolerance=0.2,
            quiet=True,
        ):
            return "Failure"

        if self.global_context.ctx.full_rotation_done:
            return "FullRotationDone"

        return "Completed"


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
