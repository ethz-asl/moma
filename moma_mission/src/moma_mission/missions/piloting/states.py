
import rospy
import numpy as np
from typing import List
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose

from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from moma_mission.utils.transforms import se3_to_pose_ros
from moma_mission.utils.trajectory import get_timed_path_to_target
from moma_mission.states.navigation import SingleNavGoalState
from moma_mission.states.model_fit import ModelFitState

from moma_mission.core import StateRosControl, StateRos
from moma_mission.utils.transforms import numpy_to_pose_stamped

from moma_mission.missions.piloting.frames import Frames
from moma_mission.missions.piloting.valve import Valve
from moma_mission.missions.piloting.valve_fitting import ValveFitter, ValveModel
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
        StateRos.__init__(self, ns=ns, outcomes=['ExecuteInspectionPlan', 'ExecuteManipulationPlan', 'Failure'])
    
    def run(self):
        while True:
            command_id, command = gRCS.get_current_hl_command()
            if command_id == 0:
                return 'ExecuteManipulationPlan'
            elif command_id == 1:
                rospy.loginfo("Received high level command {}: {}".format(command_id, command))
                return 'ExecuteInspectionPlan'
            elif command_id == 2:
                return 'Failure'
            rospy.sleep(1.0)
            rospy.loginfo_throttle(3.0, "In [IDLE] state... Waiting from gRCS commands.")

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
            return 'Failure'

        # !!! The following poses are all referenced to the arm base frame
        # home pose
        home_pose_position = np.array([0.096, 0.266, 0.559])
        home_pose_orientation = np.array([0.011, 0.751, 0.660, 0.010])

        home_pose = numpy_to_pose_stamped(home_pose_position, home_pose_orientation, frame_id=Frames.base_frame)

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

        rospy.loginfo("Reaching goal at {}, {}".format(goal.pose.position.x, goal.pose.position.y))
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
        T_map_target = self.get_transform(target=Frames.map_frame, source=self.target_frame)
        if T_map_target is None:
            return 'Failure'

        goal = PoseStamped()
        goal.header.frame_id = Frames.map_frame
        goal.pose = se3_to_pose_ros(T_map_target)

        rospy.loginfo("Reaching goal at {}, {}".format(goal.pose.position.x, goal.pose.position.y))
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
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)

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


class ModelFitValve(ModelFitState):
    """
    Call a detection service to fit a valve model
    """
    def __init__(self, ns):
        ModelFitState.__init__(self, ns=ns, outcomes=["Completed", "Retry", "Failure"])
        self.k = 3 # TODO remove the hard coded 3 = number of spokes in the valve
        self.valve_fitter = ValveFitter(k=3) 

        self.marker_publisher = rospy.Publisher("/detected_valve/marker", Marker, queue_size=1)

    def _object_name(self) -> str:
        return Frames.valve_frame

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

    def _model_fit(self, keypoints_perception: List[Pose], frame: str) -> TransformStamped:
        points_3d = np.zeros((3, self.k +1)) # spokes plus center
        for i, kpt in enumerate(keypoints_perception):
            points_3d[:, i] = np.array([kpt.position.x, kpt.position.y, kpt.position.z])

        valve: ValveModel
        valve = self.valve_fitter.estimate_from_3d_points(points_3d)
        marker = self._make_default_marker()
        marker.header.frame_id = frame
        marker.scale.x = 2 * valve.radius
        marker.scale.y = 2 * valve.radius
        marker.scale.z = 0.03

        rot = np.zeros((3, 3))
        rot[:, 0] = valve.axis_1
        rot[:, 1] = valve.axis_2
        rot[:, 2] = valve.n
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


class GraspState(StateRosControl):
    def __init__(self, ns):
        StateRosControl.__init__(self, ns=ns)
        path_topic_name = self.get_scoped_param("path_topic_name")
        self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)
        self.offset = self.get_scoped_param("offset")

    def run(self):
        valve_model = self.global_context.ctx.valve_model
        self.trajectory_generator.set_model(valve_model)

        
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
