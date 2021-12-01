import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path
from moma_mission.utils.transforms import se3_to_pose_ros
from moma_mission.utils.trajectory import get_timed_path_to_target
from moma_mission.states.navigation import SingleNavGoalState

from moma_mission.core import StateRosControl
from moma_mission.utils.moveit import MoveItPlanner
from moma_mission.utils.transforms import numpy_to_pose_stamped

from moma_mission.missions.piloting.frames import Frames
from moma_mission.missions.piloting.valve import Valve
from moma_mission.missions.piloting.grasping import GraspPlanner
from moma_mission.missions.piloting.trajectory import ValveTrajectoryGenerator


class HomePose(StateRosControl):
    """
    Go to some home pose
    """

    def __init__(self, ns):
        try:
            StateRosControl.__init__(self, ns=ns)
            path_topic_name = self.get_scoped_param("path_topic_name")
        except Exception:
            self.initialization_failure = True
            return

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


class NavigationState(SingleNavGoalState):
    """
    Depends on a service to provide a goal for the base
    """

    def __init__(self, ns):
        try:
            SingleNavGoalState.__init__(self, ns=ns)
            self.target_frame = self.get_scoped_param("target_frame")
        except Exception:
            self.initialization_failure = True

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
        try:
            StateRosControl.__init__(self, ns=ns)
            path_topic_name = self.get_scoped_param("path_topic_name")
            self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)
        except Exception:
            self.initialization_failure = True

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


class HomeKinova(StateRosControl):
    """
    Switct to the trajectory controller and homes the robot
    """

    def __init__(self, ns):
        try:
            StateRosControl.__init__(self, ns=ns)
            self.moveit_planner = MoveItPlanner()
        except Exception:
            self.initialization_failure = True

    def run(self):
        controller_switched = self.do_switch()
        if not controller_switched:
            return 'Failure'

        # There might be sporadic motions due to the previously running controller
        # Wait some time after the switching to not trigger a following error at
        # the moveit side
        rospy.loginfo("Sleeping 5.0 sec before homing robot.")
        rospy.sleep(5.0)

        rospy.loginfo("Reaching named position: home")
        success = self.moveit_planner.reach_named_position("home")
        if success:
            return 'Completed'
        else:
            return 'Failure'


class LateralGraspState(StateRosControl):
    """
    Switch and send target pose to the controller
    """

    def __init__(self, ns):
        try:
            StateRosControl.__init__(self, ns=ns)
            path_topic_name = self.get_scoped_param("path_topic_name")

            self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)
            self.candidate_poses_publisher = rospy.Publisher("/candidate_poses", Path, queue_size=1)
            self.grasp_planner = GraspPlanner()
        except Exception:
            self.initialization_failure = True

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
        try:
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
            self.set_context_data('full_rotation_done', False)
        except Exception:
            self.initialization_failure = True

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
                self.set_context_data("full_rotation_done", True, overwrite=True)
                rospy.loginfo("Valve has been successfully operated.")
            return 'Completed'


class LateralManipulation(ValveManipulation):
    """
    Switch and send target poses to the controller manager
    """

    def __init__(self, ns):
        try:
            ValveManipulation.__init__(self, ns=ns)
        except Exception:
            self.initialization_failure = True

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
        try:
            StateRosControl.__init__(self, ns=ns,
                                     outcomes=['Completed', 'Failure', 'FullRotationDone'])
            path_topic_name = self.get_scoped_param("path_topic_name")
            self.path_publisher = rospy.Publisher(path_topic_name, Path, queue_size=1)
            self.grasp_planner = GraspPlanner()
        except Exception:
            self.initialization_failure = True

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

        if self.get_context_data('full_rotation_done'):
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