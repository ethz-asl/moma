#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from nav_msgs.msg import Path
import tf
import copy

class MoveItClient:
    def __init__(self):
        rospy.init_node('moveit_client_node', anonymous=True)

        # Initialize moveit_commander and the move group for the arm
        moveit_commander.roscpp_initialize([])
        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.frame_id = self.arm_group.get_planning_frame()
        self.controlled_frame = self.arm_group.get_end_effector_link()
        self.current_planner = self.arm_group.get_planner_id()
        # self.available_planners = self.arm_group.get_planner_params()
        rospy.loginfo(f"Planning frame: {self.frame_id}")
        rospy.loginfo(f"End effector link: {self.controlled_frame}")
        self.arm_group.set_planner_id("RRTConnect") # options: RRTConnect, P2P, LIN, CIRC, CHOMP
        rospy.loginfo(f"Current planner: {self.arm_group.get_planner_id()}")

        # Initialize storage for poses and joint states
        # self.labels = {}
        self.stored_poses = {}
        self.stored_joint_states = {}
        self.current_label = None

        # Subscribers
        rospy.Subscriber("moma_ui/commander/label", String, self.label_callback)
        rospy.Subscriber("moma_ui/commander/target_pose", PoseStamped, self.target_pose_cb)
        rospy.Subscriber("moma_ui/commander/target_joint_state", JointState, self.target_joint_state_cb)
        rospy.Subscriber("moma_ui/commander/target_path", Path, self.follow_path)

        # TF listener
        self.tf_listener = tf.TransformListener()

        # Services
        rospy.Service("moma_ui/commander/store_pose", Trigger, self.store_current_pose_srv)
        rospy.Service("moma_ui/commander/store_joint_state", Trigger, self.store_current_joint_state_srv)
        rospy.Service("moma_ui/commander/delete_label", Trigger, self.delete_label_srv)
        rospy.Service("moma_ui/commander/delete_all_labels", Trigger, self.delete_all_labels_srv)
        rospy.Service("moma_ui/commander/goto_label", Trigger, self.goto_current_label_srv)
        rospy.Service("moma_ui/commander/print_labels", Trigger, self.print_labels)
        rospy.Service("moma_ui/commander/plan_cartesian_path", Trigger, self.plan_cartesian_path_srv)
        rospy.Service("moma_ui/commander/execute_path", Trigger, self.execute_path_srv)

        # label_callback("default")
        self.label_callback(String("init_pose"))
        self.store_current_pose_srv(TriggerRequest())
        self.label_callback(String("init_joint_state"))
        self.store_current_joint_state_srv(TriggerRequest())
        rospy.loginfo("MoveIt client node initialized.")

    def plan_cartesian_path_srv(self, req):
        # (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01  # waypoints to follow  # eef_step)
        # plan a cartesian path from the current pose to the 0.1 m in front of the current pose and 0.1 m above the current pose
        waypoints = []
        wpose = self.arm_group.get_current_pose().pose
        wpose.position.z += 0.1  # First move up (z)
        # waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += 0.1  # and sideways (y)
        # waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += 0.1  # Second move forward (x)

        # adjust the orientation such that the end effector is perpendicular to the path
        # wpose.orientation.x = 0.0

        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints=waypoints,  # waypoints to follow
            eef_step=0.01,  # eef_step
            avoid_collisions = True)
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        # execute 
        success = self.arm_group.execute(plan)
        return TriggerResponse(success=success, message="Executed cartesian path.")

    def follow_path(self, path):
        rospy.loginfo(f"===================== Received path =====================")
        waypoints = []
        current_pose = self.arm_group.get_current_pose().pose
        waypoints.append(current_pose)
        if True: #path.header.frame_id != self.frame_id:
            # rospy.logwarn(f"Path frame_id: {path.header.frame_id} does not match planning frame: {self.frame_id}")
            # rospy.logwarn("Will try to transform path to planning frame.")
            poses_added = 0
            for pose in path.poses:
                # rospy.loginfo(f"Will try to transform pose: {pose}")
                pose.header.stamp.nsecs = 0
                pose.header.stamp.secs = 0
                try:
                    pose = self.tf_listener.transformPose(self.frame_id, pose)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logerr("Failed to transform pose to planning frame.")
                    return False
                # rospy.loginfo(f"Transformed pose: {pose}")
                new_pose = copy.deepcopy(current_pose)
                new_pose.position = pose.pose.position
                waypoints.append(new_pose)
                break

        # rospy.loginfo(f"Waypoints: {waypoints}")
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints=waypoints,  # waypoints to follow
            eef_step=0.001,  # eef_step
            avoid_collisions = False)
        for i, point in enumerate(plan.joint_trajectory.points):
            if i > 1:
                delta = plan.joint_trajectory.points[i].time_from_start - plan.joint_trajectory.points[i-1].time_from_start
                # convert to seconds
                delta = delta.secs + delta.nsecs * 1e-9
                rospy.loginfo(f"Delta time: {delta}")
                if delta < 0.0:
                    rospy.logwarn(f"Delta time too small, adjusting.")
        # execute
        success = self.arm_group.execute(plan)


    def target_pose_cb(self, pose):
        rospy.loginfo(f"Received target pose: {pose}")
        if pose.header.frame_id != self.frame_id:
            rospy.logwarn(f"Pose frame_id: {pose.header.frame_id} does not match planning frame: {self.frame_id}")
            rospy.logwarn("Will try to transform pose to planning frame.")
            try:
                pose = self.tf_listener.transformPose(self.frame_id, pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Failed to transform pose to planning frame.")
                return False
        self.arm_group.set_pose_target(pose.pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success
    
    def target_joint_state_cb(self, joint_state):
        rospy.loginfo(f"Received target joint state: {joint_state}")
        self.arm_group.set_joint_value_target(joint_state.position)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        return success

    # def follow_path(self, path):

    # Service to delete all stored labels
    def delete_all_labels_srv(self, req):
        self.stored_poses = {}
        self.stored_joint_states = {}
        rospy.loginfo("Deleted all stored labels.")
        return TriggerResponse(success=True, message="Deleted all stored labels.")

    # Helper function to go to a specified pose
    def go_to_pose(self, pose):
        self.arm_group.set_pose_target(pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    # Helper function to go to a specified joint state
    def go_to_joint_state(self, joint_state):
        self.arm_group.set_joint_value_target(joint_state)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        return success

    # Topic callback to update the current label
    def label_callback(self, msg):
        self.current_label = msg.data
        rospy.loginfo(f"Active label set to: {self.current_label}")

    # Service to store current end-effector pose under the current label
    def store_current_pose_srv(self, req):
        if self.current_label is None:
            rospy.logwarn("No label set, cannot store pose.")
            return TriggerResponse(success=False, message="No label set, cannot store pose.")
        current_pose = self.arm_group.get_current_pose().pose
        if self.current_label in self.stored_poses or self.current_label in self.stored_joint_states:
            rospy.logwarn("Pose or joint already exists for this label: " + self.current_label)
            return TriggerResponse(success=False, message="Pose or joint already exists for this label: " + self.current_label)
        else:
            rospy.loginfo("Storing pose as: " + self.current_label)
            self.stored_poses[self.current_label] = current_pose
            return TriggerResponse(success=True, message="Pose stored as: " + self.current_label)

    # Service to store current joint state under the current label
    def store_current_joint_state_srv(self, req):
        if self.current_label is None:
            rospy.logwarn("No label set, cannot store joint state.")
            return TriggerResponse(success=False, message="No label set, cannot store joint state.")
        current_joint_state = self.arm_group.get_current_joint_values()
        if self.current_label in self.stored_poses or self.current_label in self.stored_joint_states:
            rospy.logwarn("Joint or pose state already exists for this label: " + self.current_label)
            return TriggerResponse(success=False, message="Joint or pose state already exists for this label: " + self.current_label)
        else:
            rospy.loginfo("Storing joint state as: " + self.current_label)
            self.stored_joint_states[self.current_label] = current_joint_state
            return TriggerResponse(success=True, message="Joint state stored as: " + self.current_label)

    # Service to delete pose or joint state under the current label
    def delete_label_srv(self, req):
        if not self.current_label:
            rospy.logwarn("No label set, cannot delete state.")
            return TriggerResponse(success=False, message="No label set, cannot delete state.")
        if self.current_label in self.stored_poses or self.current_label in self.stored_joint_states:
            if self.current_label in self.stored_poses:
                self.stored_poses.pop(self.current_label, None)
            if self.current_label in self.stored_joint_states:
                self.stored_joint_states.pop(self.current_label, None)
            rospy.loginfo("Deleted label state: " + self.current_label)
            return TriggerResponse(success=True, message="Deleted label state: " + self.current_label)
        else:
            rospy.logwarn("No stored state to delete.")
            return TriggerResponse(success=False, message="No stored state to delete.")

    # Service to go to the stored pose or joint state at the current label
    def goto_current_label_srv(self, req):
        if not self.current_label:
            rospy.logwarn("No label set.")
            return TriggerResponse(success=False, message="No label set.")
        self.go_to_label(self.current_label)
        return TriggerResponse(success=True, message="Moved to stored state: " + self.current_label)

    def print_labels(self, req):
        rospy.loginfo("Stored labels:")
        rospy.loginfo("Poses:")
        for label, pose in self.stored_poses.items():
            rospy.loginfo(f"  {label}: {pose}")
        rospy.loginfo("Joint states:")
        for label, joint_state in self.stored_joint_states.items():
            rospy.loginfo(f"  {label}: {joint_state}")
        return TriggerResponse(success=True, message="Printed stored labels.")

    # Function to delete a specific label
    def delete_label(self, label):
        if label in self.stored_poses:
            del self.stored_poses[label]
            rospy.loginfo(f"Label '{label}' deleted.")
        elif label in self.stored_joint_states:
            del self.stored_joint_states[label]
            rospy.loginfo(f"Label '{label}' deleted.")
        else:
            rospy.loginfo(f"Label '{label}' does not exist.")

    # Function to go to a specific label's stored state
    def go_to_label(self, label):
        if label not in self.stored_poses and label not in self.stored_joint_states:
            rospy.loginfo(f"Label '{label}' does not have any stored state.")
            return False
        if label in self.stored_poses:
            return self.go_to_pose(self.stored_poses[label])
        elif label in self.stored_joint_states:
            return self.go_to_joint_state(self.stored_joint_states[label])
        return False

if __name__ == "__main__":

    try:
        client = MoveItClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
