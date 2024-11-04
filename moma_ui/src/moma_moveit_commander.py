#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest, SetBool, SetBoolRequest, SetBoolResponse
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
        rospy.loginfo(f"Planning frame: {self.frame_id}")
        rospy.loginfo(f"End effector link: {self.controlled_frame}")
        self.arm_group.set_planner_id("RRTConnect") # options: RRTConnect, P2P, LIN, CIRC, CHOMP
        rospy.loginfo(f"Current planner: {self.arm_group.get_planner_id()}")


        self.topic_input = "pose"

        # Initialize storage for poses and joint states
        self.stored_poses = {}
        self.stored_joint_states = {}
        self.current_label = None

        # Subscribers
        rospy.Subscriber("moma_ui/commander/label", String, self.label_callback)
        rospy.Subscriber("moma_ui/commander/target_pose", PoseStamped, self.target_pose_cb)
        rospy.Subscriber("moma_ui/commander/target_joint_state", JointState, self.target_joint_state_cb)
        rospy.Subscriber("moma_ui/commander/target_path", Path, self.target_path_cb)

        # TF listener
        self.tf_listener = tf.TransformListener()

        self.last_target_path = None
        self.executing_path = False

        # Services
        rospy.Service("moma_ui/commander/store_pose", Trigger, self.store_current_pose_srv)
        rospy.Service("moma_ui/commander/store_joint_state", Trigger, self.store_current_joint_state_srv)
        rospy.Service("moma_ui/commander/delete_label", Trigger, self.delete_label_srv)
        rospy.Service("moma_ui/commander/delete_all_labels", Trigger, self.delete_all_labels_srv)
        rospy.Service("moma_ui/commander/goto_label", Trigger, self.goto_current_label_srv)
        rospy.Service("moma_ui/commander/print_labels", Trigger, self.print_labels)
        rospy.Service("moma_ui/commander/execute_path", Trigger, self.execute_plan_srv)
        rospy.Service("moma_ui/commander/toggle_cmd_input", SetBool, self.toggle_cmd_input_srv)

        # label_callback("default")
        self.label_callback(String("init_pose"))
        self.store_current_pose_srv(TriggerRequest())
        self.label_callback(String("init_joint_state"))
        self.store_current_joint_state_srv(TriggerRequest())
        rospy.loginfo("MoveIt client node initialized.")
        

    def toggle_cmd_input_srv(self, req):
        if req.data:
            rospy.loginfo("Topic input set to pose.")
            self.topic_input = "pose"
        else:
            rospy.loginfo("Topic input set to joint state.")
            self.topic_input = "joint_state"
        return SetBoolResponse(success=True, message="Topic input set to: " + self.topic_input)
    
    def execute_plan_srv(self, req):
        if self.last_target_path is None:
            rospy.logwarn("No path to execute.")
            return TriggerResponse(success=True, message="No path to execute.")
        elif self.executing_path:
            rospy.logwarn("Already executing path.")
            return TriggerResponse(success=True, message="Already executing path")
        else:
            self.executing_path = True
            rospy.loginfo("Executing path plan.")
            path = self.last_target_path
            # from current to first point in path
            waypoints = []
            current_pose = self.arm_group.get_current_pose().pose

            # go through each point in the path, convert to planning frame, and add to waypoints
            for pose in path.poses:
                if pose.header.frame_id != self.frame_id:
                    # convert the first point in the path to the planning frame
                    pose.header.stamp.nsecs = 0
                    pose.header.stamp.secs = 0
                    try:
                        pose = self.tf_listener.transformPose(self.frame_id, pose)
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rospy.logerr("Failed to transform pose to planning frame.")
                        return False
                new_pose = copy.deepcopy(current_pose)
                new_pose.position = pose.pose.position
                waypoints.append(new_pose)
            # plan 
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints=waypoints,  # waypoints to follow
                eef_step=0.01,  # eef_step
                avoid_collisions = False)
            # execute
            success = self.arm_group.execute(plan)
            self.executing_path = False
            return TriggerResponse(success=success, message="Executed path plan.")

    def target_path_cb(self, path):
        rospy.loginfo(f"Received target path")
        self.last_target_path = path

    def target_pose_cb(self, pose):
        rospy.loginfo(f"Received target pose")
        if self.topic_input == "pose" and not self.executing_path:
            rospy.loginfo("Will go to target pose.")
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
    
    def target_joint_state_cb(self, joint_state):
        rospy.loginfo(f"Received target joint state")
        if self.topic_input == "joint_state" and not self.executing_path:
            self.arm_group.set_joint_value_target(joint_state.position)
            success = self.arm_group.go(wait=True)
            self.arm_group.stop()

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
