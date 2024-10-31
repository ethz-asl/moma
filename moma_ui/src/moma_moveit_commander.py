#!/usr/bin/env python3

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse

class MoveItClient:
    def __init__(self):
        rospy.init_node('moveit_client_node', anonymous=True)

        # Initialize moveit_commander and the move group for the arm
        moveit_commander.roscpp_initialize([])
        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")

        # Initialize storage for poses and joint states
        self.labels = {}
        self.current_label = None

        # Subscribers
        rospy.Subscriber("label", String, self.label_callback)

        # Services
        rospy.Service("store_pose", Trigger, self.store_current_pose)
        rospy.Service("store_joint_state", Trigger, self.store_current_joint_state)
        rospy.Service("delete_current_state", Trigger, self.delete_current_state)
        rospy.Service("goto_label", Trigger, self.goto_current_label)

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
    def store_current_pose(self, req):
        if not self.current_label:
            return TriggerResponse(success=False, message="No label set.")
        current_pose = self.arm_group.get_current_pose().pose
        if self.current_label in self.labels and "pose" in self.labels[self.current_label]:
            return TriggerResponse(success=False, message="Pose already exists for this label.")
        self.labels.setdefault(self.current_label, {})["pose"] = current_pose
        return TriggerResponse(success=True, message="Pose stored.")

    # Service to store current joint state under the current label
    def store_current_joint_state(self, req):
        if not self.current_label:
            return TriggerResponse(success=False, message="No label set.")
        current_joint_state = self.arm_group.get_current_joint_values()
        if self.current_label in self.labels and "joint_state" in self.labels[self.current_label]:
            return TriggerResponse(success=False, message="Joint state already exists for this label.")
        self.labels.setdefault(self.current_label, {})["joint_state"] = current_joint_state
        return TriggerResponse(success=True, message="Joint state stored.")

    # Service to delete pose or joint state under the current label
    def delete_current_state(self, req):
        if not self.current_label:
            return TriggerResponse(success=False, message="No label set.")
        if self.current_label in self.labels:
            self.labels.pop(self.current_label, None)
            return TriggerResponse(success=True, message="Deleted label state.")
        return TriggerResponse(success=False, message="No stored state to delete.")

    # Service to go to the stored pose or joint state at the current label
    def goto_current_label(self, req):
        if not self.current_label:
            return TriggerResponse(success=False, message="No label set.")
        if self.current_label not in self.labels:
            return TriggerResponse(success=False, message="No stored state for this label.")
        if "pose" in self.labels[self.current_label]:
            success = self.go_to_pose(self.labels[self.current_label]["pose"])
            return TriggerResponse(success=success, message="Moved to pose." if success else "Failed to move to pose.")
        elif "joint_state" in self.labels[self.current_label]:
            success = self.go_to_joint_state(self.labels[self.current_label]["joint_state"])
            return TriggerResponse(success=success, message="Moved to joint state." if success else "Failed to move to joint state.")
        return TriggerResponse(success=False, message="No stored pose or joint state for this label.")

    # Function to delete a specific label
    def delete_label(self, label):
        if label in self.labels:
            del self.labels[label]
            rospy.loginfo(f"Label '{label}' deleted.")
        else:
            rospy.loginfo(f"Label '{label}' does not exist.")

    # Function to go to a specific label's stored state
    def go_to_label(self, label):
        if label not in self.labels:
            rospy.loginfo(f"Label '{label}' does not have any stored state.")
            return False
        if "pose" in self.labels[label]:
            return self.go_to_pose(self.labels[label]["pose"])
        elif "joint_state" in self.labels[label]:
            return self.go_to_joint_state(self.labels[label]["joint_state"])
        return False

if __name__ == "__main__":
    try:
        client = MoveItClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
