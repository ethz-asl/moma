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
from scipy.spatial.transform import Rotation as R
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Float64


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
        rospy.Subscriber("moma_ui/commander/target_waypoints", PoseStamped, self.target_waypoints_cb)
        rospy.Subscriber("moma_ui/commander/ee_offset_t_x", Float32, self.ee_offset_t_x_cb)
        rospy.Subscriber("moma_ui/commander/ee_offset_t_y", Float32, self.ee_offset_t_y_cb)
        rospy.Subscriber("moma_ui/commander/ee_offset_t_z", Float32, self.ee_offset_t_z_cb)
        
        self.ee_offset_t_x = 0
        self.ee_offset_t_y = 0
        self.ee_offset_t_z = 0

        # TF listener
        self.tf_listener = tf.TransformListener()

        self.waypoints_path = None

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
        rospy.Service("moma_ui/commander/delete_waypoints", Trigger, self.delete_waypoints_srv)
        rospy.Service("moma_ui/commander/execute_waypoints", Trigger, self.execute_waypoints_srv)

        # path publisher
        self.path_pub = rospy.Publisher("moma_ui/commander/path", Path, queue_size=1)

        # label_callback("default")
        self.label_callback(String("init_pose"))
        self.store_current_pose_srv(TriggerRequest())
        self.label_callback(String("init_joint_state"))
        self.store_current_joint_state_srv(TriggerRequest())
        rospy.loginfo("MoveIt client node initialized.")
        
    def ee_offset_t_x_cb(self, msg):    
        self.ee_offset_t_x = msg.data

    def ee_offset_t_y_cb(self, msg):
        self.ee_offset_t_y = msg.data
    
    def ee_offset_t_z_cb(self, msg):
        self.ee_offset_t_z = msg.data

    def toggle_cmd_input_srv(self, req):
        if req.data:
            rospy.loginfo("Topic input set to pose.")
            self.topic_input = "pose"
        else:
            rospy.loginfo("Topic input set to joint state.")
            self.topic_input = "joint_state"
        return SetBoolResponse(success=True, message="Topic input set to: " + self.topic_input)
    
    def execute_plan_srv(self, req):
        # replace path with a dummy path
        # self.last_target_path = Path()
        # self.last_target_path.header.stamp = rospy.Time.now()
        # self.last_target_path.header.frame_id = self.frame_id
        # current_pose = PoseStamped()
        # current_pose.pose = self.arm_group.get_current_pose().pose
        # current_pose.header.stamp = rospy.Time.now()
        # current_pose.header.frame_id = self.frame_id
        # print('type(current_pose)', type(current_pose)) 
        # self.last_target_path.poses = [current_pose]
        # # second one is just 0.1m in front of the first one
        # second_pose = copy.deepcopy(current_pose)
        # second_pose.pose.position.x += 0.1
        # self.last_target_path.poses.append(second_pose)
        if self.last_target_path is None and self.waypoints_path is None:
            rospy.logwarn("No path to execute.")
            return TriggerResponse(success=True, message="No path to execute.")
        elif self.executing_path:
            rospy.logwarn("Already executing path.")
            return TriggerResponse(success=True, message="Already executing path")
        else:
            self.executing_path = True
            rospy.loginfo("Executing path plan.")
            if self.last_target_path is not None:
                rospy.loginfo("Using last target path.")
                path = self.last_target_path
            elif self.waypoints_path is not None:
                rospy.loginfo("Using waypoints path.")
                path = self.waypoints_path
            else:
                rospy.logwarn("No path to execute.")
                return TriggerResponse(success=True, message="No path to execute")
            # path = self.last_target_path
            # from current to first point in path
            waypoints = []
            current_pose = self.arm_group.get_current_pose().pose
            first_pose = path.poses[0]
            
            if first_pose.header.frame_id != self.frame_id:
                try:
                    rospy.loginfo("Transforming first pose to planning frame.")
                    first_pose = self.tf_listener.transformPose(self.frame_id, first_pose)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logerr("Failed to transform pose to planning frame.")
                    return False
           
            # go through each point in the path, convert to planning frame, and add to waypoints
            idx = 0
            for pose in path.poses:
                print('HELLLOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO')
                if pose.header.frame_id != self.frame_id:
                    # convert the first point in the path to the planning frame
                    pose.header.stamp.nsecs = 0
                    pose.header.stamp.secs = 0
                    try:
                        pose = self.tf_listener.transformPose(self.frame_id, pose)
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rospy.logerr("Failed to transform pose to planning frame.")
                        return False

                # compute orientation based on the difference between current and next point in path
                if idx < len(path.poses) - 1:                
                    pos_current = path.poses[idx].pose.position
                    pos_next = path.poses[idx+1].pose.position
                elif idx == len(path.poses) - 1:
                    pos_current = path.poses[idx-1].pose.position
                    pos_next = path.poses[idx].pose.position
                '''
                # compute yaw
                dy = pos_next.y - pos_current.y
                dx = pos_next.x - pos_current.x
                yaw = np.arctan2(dy, dx)
                yaw_deg = -np.degrees(yaw)
                # wrap to +/- 90
                if yaw_deg > 90:
                    yaw_deg = yaw_deg - 180
                elif yaw_deg < -90:
                    yaw_deg = yaw_deg + 180
                print('pos_current.x', pos_current.x)
                print('pos_current.y', pos_current.y)
                print('pos_next.x', pos_next.x)
                print('pos_next.y', pos_next.y)

                print('dx', dx)
                print('dy', dy)
                print('yaw', yaw)
                print('yaw_deg', yaw_deg)
                
                rospy.loginfo(f"Yaw: {yaw_deg}")    

                # fix orientation
                roll_deg = 180
                pitch_deg = 0
                # yaw_deg = 45 # 
                r = R.from_euler('xyz', [roll_deg, pitch_deg, yaw_deg], degrees=True)
                rq  = r.as_quat()
                '''
                
                nx_sweep_dir = np.array([pos_next.x - pos_current.x, pos_next.y - pos_current.y])
                print('nx_sweep_dir', nx_sweep_dir)

                if nx_sweep_dir[1] < 0:
                    print('Need to flip')
                    nx_sweep_dir = -nx_sweep_dir
                print('After flip nx_sweep_dir', nx_sweep_dir)
                nx_sweep_dir = nx_sweep_dir / np.linalg.norm(nx_sweep_dir)
                ny_sweep_dir = np.array([nx_sweep_dir[1], -nx_sweep_dir[0]])

                print('nx_sweep_dir', nx_sweep_dir)
                print('ny_sweep_dir', ny_sweep_dir)
                print('norm nx_sweep_dir', np.linalg.norm(nx_sweep_dir))
                print('norm ny_sweep_dir', np.linalg.norm(ny_sweep_dir))
                print('dot', np.dot(nx_sweep_dir, ny_sweep_dir))

                # 3x3 rotation matrix
                rotmat = np.array([
                    [nx_sweep_dir[0], ny_sweep_dir[0], 0],
                    [nx_sweep_dir[1], ny_sweep_dir[1], 0],
                    [0, 0, -1]])
                
                #is it a rotation matrix?
                dete = np.linalg.det(rotmat)
                print('dete', dete)
                
                print('rotmat', rotmat)
                print('shape', rotmat.shape)
                # convert to quaternion
                rrr = R.from_matrix(rotmat)
                # offset by 90 degrees
                rrr = rrr * R.from_euler('xyz', [0, 0, 90], degrees=True)
                print('rrr', rrr)
                rq = rrr.as_quat()
                print('rq', rq)

                new_pose = copy.deepcopy(current_pose)
                new_pose.position = pose.pose.position
                new_pose.orientation.x = rq[0]
                new_pose.orientation.y = rq[1]
                new_pose.orientation.z = rq[2]
                new_pose.orientation.w = rq[3]

                # apply offset (todo: do this in the EE frame)
                new_pose.position.x += self.ee_offset_t_x
                new_pose.position.y += self.ee_offset_t_y
                new_pose.position.z += self.ee_offset_t_z

                waypoints.append(new_pose)


            # publish waypoints as path for visualization
            path = Path()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = self.frame_id
            path.poses = []
            for wp in waypoints:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = self.frame_id
                pose.pose = wp
                path.poses.append(pose)

            self.path_pub.publish(path)

            # decorate with first and last pose that are the same height as the current pose
            first_pose = copy.deepcopy(waypoints[0])
            first_pose.position.z = current_pose.position.z + 0.1
            waypoints.insert(0, first_pose)
            last_pose = copy.deepcopy(waypoints[-1])
            last_pose.position.z = current_pose.position.z + 0.1
            waypoints.append(last_pose)


            # go to first point in path but with RRTConnect
            # switch to RRTConnect
            # self.arm_group.set_planner_id("RRTConnect")
            # self.arm_group.set_pose_target(waypoints[0])
            # self.arm_group.go(wait=True)

            # plan 
            try:
                (plan, fraction) = self.arm_group.compute_cartesian_path(
                    waypoints=waypoints,  # waypoints to follow
                    eef_step=0.01,  # eef_step
                    avoid_collisions = False,
                    jump_threshold=1.0)
                # execute
                success = self.arm_group.execute(plan)
                self.executing_path = False
                return TriggerResponse(success=success, message="Executed path plan.")
            except Exception as e:
                rospy.logerr(f"Failed to execute path plan: {e}")
                self.executing_path = False
                return TriggerResponse(success=False, message="Failed to execute path plan.")


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

    def target_waypoints_cb(self, pose):    
        # buffer new waypoint
        rospy.loginfo(f"Received target waypoint")
        if self.waypoints_path is None:
            self.waypoints_path = Path()
            self.waypoints_path.header.frame_id = self.frame_id
        if pose.header.frame_id != self.frame_id:
            rospy.logwarn(f"Pose frame_id: {pose.header.frame_id} does not match planning frame: {self.frame_id}")
            rospy.logwarn("Will try to transform pose to planning frame.")
            print('Frame 1 ', pose.header.frame_id)
            print('Frame 2 ', self.frame_id)
            # wait for transform
            self.tf_listener.waitForTransform(self.frame_id, pose.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
            try:
                pose = self.tf_listener.transformPose(self.frame_id, pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Failed to transform pose to planning frame.")
                return False
        self.waypoints_path.poses.append(pose)
        self.path_pub.publish(self.waypoints_path)

    def delete_waypoints_srv(self, req):
        self.waypoints_path = None
        rospy.loginfo("Deleted waypoints.")
        return TriggerResponse(success=True, message="Deleted waypoints.")
    
    def execute_waypoints_srv(self, req):
        # if self.waypoints_path is None:
        #     rospy.logwarn("No waypoints to execute.")
        #     return TriggerResponse(success=False, message="No waypoints to execute.")
        # elif self.executing_path:
        #     rospy.logwarn("Already executing path.")
        #     return TriggerResponse(success=True, message="Already executing path")
        # else:
        self.last_target_path = self.waypoints_path
        # call execute plan service
        resp = self.execute_plan_srv(TriggerRequest())
        return resp

    # Service to delete all stored labels
    def delete_all_labels_srv(self, req):
        self.stored_poses = {}
        self.stored_joint_states = {}
        rospy.loginfo("Deleted all stored labels.")
        return TriggerResponse(success=True, message="Deleted all stored labels.")

    # Helper function to go to a specified pose
    def go_to_pose(self, pose):
        self.arm_group.set_pose_target(pose)
        success = self.arm_group.go(wait=False)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    # Helper function to go to a specified joint state
    def go_to_joint_state(self, joint_state):
        self.arm_group.set_joint_value_target(joint_state)
        success = self.arm_group.go(wait=False)
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
