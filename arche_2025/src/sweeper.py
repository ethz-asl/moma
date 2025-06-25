import moveit_commander
import rospy
import tf


from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path

from visualization_msgs.msg import Marker, MarkerArray

class MoveItSweeperClient:
    def __init__(self):
        rospy.init_node('moveit_client_node', anonymous=False)

        self.workplane_id = "workplane"
        self.ee_roll_offset_deg = 180.0  # offset for the end effector roll, in degrees

        # Initialize moveit_commander and the move group for the arm
        moveit_commander.roscpp_initialize([])
        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")
        rospy.loginfo("MoveIt client initialized.")
        self.controlled_frame = self.arm_group.get_end_effector_link()
        self.arm_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
        self.arm_group.set_planner_id("LIN")
        # print controlled frame
        rospy.loginfo(f"Controlled frame: {self.controlled_frame}")
        # get base frame for planning
        self.base_frame = self.arm_group.get_planning_frame()
        rospy.loginfo(f"Base frame for planning: {self.base_frame}")


        # Publishers
        self.stored_wp_path_pub = rospy.Publisher("/move_base_simple/goal_path", Path, queue_size=10)
        self.viz_marker_pub = rospy.Publisher("/sweeper/visualization_marker", Marker, queue_size=10, latch=True)

        # Subscribers
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_navgoal_callback)

        # Services
        self.go_home_service = rospy.Service('go_home', Empty, self.go_home_service)
        self.follow_navgoal_service = rospy.Service('follow_navgoal', Empty, self.rviz_navgoal_callback)
        self.goto_last_navgoal_service = rospy.Service('goto_last_navgoal', Empty, self.goto_navgoal_srv)
        self.follow_sweep_service = rospy.Service('follow_sweep', Empty, self.follow_sweep_srv)
        # Variables
        self.rviz_navgoal_list = None # this can only hold 1-2 goals at a time, so we can use it to store the last goal
        self.moveit_navgoal_list = None
        # go home
        self.go_home()

        # publish workplane marker as plane
        self.publish_workplane_marker()

    def publish_workplane_marker(self):
        if not hasattr(self, 'viz_marker_pub') or self.viz_marker_pub is None:
            rospy.logwarn("viz_marker_pub is not initialized.")
            return

        marker = Marker()
        marker.header.frame_id = "workplane"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "sweeper_workplane"
        marker.id = 0
        marker.type = Marker.CUBE  # Correct constant name
        marker.action = Marker.ADD

        # Position and orientation
        marker.pose.orientation.w = 1.0  # Identity quaternion

        # Dimensions (meters)
        marker.scale.x = 0.32
        marker.scale.y = 0.32
        marker.scale.z = 0.01

        # Color (semi-transparent green)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Publish the marker
        self.viz_marker_pub.publish(marker)
        rospy.loginfo("Workplane marker published.")

    def go_home(self):
        # ensure "home" is in the named targets
        named_states = self.arm_group.get_named_targets()
        if "home" not in named_states:
            rospy.logerr("Named target 'home' not found in the move group.")
            return False
        
        # going home
        self.arm_group.set_planner_id("PTP")
        self.arm_group.set_named_target("home")
        success = self.arm_group.go(wait=True)
        return success
    
    def go_home_service(self, req):
        success = self.go_home()
        if success:
            rospy.loginfo("Successfully moved to home position.")
        else:
            rospy.logerr("Failed to move to home position.")
        return success        

    def goto_navgoal_srv(self, req):
        if self.moveit_navgoal_list is None or len(self.moveit_navgoal_list) == 0:
            rospy.logerr("No navigation goals received yet.")
            return False
        
        # first, go home
        rospy.loginfo("Going home before navigating to the last goal.")
        success = self.go_home()

        if not success:
            rospy.logerr("Failed to go home before navigating to the last goal.")
            return False
        # then, navigate to the last goal
        last_goal = self.moveit_navgoal_list[-1]
        
        # print current EE
        current_ee_pose = self.arm_group.get_current_pose(self.controlled_frame)
        rospy.loginfo(f"Current end effector pose: {current_ee_pose}")
        rospy.loginfo(f"Last goal pose: {last_goal.pose}")
        
        self.arm_group.set_planner_id("PTP")
        self.arm_group.set_pose_target(last_goal.pose)
        success = self.arm_group.go(wait=True)
        if not success:
            rospy.logerr("Failed to navigate to the last goal.")
            return False
        rospy.loginfo("Successfully navigated to the last goal.")

        return True

    def follow_sweep_srv(self, req):
        # length of the sweep path needs to be >=2
        if self.moveit_navgoal_list is None or len(self.moveit_navgoal_list) < 2:
            rospy.logerr("Not enough navigation goals received to perform a sweep.")
            return False
        # first, go home
        rospy.loginfo("Going home before performing the sweep.")
        success = self.go_home()
        if not success:
            rospy.logerr("Failed to go home before performing the sweep.")
            return False
        # then, perform the sweep
        rospy.loginfo("Performing sweep with the last two navigation goals.")
        # get the last two goals
        sweep_start = self.moveit_navgoal_list[-2]
        sweep_end = self.moveit_navgoal_list[-1]

        # go to the start of the sweep
        self.arm_group.set_planner_id("PTP")
        self.arm_group.set_pose_target(sweep_start.pose)
        success = self.arm_group.go(wait=True)
        if not success:
            rospy.logerr("Failed to navigate to the start of the sweep.")
            return False
        rospy.loginfo("Successfully navigated to the start of the sweep.")
        # perform the sweep to the end
        self.arm_group.set_planner_id("LIN")
        self.arm_group.set_pose_target(sweep_end.pose)
        success = self.arm_group.go(wait=True)
        if not success:
            rospy.logerr("Failed to navigate to the end of the sweep.")
            return False
        rospy.loginfo("Successfully navigated to the end of the sweep.")
        
        # done
        rospy.loginfo("Sweep completed successfully.")
        return True


    def rviz_navgoal_callback(self, msg):
        # ensure msg is in the workplane frame
        if msg.header.frame_id != self.workplane_id:
            rospy.logerr("Received goal is not in the 'workplane' frame.")
            return False
        
        # apply the end effector roll offset
        pose_with_offset = self.apply_rpy_offset(msg, 2*1.57, 0.0, 0.0)

        if self.rviz_navgoal_list is None:
            self.rviz_navgoal_list = []
            self.rviz_navgoal_list.append(pose_with_offset)
            rospy.loginfo("First goal received, storing in list.")
        elif len(self.rviz_navgoal_list) == 1:
            self.rviz_navgoal_list.append(pose_with_offset)
            rospy.loginfo("Second goal received, storing in list.")
        elif len(self.rviz_navgoal_list) > 1:
            rospy.logwarn("Waypoint list is too long, only the last two goals will be used.")
            self.rviz_navgoal_list = [self.rviz_navgoal_list[-1]]
            self.rviz_navgoal_list.append(pose_with_offset)
        
        # create a Path and publish
        stored_waypoints = Path()
        stored_waypoints.header.frame_id = self.workplane_id #self.base_frame
        stored_waypoints.poses = self.rviz_navgoal_list
        self.stored_wp_path_pub.publish(stored_waypoints)

        # create the movei navgoal list
        self.moveit_navgoal_list = []
        listener = tf.TransformListener()
        for pose in self.rviz_navgoal_list:
            try:
                listener.waitForTransform(self.base_frame, self.workplane_id, rospy.Time(0), rospy.Duration(4.0))
                moveit_pose = listener.transformPose(self.base_frame, pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(f"TF Exception: {e}")
                return False
            rospy.loginfo(f"Received goal pose: {moveit_pose.pose}")
            self.moveit_navgoal_list.append(moveit_pose)
        return True

    def apply_rpy_offset(self, pose_stamped, roll_offset, pitch_offset, yaw_offset):
        # Extract current orientation
        quat = (
            pose_stamped.pose.orientation.x,
            pose_stamped.pose.orientation.y,
            pose_stamped.pose.orientation.z,
            pose_stamped.pose.orientation.w
        )

        # Convert to rotation matrix
        original_matrix = tf.transformations.quaternion_matrix(quat)

        # Create a rotation matrix from the RPY offset
        offset_matrix = tf.transformations.euler_matrix(roll_offset, pitch_offset, yaw_offset)

        # Apply the offset: new_rotation = original * offset
        new_matrix = tf.transformations.concatenate_matrices(original_matrix, offset_matrix)

        # Convert back to quaternion
        new_quat = tf.transformations.quaternion_from_matrix(new_matrix)

        # Update the pose
        pose_stamped.pose.orientation = Quaternion(*new_quat)

        return pose_stamped

if __name__ == "__main__":

    try:
        client = MoveItSweeperClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
