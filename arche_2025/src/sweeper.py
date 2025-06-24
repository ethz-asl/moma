import moveit_commander
import rospy

from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from visualization_msgs.msg import Marker, MarkerArray

class MoveItSweeperClient:
    def __init__(self):
        rospy.init_node('moveit_client_node', anonymous=True)

        self.workplane_id = "workplane"

        # Initialize moveit_commander and the move group for the arm
        moveit_commander.roscpp_initialize([])
        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")
        rospy.loginfo("MoveIt client initialized.")
        self.controlled_frame = self.arm_group.get_end_effector_link()
        self.arm_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
        self.arm_group.set_planner_id("LIN")

        # Publishers
        self.stored_wp_path_pub = rospy.Publisher("/move_base_simple/goal_path", Path, queue_size=10)
        self.viz_marker_pub = rospy.Publisher("/sweeper/visualization_marker", Marker, queue_size=10, latch=True)

        # Subscribers
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.rviz_navgoal_callback)

        # Services
        self.go_home_service = rospy.Service('go_home', Empty, self.go_home_service)
        self.follow_navgoal_service = rospy.Service('follow_navgoal', Empty, self.rviz_navgoal_callback)

        # Variables
        self.rviz_navgoal_list = None # this can only hold 1-2 goals at a time, so we can use it to store the last goal
        
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
        self.arm_group.set_named_target("home")
        success = self.arm_group.go(wait=True)
        return success

    def rviz_navgoal_callback(self, msg):
        # ensure msg is in the workplane frame
        if msg.header.frame_id != self.workplane_id:
            rospy.logerr("Received goal is not in the 'workplane' frame.")
            return False
        if self.rviz_navgoal_list is None:
            self.rviz_navgoal_list = []
            self.rviz_navgoal_list.append(msg)
            rospy.loginfo("First goal received, storing in list.")
        elif len(self.rviz_navgoal_list) == 1:
            self.rviz_navgoal_list.append(msg)
            rospy.loginfo("Second goal received, storing in list.")
        elif len(self.rviz_navgoal_list) > 1:
            rospy.logwarn("Waypoint list is too long, only the last two goals will be used.")
            self.rviz_navgoal_list = [self.rviz_navgoal_list[-1]]
            self.rviz_navgoal_list.append(msg)
        
        # create a Path and publish
        stored_waypoints = Path()
        stored_waypoints.header.frame_id = self.workplane_id
        stored_waypoints.poses = self.rviz_navgoal_list
        self.stored_wp_path_pub.publish(stored_waypoints)

        return True

    def go_home_service(self, req):
        success = self.go_home()
        if success:
            rospy.loginfo("Successfully moved to home position.")
        else:
            rospy.logerr("Failed to move to home position.")
        return success        

if __name__ == "__main__":
    try:
        client = MoveItSweeperClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
