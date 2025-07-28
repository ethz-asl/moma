import moveit_commander
import rospy
import tf
import numpy as np

from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import String

from visualization_msgs.msg import Marker, MarkerArray

class ReachabilityChecker:
    def __init__(self):
        rospy.init_node('reachability_checker_node', anonymous=False)

        # Initialize MoveIt
        moveit_commander.roscpp_initialize([])
        self.arm_group = moveit_commander.MoveGroupCommander("panda_arm")
        rospy.loginfo("MoveIt client initialized.")
        self.controlled_frame = self.arm_group.get_end_effector_link()
        # self.arm_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
        # self.arm_group.set_planner_id("LIN")
        rospy.loginfo(f"Controlled frame: {self.controlled_frame}")
        self.base_frame = self.arm_group.get_planning_frame()
        rospy.loginfo(f"Base frame for planning: {self.base_frame}")

        # Markers
        self.workplane_marker = None
        self.sweep_markers = None
        self.grasp_markers = None

        # Workplane parameters
        self.workplane_origin = np.array([0.555, 0.0, 0.3])
        self.workplane_width = 0.5
        self.workplane_length = 0.5

        # Initialize workplane grid points
        self.top_down_grid_points = None
        self.valid_topdown_grasp = []
        self.top_down_grasp_checker()
        self.sweep_checker()

        # Publishers
        self.marker_pub = rospy.Publisher('/reachability_checker/markers', MarkerArray, queue_size=10)
        self.chat_pub = rospy.Publisher('/reachability_checker/chat', String, queue_size=10)

        # Start periodic timer for marker publishing
        rospy.Timer(rospy.Duration(1.0), self.publish_markers)

    def top_down_grasp_checker(self):
        # sample a uniform grid of points on the workplane
        x_points = np.linspace(self.workplane_origin[0] - self.workplane_length / 2,
                               self.workplane_origin[0] + self.workplane_length / 2, 10)
        y_points = np.linspace(self.workplane_origin[1] - self.workplane_width / 2,
                               self.workplane_origin[1] + self.workplane_width / 2, 10)
        
        self.top_down_grid_points = np.array(np.meshgrid(x_points, y_points)).T.reshape(-1, 2)        
        self.valid_topdown_grasp = []
        print("Current end effector orientation:", self.arm_group.get_current_pose().pose.orientation)
        cnt_success = 0
        cnt_fail = 0
        cnt_total = 0
        self.grasp_markers = []
        for point in self.top_down_grid_points:
            cnt_total += 1
            if cnt_total % 100 == 0:
                rospy.loginfo(f"Checking point {cnt_total}/{len(self.top_down_grid_points)}: {point}")
                rospy.loginfo(f"Progress: {cnt_total / len(self.top_down_grid_points) * 100:.2f}%")
            # Create a target pose for each grid point
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.base_frame
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = point[0]
            target_pose.pose.position.y = point[1]
            target_pose.pose.position.z = self.workplane_origin[2]
            target_pose.pose.orientation = Quaternion(-1, 0, 0, 0)  # Default orientation

            # create marker already
            point_marker = Marker()
            point_marker.header.frame_id = self.base_frame
            point_marker.header.stamp = rospy.Time.now()
            point_marker.ns = "grid_points"
            point_marker.id = cnt_total
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = point[0]
            point_marker.pose.position.y = point[1]
            point_marker.pose.position.z = self.workplane_origin[2] + 0.01
            point_marker.pose.orientation = Quaternion(0, 0, 0, 1)
            point_marker.scale.x = 0.02
            point_marker.scale.y = 0.02
            point_marker.scale.z = 0.02 # Small sphere for grid points  
            point_marker.lifetime = rospy.Duration(1.5)
            point_marker.color.a = 1.0

            try:
                plan = self.arm_group.plan(target_pose)
            except Exception as e:
                rospy.logwarn(f"Planning failed for point {point}: {e}")
                cnt_fail += 1
                point_marker.color.r = 1.0
                point_marker.color.g = 0.0
                point_marker.color.b = 0.0
                self.grasp_markers.append(point_marker)
                self.valid_topdown_grasp.append(False)
                continue

            cnt_success += 1
            point_marker.color.r = 0.0
            point_marker.color.g = 1.0
            point_marker.color.b = 0.0

            self.grasp_markers.append(point_marker)
            self.valid_topdown_grasp.append(True)
            self.arm_group.clear_pose_targets()            
        
        rospy.loginfo(f"Reachability check complete: {cnt_success} success, {cnt_fail} fail, total {cnt_total} points checked.")
        rospy.loginfo(f"Ratio of reachable points: {cnt_success / cnt_total:.2f}")

    def sweep_checker(self):
        # use the four corner points of the workplane to define six sweeps
        corner_points = [
            (self.workplane_origin[0] - self.workplane_length / 2, self.workplane_origin[1] - self.workplane_width / 2),
            (self.workplane_origin[0] - self.workplane_length / 2, self.workplane_origin[1] + self.workplane_width / 2),
            (self.workplane_origin[0] + self.workplane_length / 2, self.workplane_origin[1] - self.workplane_width / 2),
            (self.workplane_origin[0] + self.workplane_length / 2, self.workplane_origin[1] + self.workplane_width / 2)
        ]
        # find all combinations of two distinct corner points
        from itertools import combinations
        corner_combinations = list(combinations(corner_points, 2))
        # define sweeps as tuples of start and end points
        self.checked_sweeps = []
        self.sweep_validity = []
        
        for start, end in corner_combinations:
            # move arm to "home" position
            self.arm_group.set_named_target("home")
            self.arm_group.go(wait=True)
            self.arm_group.clear_pose_targets()
            # create a target pose for the start point
            start_pose = PoseStamped()
            start_pose.header.frame_id = self.base_frame
            start_pose.header.stamp = rospy.Time.now()
            start_pose.pose.position.x = start[0]
            start_pose.pose.position.y = start[1]
            start_pose.pose.position.z = self.workplane_origin[2]
            start_pose.pose.orientation = Quaternion(-1, 0, 0, 0)  # Default orientation
            # create a target pose for the end point
            end_pose = PoseStamped()
            end_pose.header.frame_id = self.base_frame
            end_pose.header.stamp = rospy.Time.now()
            end_pose.pose.position.x = end[0]
            end_pose.pose.position.y = end[1]
            end_pose.pose.position.z = self.workplane_origin[2]
            end_pose.pose.orientation = Quaternion(-1, 0, 0, 0)  # Default orientation
            # go to the start point
            '''
            self.arm_group.set_pose_target(start_pose)
            try:
                self.arm_group.go(wait=True)
            except Exception as e:
                rospy.logwarn(f"Failed to reach start point {start}: {e}")
            # go to the end point
            self.arm_group.set_pose_target(end_pose)
            try:
                self.arm_group.go(wait=True)
            except Exception as e:
                rospy.logwarn(f"Failed to reach end point {end}: {e}")
            # check if the arm can reach the end point
            '''

            self.checked_sweeps.append((start, end))
        # self.checked_sweeps = sweeps
        print(f"Checked {len(self.checked_sweeps)} sweeps.")

    def publish_markers(self, event):
        # MarkerArray message
        marker_array = MarkerArray()

        # Workplane as CUBE marker
        workplane_marker = Marker()
        workplane_marker.header.frame_id = self.base_frame
        workplane_marker.header.stamp = rospy.Time.now()
        workplane_marker.ns = "workplane"
        workplane_marker.id = 0
        workplane_marker.type = Marker.CUBE
        workplane_marker.action = Marker.ADD
        workplane_marker.pose.position.x = self.workplane_origin[0]
        workplane_marker.pose.position.y = self.workplane_origin[1]
        workplane_marker.pose.position.z = self.workplane_origin[2]
        workplane_marker.pose.orientation = Quaternion(0, 0, 0, 1)
        workplane_marker.scale.x = self.workplane_length
        workplane_marker.scale.y = self.workplane_width
        workplane_marker.scale.z = 0.01
        workplane_marker.color.r = 0.0
        workplane_marker.color.g = 0.0
        workplane_marker.color.b = 1.0
        workplane_marker.color.a = 0.5
        workplane_marker.lifetime = rospy.Duration(1.5)  # Lifetime a bit longer than timer rate
        marker_array.markers.append(workplane_marker)

        if self.grasp_markers is not None:
            # add all grasp markers to the marker array
            for i, marker in enumerate(self.grasp_markers):
                marker_array.markers.append(marker)

        # create line markers for the sweeps
        for i, (start, end) in enumerate(self.checked_sweeps):
            print(f"Creating sweep marker {i} from {start} to {end}")
            sweep_marker = Marker()
            sweep_marker.header.frame_id = self.base_frame
            sweep_marker.header.stamp = rospy.Time.now()
            sweep_marker.ns = "sweeps"
            sweep_marker.id = i + 100
            sweep_marker.type = Marker.LINE_LIST
            sweep_marker.action = Marker.ADD
            sweep_marker.pose.position.x = 0.0
            sweep_marker.pose.position.y = 0.0
            sweep_marker.pose.position.z = self.workplane_origin[2] + 0.01
            sweep_marker.pose.orientation = Quaternion(0, 0, 0, 1)
            sweep_marker.scale.x = 0.01
            sweep_marker.color.r = 1.0
            sweep_marker.color.g = 0.0
            sweep_marker.color.b = 0.0
            sweep_marker.color.a = 1.0
            # sweep_marker.lifetime = rospy.Duration(1.5)
            # Add start and end points
            point_start = PoseStamped()
            point_start.header.frame_id = self.base_frame
            point_start.header.stamp = rospy.Time.now()
            point_start.pose.position.x = start[0]
            point_start.pose.position.y = start[1]
            point_start.pose.position.z = self.workplane_origin[2] + 0.01
            point_end = PoseStamped()
            point_end.header.frame_id = self.base_frame
            point_end.header.stamp = rospy.Time.now()
            point_end.pose.position.x = end[0]
            point_end.pose.position.y = end[1]
            point_end.pose.position.z = self.workplane_origin[2] + 0.01
            sweep_marker.points.append(point_start.pose.position)
            sweep_marker.points.append(point_end.pose.position)
            marker_array.markers.append(sweep_marker)

        # Publish markers and chat
        self.marker_pub.publish(marker_array)
        self.chat_pub.publish("Workplane marker updated.")
        rospy.loginfo("Published workplane marker.")

if __name__ == "__main__":
    print("Starting reachability checker node...")
    checker = ReachabilityChecker()
    rospy.spin()
