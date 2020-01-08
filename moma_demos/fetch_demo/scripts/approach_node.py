#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
import scipy.ndimage
from PIL import Image
from matplotlib import pyplot as plt
import cv2

from fetch_demo.msg import ApproachAction, ApproachResult
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from moma_utils.ros_conversions import waypoint_to_pose_msg

DEBUG = False


class ApproachActionServer:
    """
        When called, this action should find a collision free position for the robot,
        as close as possible to the target location (target object), facing it, and navigate
        the robot there using the navigation action.
    """

    def __init__(self):
        # Obtain map from map server
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.map = None

        # Connection to navigation stack
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        self.move_base_client.wait_for_server()

        # Launch action server
        action_name = "approach_action"
        self.action_server = actionlib.SimpleActionServer(
            action_name, ApproachAction, execute_cb=self.approach_cb, auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("Approach action server started.")

        # Set some parameters
        self.robot_width_m = 1.0

    def approach_cb(self, msg):
        rospy.loginfo("Start approaching object")
        result = ApproachResult()

        target_pos_px = np.array([1276, 2573])

        # Find closest free point on map
        assert self.map_data[target_pos_px[0], target_pos_px[1]] == 100

        first_empty_pos_px = []
        first_empty_distance = 1000
        found_first_empty_pos = False
        for radius in range(1, 100):
            for x_idx in range(
                target_pos_px[0] - radius, target_pos_px[0] + radius + 1
            ):
                if (
                    x_idx == target_pos_px[0] - radius
                    or x_idx == target_pos_px[0] + radius
                ):
                    # In the first and last column, iterate through everything
                    y_idx_range = range(
                        target_pos_px[1] - radius, target_pos_px[1] + radius + 1
                    )
                else:
                    # Only check first and last value
                    y_idx_range = [target_pos_px[1] - radius, target_pos_px[1] + radius]

                for y_idx in y_idx_range:
                    if self.map_data[x_idx, y_idx] == 0:
                        # We found an empty pixel, save its target distance
                        distance_to_target = np.linalg.norm(
                            np.array([x_idx, y_idx]) - np.array(target_pos_px)
                        )
                        if distance_to_target < first_empty_distance:
                            found_first_empty_pos = True
                            first_empty_distance = distance_to_target
                            first_empty_pos_px = [x_idx, y_idx]

                        # TODO Now check if the robot can be placed here
            if found_first_empty_pos:
                break
        assert len(first_empty_pos_px) == 2

        # Convert the found location from image coordinates to map frame
        first_empty_pos_m = (
            np.array(first_empty_pos_px) * self.map_resolution + self.map_origin
        )
        target_pos_m = np.array(target_pos_px) * self.map_resolution + self.map_origin
        direction_vector = first_empty_pos_m - target_pos_m
        direction_vector /= np.linalg.norm(direction_vector)

        # Move the found position further away, to account for the robot size
        robot_goal_pos_m = (
            first_empty_pos_m + direction_vector * self.robot_width_m * 0.7
        )
        robot_goal_pos_px = (robot_goal_pos_m - self.map_origin) / self.map_resolution

        if DEBUG:
            # Display map and the locations we found
            plt.matshow(self.map_data, interpolation=None)
            plt.scatter(target_pos_px[1], target_pos_px[0], s=50, c="red")
            plt.scatter(first_empty_pos_px[1], first_empty_pos_px[0], s=50, c="cyan")
            plt.scatter(robot_goal_pos_px[1], robot_goal_pos_px[0], s=50, c="lime")
            plt.show()

        # Move there using the navigation action
        # TODO verify that this yaw angle is correct.
        yaw = np.arctan2(-direction_vector[1], -direction_vector[0])
        # Negative sign for direction_vector because we want vector from robot position to object position
        waypoint = np.hstack((robot_goal_pos_m, np.rad2deg(yaw)))
        # TODO verify whether waypoint is in the correct frame.
        if not self._visit_waypoint(waypoint):
            return

        rospy.loginfo("Finished approach")
        self.action_server.set_succeeded(result)

    def map_cb(self, msg):
        rospy.loginfo("Received map")
        self.map = msg
        self.map_width = self.map.info.width
        self.map_resolution = self.map.info.resolution
        self.map_origin = np.array(
            [msg.info.origin.position.x, msg.info.origin.position.y]
        )
        assert msg.info.origin.position.z == 0.0
        assert msg.info.origin.orientation.x == 0.0
        assert msg.info.origin.orientation.y == 0.0
        assert msg.info.origin.orientation.z == 0.0
        assert msg.info.origin.orientation.w == 1.0

        self.map_data = np.array(msg.data)
        self.map_data = np.reshape(self.map_data, (self.map_width, self.map_width))

    def _visit_waypoint(self, waypoint):
        pose = waypoint_to_pose_msg(waypoint)
        navigation_goal = MoveBaseGoal(target_pose=pose)
        self.move_base_client.send_goal(navigation_goal)
        state = GoalStatus.PENDING
        while state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
            if self.action_server.is_preempt_requested():
                self.move_base_client.cancel_all_goals()
                self.action_server.set_preempted()
                return False
            rospy.sleep(0.5)
            state = self.move_base_client.get_state()

        if state is not GoalStatus.SUCCEEDED:
            rospy.logerr("Failed to navigate to approach waypoint.")
            self.action_server.set_aborted()
            return False

        rospy.loginfo("Reached approach waypoint.")
        return True

    def _construct_robot_mask(self):
        """
            This function is currently not in use. Would be useful if we want to check in the
            future whether the robot actually can be positioned there collision free. 
        """

        robot_width_px = int(self.robot_width_m / self.map_resolution)

        robot_mask = np.zeros((self.map_width, self.map_width))
        lower_bound = (self.map_width - robot_width_px) / 2
        upper_bound = (self.map_width + robot_width_px) / 2
        robot_mask[lower_bound:upper_bound, lower_bound:upper_bound] = 100

        # Shift and rotate robot
        angle = np.deg2rad(10.0)
        rotation_matrix = np.array(
            [[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]]
        )
        robot_mask_transformed = scipy.ndimage.affine_transform(
            robot_mask, rotation_matrix, offset=self.map_width / 2
        )
        plt.imshow(robot_mask)
        plt.show()
        plt.imshow(robot_mask_transformed)
        plt.show()


def main():
    rospy.init_node("approach_action_node")
    action = ApproachActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
