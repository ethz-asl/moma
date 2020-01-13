#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
import scipy.ndimage
from matplotlib import pyplot as plt

from fetch_demo.msg import ApproachAction, ApproachResult
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

from moma_utils.ros_conversions import waypoint_to_pose_msg

from fetch_demo.common import MovingActionServer

DEBUG = False


class ApproachActionServer(MovingActionServer):
    """
        When called, this action should find a collision free position for the robot,
        as close as possible to the target location (target object), facing it, and navigate
        the robot there using the navigation action.

        Useful for testing in map frame [m]: x=26.7705631256 y=-0.539741873741
    """

    def __init__(self):
        action_name = "approach_action"
        super(ApproachActionServer, self).__init__(action_name, ApproachAction)
        if DEBUG:
            rospy.loginfo("Debug logging is active.")

        # Obtain map from map server
        rospy.Subscriber("/approach_module/map", OccupancyGrid, self.map_cb)
        self.map = None

        # Set some parameters
        self.robot_width_m = 1.0

    def action_callback(self, msg):
        """Action server callback.

        For debugging purposes, the target position [1276, 2573], given in pixels, is
        useful, i.e. on the table in KoZe.

        Args:
            msg (ApproachGoal): Message specifying the target object location in the map frame.
        """

        rospy.loginfo("Start approaching object")
        result = ApproachResult()

        target_pos_m = np.array(
            [msg.target_object_pose.position.x, msg.target_object_pose.position.y]
        )
        target_pos_px = self._convert_m_to_px(target_pos_m)

        # Find closest free point on map
        first_empty_pos_m = self._find_closest_free_space(target_pos_px)

        # Move the found position further away, to account for the robot size
        robot_goal_pos_m, direction_vector = self._move_position_away(
            first_empty_pos_m, target_pos_m
        )

        rospy.loginfo(
            "Found robot target location: x={}, y={}".format(
                robot_goal_pos_m[0], robot_goal_pos_m[1]
            )
        )

        # Visualize map and locations we found if desired
        if DEBUG:
            first_empty_pos_px = self._convert_m_to_px(first_empty_pos_m)
            robot_goal_pos_px = self._convert_m_to_px(robot_goal_pos_m)
            self._plot_map(target_pos_px, first_empty_pos_px, robot_goal_pos_px)

        # Move there using the navigation action
        yaw = np.arctan2(-direction_vector[1], -direction_vector[0])
        # Negative sign for direction_vector because we want vector from robot position to object position
        waypoint = np.hstack((robot_goal_pos_m, np.rad2deg(yaw)))
        if not self._visit_waypoint(waypoint):
            return

        rospy.loginfo("Finished approach")
        self.action_server.set_succeeded(result)

    def map_cb(self, msg):
        rospy.loginfo("Received map")
        self.map = msg
        self.map_width = self.map.info.width
        self.map_height = self.map.info.height
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
        self.map_data = np.reshape(self.map_data, (self.map_height, self.map_width))

    def _find_closest_free_space(self, target_pos_px):
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
        first_empty_pos_m = self._convert_px_to_m(np.array(first_empty_pos_px))

        return first_empty_pos_m

    def _move_position_away(self, first_empty_pos_m, target_pos_m):
        direction_vector = first_empty_pos_m - target_pos_m
        direction_vector /= np.linalg.norm(direction_vector)

        robot_goal_pos_m = (
            first_empty_pos_m
            + direction_vector * self.robot_width_m * 0.5
            - 0.3 * direction_vector
        )
        return robot_goal_pos_m, direction_vector

    def _plot_map(self, target_pos_px, first_empty_pos_px, robot_goal_pos_px):
        plt.matshow(self.map_data, interpolation=None)
        plt.scatter(target_pos_px[1], target_pos_px[0], s=50, c="red")
        plt.scatter(first_empty_pos_px[1], first_empty_pos_px[0], s=50, c="cyan")
        plt.scatter(robot_goal_pos_px[1], robot_goal_pos_px[0], s=50, c="lime")
        plt.show()

    def _convert_m_to_px(self, position_m):
        position_px_unrounded = (position_m - self.map_origin) / self.map_resolution
        position_px_rounded = position_px_unrounded.astype(np.int32)
        position_px_rounded_flipped = np.flip(position_px_rounded, axis=0)
        return position_px_rounded_flipped

    def _convert_px_to_m(self, position_px_flipped):
        position_px = np.flip(position_px_flipped, axis=0)
        return position_px * self.map_resolution + self.map_origin

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
