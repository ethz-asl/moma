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

        target_pos = np.array([1276, 2573])

        # Display map
        plt.imshow(self.map_data, interpolation="nearest")
        plt.show()

        # Find closest free point on map
        assert self.map_data[target_pos[0], target_pos[1]] == 100

        # robot_width_px = int(self.robot_width_m / self.map_resolution)

        # robot_mask = np.zeros((self.map_width, self.map_width))
        # lower_bound = (self.map_width - robot_width_px) / 2
        # upper_bound = (self.map_width + robot_width_px) / 2
        # robot_mask[lower_bound:upper_bound, lower_bound:upper_bound] = 100

        # Shift and rotate robot
        # angle = np.deg2rad(10.0)
        # rotation_matrix = np.array(
        #     [[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]]
        # )
        # robot_mask_transformed = scipy.ndimage.affine_transform(
        #     robot_mask, rotation_matrix, offset=self.map_width / 2
        # )
        # plt.imshow(robot_mask)
        # plt.show()
        # plt.imshow(robot_mask_transformed)
        # plt.show()

        first_empty_pos = []
        first_empty_distance = 1000
        found_first_empty_pos = False
        for radius in range(1, 100):
            for x_idx in range(target_pos[0] - radius, target_pos[0] + radius + 1):
                if x_idx == target_pos[0] - radius or x_idx == target_pos[0] + radius:
                    # In the first and last column, iterate through everything
                    y_idx_range = range(
                        target_pos[1] - radius, target_pos[1] + radius + 1
                    )
                else:
                    # Only check first and last value
                    y_idx_range = [target_pos[1] - radius, target_pos[1] + radius]

                for y_idx in y_idx_range:
                    if self.map_data[x_idx, y_idx] == 0:
                        # We found an empty pixel, save its target distance
                        distance_to_target = np.linalg.norm(
                            np.array([x_idx, y_idx]) - np.array(target_pos)
                        )
                        if distance_to_target < first_empty_distance:
                            found_first_empty_pos = True
                            first_empty_distance = distance_to_target
                            first_empty_pos = [x_idx, y_idx]

                        # Now check if the robot can be placed here
                        # for distance in range(robot_width_px / 2, robot_width_px):
                        #     pass
            if found_first_empty_pos:
                break
        assert len(first_empty_pos) == 2

        # Convert the found location from image coordinates to map frame
        first_empty_pos_m = (
            np.array(first_empty_pos) * self.map_resolution + self.map_origin
        )
        target_pos_m = np.array(target_pos) * self.map_resolution + self.map_origin
        direction_vector = first_empty_pos_m - target_pos_m
        direction_vector /= np.linalg.norm(direction_vector)

        # Move the found position further away, to account for the robot size
        robot_goal_pos = first_empty_pos_m + direction_vector * self.robot_width_m * 0.7

        # Move there using the navigation action

        rospy.sleep(2.0)

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


def main():
    rospy.init_node("approach_action_node")
    action = ApproachActionServer()
    rospy.spin()


if __name__ == "__main__":
    main()
