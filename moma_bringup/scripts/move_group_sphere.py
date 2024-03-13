#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
import scipy as sc
from scipy.spatial.transform import Rotation

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class SampleSphericalPoses:
    """ """

    def __init__(self):
        super(SampleSphericalPoses, self).__init__()

        # Set random generator
        self.rng = np.random.default_rng()

        # Set radius Z-distance from EE to tag
        self.radius = 0.5

        # Set target (TODO get this from world//real tag)
        self.target_centre = np.array([0.5, 0.0, 0.2])

        # Set sphere centre as offset from target
        self.sphere_centre = self.target_centre + np.array([0.0, 0.0, 0.1])

        # Set camera offset
        self.cam_offset = np.array([0.0, -0.05, 0.0])

        # Set phi & theta limits for sphere
        self.theta_limits = [0.0, np.pi / 2]
        self.phi_limits = [-5 * (np.pi/8), 5* (np.pi/8)]

    def get_pose(self):
        """
        Get randomely generated samples of spherical coordinate,
        and get pose to target.

        #TODO -> rearange the functions

        @returns: tuple (pos, ori)
            pos: A list of cartesian positions [x, y, z]
            ori: A list of quaternion orientations [x, y, z, w]
        """

        # Get a randomely sampled phi & theta
        phi = self.get_phi(self.rng.random())
        theta = self.get_theta(self.rng.random())
        print(f"(r, theta, phi) = ({self.radius}, {theta}, {phi})")

        # The positional part of the pose in cartesian
        pos = self.spherical2cartesian(self.radius, theta, -phi)
        pos = self.add_offset(pos, self.sphere_centre)
        print(f"(x, y, z) = ({pos[0]}, {pos[1]}, {pos[2]})")

        rot_matrix = self.get_rotation_matrix(pos, self.target_centre)
        if not self.is_rotation_matrix(rot_matrix):
            raise ValueError("Rotation matrix is invalid")
        else:
            ori = self.rotation2quat(rot_matrix)
            print(f"(x, y, z, w) = ({ori[0]}, {ori[1]}, {ori[2]}. {ori[3]})")

            # return pos, ori
            # adding cam offset
            cam_point = self.apply_quat_rotation(ori, self.cam_offset)
            pos_cam_centre = self.add_offset(pos, cam_point)
            rot_matrix = self.get_rotation_matrix(pos_cam_centre, self.target_centre)
            if not self.is_rotation_matrix(rot_matrix):
                raise ValueError("Rotation matrix is invalid")
            else:
                ori = self.rotation2quat(rot_matrix)
                print(f"Cam: (x, y, z, w) = ({ori[0]}, {ori[1]}, {ori[2]}. {ori[3]})")
                return pos, ori

    def get_phi(self, rand):
        """
        Longitude (EW)
        """
        return (self.phi_limits[1] - self.phi_limits[0]) * rand + self.phi_limits[0]

    def get_theta(self, rand):
        """
        Latitude (NS) from equator
        """
        return (self.theta_limits[1] - self.theta_limits[0]) * rand + self.theta_limits[
            0
        ]

    def spherical2cartesian(self, radius, theta, phi):
        """ """
        # Taking equatorial theta, so need to caluate azimuth
        azi = (np.pi / 2.0) - theta

        x = radius * np.sin(azi) * np.cos(phi)
        y = radius * np.sin(azi) * np.sin(phi)
        z = radius * np.cos(azi)

        return np.array([x, y, z])

    def add_offset(self, position, offset):
        """ """
        return position + offset

    def get_rotation_matrix(self, position, target):
        """
        NEED TO GET TARGET POSITION

        MAYBE PUT IN EXAMPLE FROM SIM
        """
        world_z = np.array([0, 0, 1])
        # world X (1,0,0)
        # world Y (0,1,0)

        # target is 2 dimensions from world
        # find R3 (Z) as normalised difference to target
        # target is down so negative needed TODO add in function
        R3 = position - target
        R3 /= -np.linalg.norm(R3)

        R1 = np.cross(R3, world_z)
        R1 /= np.linalg.norm(R1)

        R2 = np.cross(R3, R1)
        R2 /= np.linalg.norm(R2)

        R = np.c_[R1, R2, R3]
        print("R : ", R)

        return R

    def is_rotation_matrix(self, R):
        # square matrix test
        if R.ndim != 2 or R.shape[0] != R.shape[1]:
            return False

        should_be_identity = np.allclose(
            R.dot(R.T), np.identity(R.shape[0], np.float64)
        )
        should_be_one = np.allclose(np.linalg.det(R), 1)
        return should_be_identity and should_be_one

    def rotation2quat(self, rotation):
        """
        Pass in rotation
        """
        quat = Rotation.from_matrix(rotation).as_quat()
        return quat

    def apply_quat_rotation(self, quat, point):
        """ """
        rot = Rotation.from_quat(quat)
        return rot.apply(point)

    def pointZAxisAt(self, point):
        """!  Given a point, this function orients the object such that
        its z-axis points at the given point
        @param point point in world coordinates
        """
        z_axis_target = Vector(point) - self.getWorldPos()
        self.alignZAxisTo(z_axis_target)

    def pointZAxisAway(self, point):
        """!  Given a point, this function orients the object such that
        its z-axis points in the opposite direction of the given point
        @param point point in world coordinates
        """
        z_axis_target = Vector(point) - self.getWorldPos()
        self.alignZAxisTo(-z_axis_target)

    def alignZAxisTo(self, direction):
        """!  Given a direction vector, this function computes the
        rotation quaternion, such that the new x-axis is horizontal
        and the z-axis points in the specified direction
        @param direction list/array/Vector of 3 values
        @returns mathutils.Quaternion representation the rotation
                 that is needed to rotate the object
        """
        world_z = np.array([0, 0, 1], dtype=np.float)
        new_z = direction
        new_x = np.cross(world_z, new_z)
        new_x /= np.linalg.norm(new_x)
        new_y = np.cross(new_z, new_x)
        new_y /= np.linalg.norm(new_y)
        rotmat = Matrix(np.array([new_x, new_y, new_z]).T)
        self.setOrientation(rotmat.to_quaternion())


class MoveGroupSphereSamples(object):
    """MoveGroupSphereSamples"""

    def __init__(self):
        super(MoveGroupSphereSamples, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_test", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_home_state(self):
        """
        Planning to a Joint Goal
        The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        thing we want to do is move it to a slightly better configuration.
        We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        """
        # We get the joint values from the group and change some of the values:
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, position, orientation):
        """
        Planning to a Pose Goal
        Plan a motion for this group to a desired pose for the
        end-effector:
        """

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = orientation[0]
        pose_goal.orientation.y = orientation[1]
        pose_goal.orientation.z = orientation[2]
        pose_goal.orientation.w = orientation[3]

        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]

        self.move_group.set_pose_target(pose_goal)

        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(self.display_trajectory)

    def execute_plan(self, plan):
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        self.move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

def plot_3D_no_truth(position_list):
    # Extract x, y, and z coordinates
    x = [pos[0] for pos in position_list]
    y = [pos[1] for pos in position_list]
    z = [pos[2] for pos in position_list]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_trisurf(x, y, z, color="white", edgecolors="grey", alpha=0.5)

    ax.scatter(x, y, z, c="cyan", alpha=0.5)
    # add here different colours for made position and didn't make position

    # Set labels for the axes
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # Show the plot
    plt.legend()
    plt.show()


def plot_3D(position_list, truth_list):
    # Extract x, y, and z coordinates
    x = [pos[0] for pos in position_list]
    y = [pos[1] for pos in position_list]
    z = [pos[2] for pos in position_list]

    # Separate points into two lists based on the truth values
    true_points = [position_list[i] for i in range(len(position_list)) if truth_list[i]]
    false_points = [
        position_list[i] for i in range(len(position_list)) if not truth_list[i]
    ]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot_trisurf(x, y, z, color="white", edgecolors="grey", alpha=0.5)

    # add here different colours for made position and didn't make position
    # Scatter plot for True points (in one color)
    ax.scatter(
        [pos[0] for pos in true_points],
        [pos[1] for pos in true_points],
        [pos[2] for pos in true_points],
        c="cyan",
        alpha=0.5,
    )

    # Scatter plot for False points (in another color)
    ax.scatter(
        [pos[0] for pos in false_points],
        [pos[1] for pos in false_points],
        [pos[2] for pos in false_points],
        c="magenta",
        label="did not reach",
        alpha=0.5,
    )
    # Set labels for the axes
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # Show the plot
    plt.legend()
    plt.show()


def main():
    move = MoveGroupSphereSamples()
    move.go_to_home_state()

    pos_list = []
    truth_list = []

    while True:
        try:
            print("")
            print("----------------------------------------------------------")
            print("Sampling sphere coordinates for wrist calibration")
            print("----------------------------------------------------------")
            print("Press 'h' to go to a home state")
            print("----------------------------------------------------------")
            print("Press 'enter' to go to the next sample point")
            print("----------------------------------------------------------")
            print("Press 'g' to plot a graph")
            print("----------------------------------------------------------")
            print("Press 'q' to stop sampling exit")

            print("")

            user_input = input("============ Choose an option: ")
            print("")

            if user_input == "h":
                move.go_to_home_state()
            elif user_input == "g":
                plot_3D(pos_list, truth_list)
                #plot_3D_no_truth(pos_list)
            elif user_input == "q":
                print("============ Sampling complete!")
                print("")
                break
            else:
                sample = SampleSphericalPoses()
                print("getting random pose on sphere")
                print("")
                pos, ori = sample.get_pose()
                print("pos :", pos)
                print("ori :", ori)
                print("")
                pos_list.append(pos)
                if not move.go_to_pose_goal(pos, ori):
                    print("Trajectory not executed")
                    print("")
                    truth_list.append(False)
                else:
                    truth_list.append(True)

        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return


if __name__ == "__main__":
    main()
