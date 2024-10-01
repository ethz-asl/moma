#!/usr/bin/env python

from __future__ import annotations  # for type hinting
from typing import Callable

import os
import rospy
import rospkg
import numpy as np

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Trigger

from moma_actions.base_robot_interface import Move, AtPose, ParamClient, TopicClient, ComponentStatus

import moma_utils.ros.gazebo_utils as gazebo_utils
import moma_utils.ros.transform_utils as utils


class GiraffeMove(Move):

    # TODO add local vs global to checking at_goal functions!!!
    def __init__(
        self,
        goal_pose: Pose,
        feedback_pub: Callable[[Pose], None],
        timeout: rospy.Rate = 1.0,
        tol_lin: float = 0.05,
        tol_ang: float = 5.0 * np.pi / 180.0,
        max_lin_vel: float = 0.2,
        max_ang_vel: float = 0.2,
    ) -> None:
        super().__init__(goal_pose)
        self.timeout = rospy.Rate(timeout)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.twist_pub = rospy.Publisher(
            "/mobile_base/swerve_controller/cmd_vel", Twist, queue_size=10
        )
        self.feedback_pub = feedback_pub
        rospy.sleep(1)  # wait for subscriber to latch

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose

    def init_move(self) -> bool:
        """
        check if within goal tolerance, then move based on distance
        first x, then y, then turn in yaw
        """
        while not rospy.is_shutdown():
            self.feedback_pub(self.current_pose)

            if self.move():
                self.feedback_pub(self.current_pose)
                rospy.loginfo("Success, robot reached move goal")
                return True
            else:
                self.feedback_pub(self.current_pose)
                rospy.logerr("Failure, robot did not reach move goal")
                return False

        return False

    def compute_twist(self) -> Twist:
        twist_msg = Twist()

        goal_x = self.goal_pose.position.x
        goal_y = self.goal_pose.position.y

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        delta_x_global = goal_x - current_x
        delta_y_global = goal_y - current_y

        goal_quat = utils.array_from_pose(self.goal_pose)[3:]
        goal_yaw = utils.angle_from_quaternion(goal_quat)
        current_quat = utils.array_from_pose(self.current_pose)[3:]
        current_yaw = utils.angle_from_quaternion(current_quat)

        delta_x_local = (
            np.cos(current_yaw) * delta_x_global
            + np.sin(current_yaw) * delta_y_global
        )
        delta_y_local = (
            -np.sin(current_yaw) * delta_x_global
            + np.cos(current_yaw) * delta_y_global
        )

        delta_yaw = utils.wrap_angle(goal_yaw - current_yaw)

        twist_msg.linear.x = np.clip(
            delta_x_local, -self.max_lin_vel, self.max_lin_vel
        )
        twist_msg.linear.y = np.clip(
            delta_y_local, -self.max_lin_vel, self.max_lin_vel
        )
        twist_msg.angular.z = np.clip(
            delta_yaw, -self.max_ang_vel, self.max_ang_vel
        )

        rospy.loginfo(f"MoveX: Sending velocity {twist_msg.linear.x:.2f} [m/s] to x")
        rospy.loginfo(f"MoveY: Sending velocity {twist_msg.linear.y:.2f} [m/s] to y")
        rospy.loginfo(f"MoveYaw: Sending velocity {twist_msg.angular.z:.2f} [rad/s] to yaw")

        return twist_msg

    def move(self) -> bool:
        """
        move the robot towards the goal using local frame
        """
        if self.at_goal_x() and self.at_goal_y() and self.at_goal_yaw():
            return True

        while (
            not self.at_goal_x()
            or not self.at_goal_y()
            or not self.at_goal_yaw()
        ):
            self.feedback_pub(self.current_pose)
            twist_msg = self.compute_twist()
            self.twist_pub.publish(twist_msg)
            self.timeout.sleep()

        self.twist_pub.publish(self.compute_empty_twist())  # stop if at goal

        return self.at_goal_x() and self.at_goal_y() and self.at_goal_yaw()

    def compute_empty_twist(self) -> Twist:
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        return twist_msg

    def at_goal_x(self) -> bool:
        goal_x = self.goal_pose.position.x
        current_x = self.current_pose.position.x

        # rospy.logwarn(f"X[current, goal] ({current_x:.2f}, {goal_x:.2f}) [m,m]")

        return abs(goal_x - current_x) < self.tol_lin

    def at_goal_y(self) -> bool:
        goal_y = self.goal_pose.position.y
        current_y = self.current_pose.position.y

        # rospy.logwarn(f"Y[current, goal] ({current_y:.2f}, {goal_y:.2f}) [m,m]")

        return abs(goal_y - current_y) < self.tol_lin

    def at_goal_yaw(self) -> bool:
        goal_quat = utils.array_from_pose(self.goal_pose)[3:]
        goal_yaw = utils.angle_from_quaternion(goal_quat)
        goal_yaw = utils.wrap_angle(goal_yaw)

        current_quat = utils.array_from_pose(self.current_pose)[3:]
        current_yaw = utils.angle_from_quaternion(current_quat)
        current_yaw = utils.wrap_angle(current_yaw)

        # rospy.logwarn(
        #     f"Yaw[current, goal] ({current_yaw:.2f}, {goal_yaw:.2f}) [rad,rad]"
        # )

        yaw_diff = utils.wrap_angle(goal_yaw - current_yaw)

        return abs(yaw_diff) < self.tol_ang

    def displacement_from_pose(
        self, target_pose: np.array | Pose
    ) -> tuple[np.array, np.array]:
        """Get distance between target and current pose [m, deg]"""
        if type(target_pose) == Pose:
            target_pose = utils.array_from_pose(target_pose)

        target_pose_xy = target_pose[:2]
        target_pose_th = utils.angle_from_quaternion(target_pose[3:], "yaw")
        target_pose_th = utils.wrap_angle(target_pose_th)

        current_pose = self.current_pose
        if type(current_pose) == Pose:
            current_pose = utils.array_from_pose(current_pose)

        current_pose_xy = current_pose[:2]
        current_pose_th = utils.angle_from_quaternion(current_pose[3:], "yaw")
        current_pose_th = utils.wrap_angle(current_pose_th)

        disp_xy = round(
            np.linalg.norm(current_pose_xy - target_pose_xy) - 0.5, 3
        )
        delta_th = target_pose_th - current_pose_th

        disp_th = round(delta_th, 3)
        disp_th = utils.wrap_angle(disp_th)  # wrapping around [-180,180]

        rospy.logwarn(
            f"Robot at distance {abs(disp_xy):.2f}[m] and {abs(disp_th):.2f}[rad] from target"
        )

        return disp_xy, disp_th

class GiraffeMoveX(Move):
    def __init__(
        self,
        distance: float,
        feedback_pub: Callable[[Pose], None],
        max_lin_vel: float = 0.2,
        tol_lin: float = 0.05,
        reverse: bool = False,
        kp: float = 0.8,
    ):
        super().__init__()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.twist_pub = rospy.Publisher(
            "/mobile_base/swerve_controller/cmd_vel", Twist, queue_size=10
        )
        self.distance = distance
        self.feedback_pub = feedback_pub
        self.max_lin_vel = max_lin_vel
        self.tol_lin = tol_lin
        self.kp = kp
        self.reverse = reverse

        self.initial_x = None
        self.initial_y = None

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose
        if self.initial_x == None or self.initial_y == None:
            self.initial_x = self.current_pose.position.x
            self.initial_y = self.current_pose.position.y

    def init_move(self) -> bool:
        """
        Move the robot forward in the x direction until it reaches the goal
        """
        if not self.reverse:
            rospy.loginfo(f"moving forwards {self.distance} m")
        else:
            rospy.loginfo(f"moving backwards {self.distance} m")

        if self.at_goal_x():
            return True

        while not rospy.is_shutdown() and not self.at_goal_x():
            self.feedback_pub(self.current_pose)
            twist_msg = self.compute_twist_x()
            self.twist_pub.publish(twist_msg)
            self.rate.sleep()

        # Stop the robot once it reaches the goal
        self.twist_pub.publish(self.compute_empty_twist())
        rospy.loginfo("Success, reached x goal")

        return self.at_goal_x()

    def compute_twist_x(self) -> Twist:
        """
        Computes the twist message to move in the x direction
        """
        twist_msg = Twist()

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        current_quat = self.current_pose.orientation
        current_yaw = utils.angle_from_quaternion([
            current_quat.x,
            current_quat.y,
            current_quat.z,
            current_quat.w
        ])

        delta_x_global = current_x - self.initial_x
        delta_y_global = current_y - self.initial_y

        delta_x_local = np.cos(current_yaw) * delta_x_global + np.sin(current_yaw) * delta_y_global
        vel = self.kp * (self.distance - delta_x_local)
        if self.reverse:
            vel = -vel

        twist_msg.linear.x = np.clip(vel, -self.max_lin_vel/2, self.max_lin_vel/2)
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0  # No rotation needed

        rospy.loginfo(
            f"MoveX: Sending velocity {twist_msg.linear.x:.2f} [m/s] in x direction"
        )
        return twist_msg

    def compute_empty_twist(self) -> Twist:
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        return twist_msg

    def at_goal_x(self) -> bool:
        """
        Check if the robot has moved the desired distance
        """
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        current_quat = self.current_pose.orientation
        current_yaw = utils.angle_from_quaternion([
            current_quat.x,
            current_quat.y,
            current_quat.z,
            current_quat.w
        ])

        delta_x_global = current_x - self.initial_x
        delta_y_global = current_y - self.initial_y

        delta_x_local = np.cos(current_yaw) * delta_x_global + np.sin(current_yaw) * delta_y_global

        if self.reverse:
            target_x = -self.distance
        else:
            target_x = self.distance

        rospy.logwarn(f"target_x: {target_x:.2f}, delta_x_local: {delta_x_local:.2f}")

        return abs(target_x - delta_x_local) < self.tol_lin

class GiraffeMoveY(Move):
    def __init__(
        self,
        distance: float,
        feedback_pub: Callable[[Pose], None],
        max_lin_vel: float = 0.2,
        tol_lin: float = 0.05,
        left: bool = False,
        kp: float = 0.8,
    ):
        super().__init__()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.twist_pub = rospy.Publisher(
            "/mobile_base/swerve_controller/cmd_vel", Twist, queue_size=10
        )
        self.distance = distance
        self.feedback_pub = feedback_pub
        self.max_lin_vel = max_lin_vel
        self.tol_lin = tol_lin
        self.kp = kp
        self.left = left

        self.initial_x = None
        self.initial_y = None

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose
        if self.initial_x == None or self.initial_y == None:
            self.initial_x = self.current_pose.position.x
            self.initial_y = self.current_pose.position.y

    def init_move(self) -> bool:
        """
        Move the robot forward in the x direction until it reaches the goal
        """
        if not self.left:
            rospy.loginfo(f"moving right {self.distance} m")
        else:
            rospy.loginfo(f"moving left {self.distance} m")

        if self.at_goal_y():
            return True

        while not rospy.is_shutdown() and not self.at_goal_y():
            self.feedback_pub(self.current_pose)
            twist_msg = self.compute_twist_y()
            self.twist_pub.publish(twist_msg)
            self.rate.sleep()

        # Stop the robot once it reaches the goal
        self.twist_pub.publish(self.compute_empty_twist())
        rospy.loginfo("Success, reached y goal")

        return self.at_goal_y()

    def compute_twist_y(self) -> Twist:
        """
        Computes the twist message to move in the x direction
        """
        twist_msg = Twist()

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        current_quat = self.current_pose.orientation
        current_yaw = utils.angle_from_quaternion([
            current_quat.x,
            current_quat.y,
            current_quat.z,
            current_quat.w
        ])

        delta_x_global = current_x - self.initial_x
        delta_y_global = current_y - self.initial_y

        delta_y_local = -np.sin(current_yaw) * delta_x_global + np.cos(current_yaw) * delta_y_global
        vel = self.kp * (self.distance - delta_y_local)
        if not self.left:
            vel = -vel

        twist_msg.linear.x = 0.0
        twist_msg.linear.y = np.clip(vel, -self.max_lin_vel/2, self.max_lin_vel/2)
        twist_msg.angular.z = 0.0  # No rotation needed

        rospy.loginfo(
            f"MoveY: Sending velocity {twist_msg.linear.y:.2f} [m/s]"
        )
        return twist_msg

    def compute_empty_twist(self) -> Twist:
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        return twist_msg

    def at_goal_y(self) -> bool:
        """
        Check if the robot has moved the desired forward distance
        """
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        current_quat = self.current_pose.orientation
        current_yaw = utils.angle_from_quaternion([
            current_quat.x,
            current_quat.y,
            current_quat.z,
            current_quat.w
        ])

        delta_x_global = current_x - self.initial_x
        delta_y_global = current_y - self.initial_y

        delta_y_local = -np.sin(current_yaw) * delta_x_global + np.cos(current_yaw) * delta_y_global

        if not self.left:
            target_y = -self.distance
        else:
            target_y = self.distance

        return abs(target_y - delta_y_local) < self.tol_lin

class GiraffeTurnYaw(Move):
    def __init__(
        self,
        angle: float,
        feedback_pub: Callable[[Pose], None],
        max_lin_vel: float = 0.2,
        ccw: bool = False,
        kp: float = 0.8,
    ):
        super().__init__()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.twist_pub = rospy.Publisher(
            "/mobile_base/swerve_controller/cmd_vel", Twist, queue_size=10
        )
        self.angle = angle
        self.feedback_pub = feedback_pub
        self.max_lin_vel = max_lin_vel
        self.kp = kp
        self.ccw = ccw

        self.initial_yaw = None

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose
        if self.initial_yaw == None:
            current_quat = self.current_pose.orientation
            self.initial_yaw = utils.angle_from_quaternion([
                current_quat.x,
                current_quat.y,
                current_quat.z,
                current_quat.w
            ])
            self.target_yaw = utils.wrap_angle(self.initial_yaw + self.angle)

    def init_move(self) -> bool:
        """
        Move the robot forward in the x direction until it reaches the goal
        """
        if not self.ccw:
            rospy.loginfo(f"turning clockwise {self.angle:.2f} rad")
        else:
            rospy.loginfo(f"turning counter clockwise {self.angle:2f} rad")

        if self.at_goal_yaw():
            return True

        while not rospy.is_shutdown() and not self.at_goal_yaw():
            self.feedback_pub(self.current_pose)
            twist_msg = self.compute_twist_yaw()
            self.twist_pub.publish(twist_msg)
            self.rate.sleep()

        # Stop the robot once it reaches the goal
        self.twist_pub.publish(self.compute_empty_twist())
        rospy.loginfo("Success, reached y goal")

        return self.at_goal_yaw()

    def compute_twist_yaw(self) -> Twist:
        """
        Computes the twist message to turn in the yaw direction
        """
        twist_msg = Twist()

        current_quat = self.current_pose.orientation
        current_yaw = utils.angle_from_quaternion([
            current_quat.x,
            current_quat.y,
            current_quat.z,
            current_quat.w
        ])

        delta_yaw = utils.wrap_angle(current_yaw - self.target_yaw)
        rospy.logwarn(f"[target, cur]: [{self.target_yaw:.2f}, {current_yaw:.2f}]")
        rospy.logwarn(f"[delta_yaw]: [{delta_yaw:.2f}]")

        if not self.ccw and delta_yaw > 0: 
            delta_yaw -= 2 * np.pi
        elif self.ccw and delta_yaw < 0:
            delta_yaw += 2 * np.pi  

        vel = self.kp * delta_yaw

        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = np.clip(vel, -self.max_ang_vel/2, self.max_ang_vel/2)

        rospy.loginfo(
            f"TurnYaw: Sending velocity {twist_msg.angular.z:.2f} [rad/s]"
        )
        return twist_msg

    def compute_empty_twist(self) -> Twist:
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.angular.z = 0.0
        return twist_msg

    def at_goal_yaw(self) -> bool:
        """
        Check if the robot has moved the desired forward distance
        """

        current_quat = self.current_pose.orientation
        current_yaw = utils.angle_from_quaternion([
            current_quat.x,
            current_quat.y,
            current_quat.z,
            current_quat.w
        ])
        
        yaw_diff = utils.wrap_angle(self.target_yaw - current_yaw)

        return abs(yaw_diff) < self.tol_ang

class GiraffeComponents():
    def __init__(self):
        self.init_srvs()
        self.init_publishers()
        self.init_gazebo_models()

    def init_publishers(self):
        self.hatch_up_pub = rospy.Publisher("/hatch_up", Bool, queue_size=10, latch=True)
        self.roller_up_pub = rospy.Publisher("/roller_up", Bool, queue_size=10, latch=True)

        rospy.sleep(0.5)
        msg = Bool()
        msg.data = True
        self.hatch_up_pub.publish(msg)
        self.roller_up_pub.publish(msg)
        rospy.logwarn("published messages")

    def init_srvs(self):
        self.open_hatch_srv = rospy.get_param("~open_hatch_srv", "/open_hatch")
        self.close_hatch_srv = rospy.get_param("~close_hatch_srv", "/close_hatch")
        self.raise_roller_srv = rospy.get_param("~raise_roller_srv", "/raise_roller")
        self.lower_roller_srv = rospy.get_param("~lower_roller_srv", "/lower_roller")
        
        rospy.wait_for_service(self.open_hatch_srv)
        rospy.wait_for_service(self.close_hatch_srv)
        rospy.wait_for_service(self.raise_roller_srv)
        rospy.wait_for_service(self.lower_roller_srv)

    def init_gazebo_models(self):
        pkg = rospkg.RosPack()
        pkg_path = pkg.get_path('heron_demo')

        model_mesh_path = os.path.join(pkg_path + '/meshes')
        gazebo_model_path = os.getenv('GAZEBO_MODEL_PATH', '')
        
        if model_mesh_path not in gazebo_model_path:
            new_gazebo_model_path = f"{gazebo_model_path}:{model_mesh_path}" if gazebo_model_path else model_mesh_path
            os.environ['GAZEBO_MODEL_PATH'] = new_gazebo_model_path
            print(f"GAZEBO_MODEL_PATH set to: {new_gazebo_model_path}")
        else:
            print(f"GAZEBO_MODEL_PATH already contains: {model_mesh_path}")

        self.arr_name = "arrow"
        self.arr_path = pkg_path + "/urdf/arrow.sdf"

        self.open_pose = Pose()
        self.open_pose.position.x = 0.0
        self.open_pose.position.y = 0.0
        self.open_pose.position.z = 0.0
        self.open_pose.orientation.w = 1.0

        self.close_pose = Pose()
        self.close_pose.position.x = 0.0
        self.close_pose.position.y = 0.0
        self.close_pose.position.z = 0.54
        self.close_pose.orientation.x = 1.0 

        self.raise_pose = Pose()
        self.raise_pose.position.x = 2.0
        self.raise_pose.position.y = 1.0
        self.raise_pose.position.z = 0.0
        self.raise_pose.orientation.w = 1.0

        self.lower_pose = Pose()
        self.lower_pose.position.x = 2.0
        self.lower_pose.position.y = 1.0
        self.lower_pose.position.z = 3.0
        self.lower_pose.orientation.z = 1.0


    def trigger(self, component: str, cmd: str) -> bool:
        if component == "hatch":
            success = self.trigger_hatch(cmd)
        elif component == "roller":
            success = self.trigger_roller(cmd)
        else:
            raise ValueError(f"only [hatch, roller] allowed, {component}")
        
        return success
    
    def trigger_srv(self, srv_name: str) -> bool:
        try:
            srv = rospy.ServiceProxy(srv_name, Trigger)
            res = srv()
            if not res.success:
                rospy.logerr(f"{res.message}")
            
            return res.success
        except rospy.ServiceException as err:
            rospy.logerr(f"{self.open_hatch_srv} srv call failed: {err}")

        return False
        
    def trigger_hatch(self, cmd: str) -> bool:
        if cmd == "open":
            success = self.open_hatch()
        elif cmd == "close": 
           success = self.close_hatch()
        else:
           raise ValueError(f"cmd not valid: [{cmd}]")
        rospy.sleep(2.0)
        return success 

    def open_hatch(self) -> bool:
        rospy.loginfo("Opening hatch")
        msg = Bool()
        if not gazebo_utils.model_exists(self.arr_name):
            gazebo_utils.spawn_model(self.arr_name, self.arr_path, self.open_pose)
        else:
            gazebo_utils.teleport_model(self.arr_name, self.open_pose)

        if self.trigger_srv(self.open_hatch_srv):
            msg.data = False
            self.hatch_up_pub.publish(msg)
            return True
        else:
            msg.data = True
            self.hatch_up_pub.publish(msg)
            return False

    def close_hatch(self):
        rospy.loginfo("Closing hatch")
        msg = Bool()
        if not gazebo_utils.model_exists(self.arr_name):
            gazebo_utils.spawn_model(self.arr_name, self.arr_path, self.close_pose)
        else:
            gazebo_utils.teleport_model(self.arr_name, self.close_pose)
            rospy.sleep(1.0)
            gazebo_utils.delete_model(self.arr_name)
            rospy.sleep(1.0)

        if self.trigger_srv(self.close_hatch_srv):
            msg.data = True
            self.hatch_up_pub.publish(msg)
            return True
        else:
            msg.data = False
            self.hatch_up_pub.publish(msg)
            return False

    def trigger_roller(self, cmd: str) -> bool:
        if cmd == "raise":
            success = self.raise_roller()
        elif cmd == "lower": 
           success = self.lower_roller()
        else:
           raise ValueError(f"cmd not valid: [{cmd}]")

        return success 

    def raise_roller(self) -> bool:
        rospy.loginfo("Raising roller")
        msg = Bool()
        if not gazebo_utils.model_exists(self.arr_name):
            gazebo_utils.spawn_model(self.arr_name, self.arr_path, self.close_pose)
        else:
            gazebo_utils.teleport_model(self.arr_name, self.close_pose)
            rospy.sleep(1.0)
            gazebo_utils.delete_model(self.arr_name)
            rospy.sleep(1.0)

        if self.trigger_srv(self.raise_roller_srv):
            msg.data = True
            self.roller_up_pub.publish(msg)
            return True
        else:
            msg.data = False
            self.roller_up_pub.publish(msg)
            return False
        
    def lower_roller(self):
        rospy.loginfo("Lowering roller")
        msg = Bool()
        if not gazebo_utils.model_exists(self.arr_name):
            gazebo_utils.spawn_model(self.arr_name, self.arr_path, self.open_pose)
        else:
            gazebo_utils.teleport_model(self.arr_name, self.open_pose)

        if self.trigger_srv(self.lower_roller_srv):
            msg.data = False
            self.roller_up_pub.publish(msg)
            return True
        else:
            msg.data = True
            self.roller_up_pub.publish(msg)
            return False

class GiraffeComponentStatus(ComponentStatus):
    def __init__(self, component: str) -> None: 
        super().__init__(component)
        self.hatch_up = None
        self._hatch_sig = False
        self.roller_up = None
        self._roller_sig = False

        if component == "hatch":
            rospy.Subscriber("/hatch_up", Bool, self.hatch_up_cb)
            self.wait_for_hatch_status()
        elif component == "roller":
            rospy.Subscriber("/roller_up", Bool, self.roller_up_cb)
            self.wait_for_roller_status()
        else:
            raise ValueError(f"only [hatch, roller] allowed, {component}")
        
    def is_hatch_up(self) -> Bool:
        rospy.loginfo(f"Is the hatch up {self.hatch_up}")
        return self.hatch_up

    def hatch_up_cb(self, msg: Bool) -> None:
        self.hatch_up = msg.data
        self._hatch_sig = True

    def wait_for_hatch_status(self, timeout: float = 5.0) -> bool:
        start_time = rospy.get_time()
        rate = rospy.Rate(10)
        while not self._hatch_sig and rospy.get_time() - start_time < timeout:
            rate.sleep()
        
        if self._hatch_sig:
            rospy.loginfo("hatch status recieved")
        else:
            rospy.logerr(f"timeout, hatch status not recieved in {timeout}[s]")
        return self._hatch_sig


    def is_roller_up(self) -> Bool:
        if self.roller_up is not None:
            rospy.loginfo(f"Is the roller up: {self.roller_up}")
            return self.roller_up
        else:
            rospy.logerr("self.roller_up is None")
            return False

    def roller_up_cb(self, msg: Bool) -> None:
        self.roller_up = msg.data
        self._roller_sig = True
        
    def wait_for_roller_status(self, timeout: float = 5.0) -> bool:
        start_time = rospy.get_time()
        rate = rospy.Rate(10)
        while not self._roller_sig and rospy.get_time() - start_time < timeout:
            rate.sleep()
        
        if self._roller_sig:
            rospy.loginfo("roller status recieved")
        else:
            rospy.logerr(f"timeout, roller status not recieved in {timeout}[s]")
        return self._roller_sig

class GiraffeAtPose(AtPose):
    def __init__(self) -> None:
        super().__init__()

        self.load_parameters()
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_cb)
        
        self.current_pose: Pose = None
        self.gazebo_pose: Pose = None

    def load_parameters(self) -> None:
        self.robot_name = rospy.get_param("~robot_name", "giraffe")

    def odom_cb(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose

    def amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.current_pose = msg.pose.pose

    def gazebo_cb(self, msg: ModelStates) -> None: 
        if self.robot_name in msg.name:
            for idx in range(len(msg.name)):
                if msg.name[idx] == self.robot_name:
                    break
                else:
                    continue
            self.gazebo_pose = msg.pose[idx]

    def at_pose(self, goal_pose: Pose, tol: float = 0.01, ground_truth: bool = True)-> bool:
        goal_arr = utils.array_from_pose(goal_pose)

        goal_xy = goal_arr[:2]
        goal_yaw = utils.angle_from_quaternion(goal_arr[3:])

        if ground_truth or self.current_pose is None:
            gazebo_arr = utils.array_from_pose(self.gazebo_pose)
            current_xy = gazebo_arr[:2]
            current_yaw = utils.angle_from_quaternion(gazebo_arr[3:])
        else:
            current_arr = utils.array_from_pose(self.current_pose)
            current_xy = current_arr[:2]
            current_yaw = utils.angle_from_quaternion(current_arr[3:])

        disp_xy = round(
            np.linalg.norm(current_xy - goal_xy) - 0.5, 3
        )
        delta_yaw = round(
            utils.wrap_angle(
                np.arctan2(
                    np.sin(goal_yaw - current_yaw),
                    np.cos(goal_yaw - current_yaw)
                )    
            ), 3
        )

        rospy.logwarn(f"Robot at dist {abs(disp_xy):.2}[m] and {delta_yaw:.2}[rad] from goal")
        
        return abs(disp_xy) < tol and abs(delta_yaw) < tol

class GiraffeParamClient(ParamClient):
    def __init__(self, param_name: str, check_condition: callable):
        self._param_name = param_name
        self._check_condition = check_condition

    def is_condition_met(self):
        param_val = rospy.get_param(self._param_name, None)
        return self._check_condition(param_val)

class GiraffeTopicClient(TopicClient):
    def __init__(self, topic_name: str, msg_type, check_condition: callable):
        self._is_condition_met = False
        self._check_condition = check_condition
        rospy.Subscriber(topic_name, msg_type, self._topic_cb)

    def _topic_cb(self, msg):
        self._is_condition_met = self._check_condition(msg)
        
    def is_condition_met(self):
        return self._is_condition_met
