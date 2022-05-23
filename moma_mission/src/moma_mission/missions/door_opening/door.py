import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from moma_mission.utils.robot import Robot
from moma_mission.utils.transforms import *
from nav_msgs.msg import Path
from numpy import newaxis


class Door:
    world_frame = "world"
    hinge_frame = "door_hinge"
    door_base_frame = "shelf"
    handle_frame = "handle_link"
    R_H_EE = R.from_euler("xyz", [0.0, -90.0, 0.0], degrees=True).as_dcm()

    # the ee is placed exactly at the end effector --> they have zero relative translation
    T_H_EE = pin.SE3(R_H_EE, np.array([0.0, 0.0, 0.0]))

    def __init__(self, description_name) -> None:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_world_door_bc = tf2_ros.StaticTransformBroadcaster()
        self.pinocchio_robot = Robot(description_name)

    def generate_door_opening_trajectory(
        self, opening_angle: float, opening_speed: float
    ) -> Path:
        """
        Generate a door opening profile, using the urdf model as a method to compute subsequent grasp
        locations as a function of the opening angle
        """
        path = Path()
        path.header.frame_id = self.world_frame
        steps = 20
        increment = 1.0 * opening_angle / steps
        t0 = rospy.get_rostime()
        dt = increment / opening_speed
        q = np.array([0.0])

        rospy.loginfo(
            f"Generating opening trajectory: increment={increment}, steps={steps}, dt waypoints={dt}"
        )
        for i in range(steps):
            q[0] = i * increment
            T_W_H = self.pinocchio_robot.get_frame_placement(self.handle_frame, q)
            T_W_EE = T_W_H.act(self.T_H_EE)

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.world_frame
            pose_stamped.header.stamp = t0 + rospy.Duration.from_sec(i * dt)
            pose_stamped.pose = se3_to_pose_ros(T_W_EE)
            path.poses.append(pose_stamped)
        return path

    def publish_base_frame_from_hand(self, world_frame, ee_frame):
        transform = self.tf_buffer.lookup_transform(
            ee_frame,  # target frame
            world_frame,  # source frame
            # tf at first available time
            rospy.Time(0),
            rospy.Duration(3),
        )
        T_EE_W = tf_to_se3(transform)

        transform = self.tf_buffer.lookup_transform(
            self.door_base_frame,  # target frame
            self.handle_frame,  # source frame
            # tf at first available time
            rospy.Time(0),
            rospy.Duration(3),
        )
        T_B_H = tf_to_se3(transform)

        # get the chained transform from world to base
        T_B_W = T_B_H.act(self.T_H_EE.act(T_EE_W))
        T_W_B = T_B_W.inverse()

        tf_world_door = TransformStamped()
        tf_world_door.header.stamp = rospy.Time.now()
        tf_world_door.header.frame_id = self.world_frame
        tf_world_door.child_frame_id = self.door_base_frame
        tf_world_door.transform.translation.x = T_W_B.translation[0]
        tf_world_door.transform.translation.y = T_W_B.translation[1]
        tf_world_door.transform.translation.z = T_W_B.translation[2]
        q = R.from_dcm(T_W_B.rotation).as_quat()
        tf_world_door.transform.rotation.x = q[0]
        tf_world_door.transform.rotation.y = q[1]
        tf_world_door.transform.rotation.z = q[2]
        tf_world_door.transform.rotation.w = q[3]

        print(
            f"Transform from {self.world_frame} to {self.door_base_frame} is \n{T_W_B}"
        )

        self.tf_world_door_bc.sendTransform(tf_world_door)
        rospy.sleep(2.0)
