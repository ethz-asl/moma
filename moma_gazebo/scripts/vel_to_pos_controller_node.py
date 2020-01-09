#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np


class VelToPosControllerNode:
    def __init__(self):
        # Load parameters
        self.rate = rospy.get_param('~rate', 125.0)
        default_joint_names = ["panda_joint{}".format(x) for x in range(1, 8)]
        self.joints = rospy.get_param('~joints', default_joint_names)

        # Set up internal parameters
        self.n_joints = len(self.joints)
        self.velocities = np.zeros(self.n_joints)
        self.positions = np.full(self.n_joints, np.nan)
        while not rospy.is_shutdown() and np.isnan(self.positions).any():
            msg = rospy.wait_for_message('/joint_states', JointState)
            for idx, joint in enumerate(self.joints):
                if joint in msg.name:
                    idx_msg = msg.name.index(joint)
                    self.positions[idx] = msg.position[idx_msg]
            rospy.sleep(1.0)

        # Set up subscriptions and publishers
        self.sub_vel = rospy.Subscriber('~command_vel_in',
                                        Float64MultiArray,
                                        self.cb_joint_vels, queue_size=1)
        self.pub_pos = rospy.Publisher('~command_pos_out', Float64MultiArray,
                                       queue_size=1)

        # Start timer for main loop
        self._timer = rospy.Timer(rospy.Duration(1/self.rate),
                                  self.publisher_loop)

    def publisher_loop(self, event):
        # Update positions
        self.positions += self.velocities/self.rate

        # Publish message
        msg = Float64MultiArray()
        msg.data = self.positions
        self.pub_pos.publish(msg)

    def cb_joint_vels(self, msg):
        data = np.array(msg.data)
        if data.shape != self.velocities.shape:
            rospy.logerr("Received wrong velocity command. Expected shape "
                         + "{}, received {}.".format(self.velocities.shape,
                                                     data.shape))
            return
        self.velocities = data


if __name__ == "__main__":
    rospy.init_node('VelToPosControllerNode')
    node = VelToPosControllerNode()
    rospy.spin()
