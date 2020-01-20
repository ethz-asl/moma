#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from moma_gazebo.srv import SetJoints, SetJointsResponse
import numpy as np
import re


class VelToPosControllerNode:
    def __init__(self):
        # Load parameters
        self.init_config = rospy.get_param('~init_joint_config', None)
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
            rospy.sleep(2.0)
            rospy.loginfo("Waiting for joint states to be published.")
        if self.init_config is not None:
            matches = re.findall(r"-J .*?(\w.*?)\s.*?([-+]?\d*\.\d+|\d+)",
                                 self.init_config)
            for name, value in matches:
                if name in self.joints:
                    idx = self.joints.index(name)
                    self.positions[idx] = float(value)

        # Set up subscriptions and publishers
        self.sub_vel = rospy.Subscriber('~command_vel_in',
                                        Float64MultiArray,
                                        self.cb_joint_vels, queue_size=1)
        self.sub_pos = rospy.Subscriber('~command_pos_in',
                                        Float64MultiArray,
                                        self.cb_joint_pos, queue_size=1)
        self.pub_pos = rospy.Publisher('~command_pos_out', Float64MultiArray,
                                       queue_size=1)

        # Set up service for changing arm configuration
        self.srv_j = rospy.Service('~set_joint_states', SetJoints,
                                   self.cb_set_joints)

        # Start timer for main loop
        self._timer = rospy.Timer(rospy.Duration(1/self.rate),
                                  self.publisher_loop)

    def cb_set_joints(self, req):
        resp = SetJointsResponse()
        if not req.states or len(req.states.data) != self.n_joints:
            resp.success = False
            return resp
        self.positions = req.states.data
        resp.success = True

        self.wait_for_joints()
        return resp

    def wait_for_joints(self):
        self.real_pos = np.full(self.n_joints, np.nan)
        sub_js = rospy.Subscriber('/joint_states',
                                  JointState,
                                  self.cb_joint_states, queue_size=1)
        while not rospy.is_shutdown():
            if not np.isnan(self.real_pos).any():
                err = np.abs(self.real_pos - self.positions)
                if (err <= 1e-3).all():
                    break
            rospy.sleep(0.25)

        sub_js.unregister()
        return

    def cb_joint_states(self, msg):
        for idx, joint in enumerate(self.joints):
            if joint in msg.name:
                idx_msg = msg.name.index(joint)
                self.real_pos[idx] = msg.position[idx_msg]

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

    def cb_joint_pos(self, msg):
        data = np.array(msg.data)
        if data.shape != self.positions.shape:
            rospy.logerr("Received wrong position command. Expected shape "
                         + "{}, received {}.".format(self.positions.shape,
                                                     data.shape))
            return
        self.positions = data


if __name__ == "__main__":
    rospy.init_node('VelToPosControllerNode')
    node = VelToPosControllerNode()
    rospy.spin()
