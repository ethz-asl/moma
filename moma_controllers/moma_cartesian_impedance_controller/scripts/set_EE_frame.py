#!/usr/bin/env python3
from typing import List
import rospy
import actionlib
import numpy as np
from franka_msgs.srv import SetEEFrame, SetEEFrameRequest, SetEEFrameResponse

rospy.init_node("set_ee_frame")

client = rospy.ServiceProxy("/franka_control/set_EE_frame", SetEEFrame)
rospy.loginfo(f"Waiting for service server...")
client.wait_for_service()
rospy.loginfo(f"... service server found")

NE_T_EE = np.array([[0., 1., 0., 0.],
                    [1., 0., 0., 0.],
                    [0., 0., -1., 0.01],
                    [0., 0., 0., 1.]])
NE_T_EE = np.reshape(NE_T_EE.transpose(), -1).tolist()

goal = SetEEFrameRequest()
goal.NE_T_EE = NE_T_EE
response : SetEEFrameResponse =  client.call(goal)
print(f"result:\n{response}")
