# !/usr/bin/env python

# ==========================================

# import tf
# import rospy


# if __name__ == "__main__":
#     rospy.init_node('tf_listener')

#     listener = tf.TransformListener()

#     rate = rospy.Rate(0.5)
#     while not rospy.is_shutdown():
#         try:
#             (trans, rot) = listener.lookupTransform('/panda_link0', '/fixed_camera_depth_optical_frame',rospy.Time(0))
#             # (trans, rot) = listener.lookupTransform('/panda_link0', '/panda_link0',rospy.Time(0))
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             continue
#         rospy.loginfo(trans)
#         rospy.loginfo(rot)

#         rate.sleep()
# ===========================================


# #!/usr/bin/env python  
# import roslib
# # roslib.load_manifest('learning_tf')
# import rospy
# import math
# import tf
# import geometry_msgs.msg
# # import turtlesim.srv

# if __name__ == '__main__':
#     rospy.init_node('turtle_tf_listener')

#     listener = tf.TransformListener()

#     # rospy.wait_for_service('spawn')
#     # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
#     # spawner(4, 2, 0, 'turtle2')

#     # turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

#     rate = rospy.Rate(10.0)
#     while not rospy.is_shutdown():
#         try:
#             (trans,rot) = listener.lookupTransform('/panda_link0', '/fixed_camera_depth_optical_frame', rospy.Time(0))
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             continue
#         rospy.loginfo(trans)
#         rospy.loginfo(rot)

#         rate.sleep()

# ============================================

# Transform a given input pose from one fixed frame to another
# import rospy
# from geometry_msgs.msg import Pose

# import tf2_ros
# import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped


# def transform_pose(input_pose, from_frame, to_frame):

#     # **Assuming /tf2 topic is being broadcasted
#     tf_buffer = tf2_ros.Buffer()
#     listener = tf2_ros.TransformListener(tf_buffer)

#     pose_stamped = tf2_geometry_msgs.PoseStamped()
#     pose_stamped.pose = input_pose
#     pose_stamped.header.frame_id = from_frame
#     pose_stamped.header.stamp = rospy.Time.now()

#     try:
#         # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
#         output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
#         # return output_pose_stamped.pose
#         return output_pose_stamped

#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#         raise


# # Test Case
# rospy.init_node("transform_test")

# my_pose = Pose()
# my_pose.position.x = 0.11868091755792527
# my_pose.position.y = 0.7560097708784841
# my_pose.position.z = 0.908798066445653
# my_pose.orientation.x = 0
# my_pose.orientation.y = 0
# my_pose.orientation.z = 0
# my_pose.orientation.w = 1

# transformed_pose = transform_pose(my_pose, 'panda_link0', 'fixed_camera_depth_optical_frame')

# print(transformed_pose)

# ===========

# import numpy as np

# array = np.empty((3,0))

# attach = np.ones([3,1])

# # pose = []
# # pose.x = 1
# # pose.y = 2
# # pose.z = 3 

# print(array)
# print(attach)

# for i in range(4):
#     array = np.append(array,[[1*i+1],[2*i+1],[3*i+1]],axis=1)

# print array[:,0]

# ============

# import numpy as np

# imagePoints = np.array([[100,300,301,270,100,269,100,300,301,270,100,269],[200,200,300,170,100,223,200,200,300,170,100,223]])
# print(imagePoints)
# scores = np.ones(6)
# print(scores)

# xmin = 270
# xmax = 322
# ymin = 170
# ymax = 222

# index=0
# for point in imagePoints.T:
#     # if not in bouningbox
#     if not ((xmin <= point[0] and point[0] <= xmax) and (ymin <= point[1] and point[1] <= ymax)):
#         scores[index] = 0
#     index += 1

# print(imagePoints)
# print(scores)

# ===========================

# import numpy as np

# array = np.array(([[ 385.17615269, 186.99711113]],
#     [[ 375.42253251, 278.67455095]],
#     [[ 389.4435259 , 140.38865319]],
#     [[ 378.23627895, 267.8645684 ]],
#     [[ 382.24159113, 142.58895867]],
#     [[ 378.56537531, 273.5382508 ]],
#     [[ 378.23627895, 267.8645684 ]],
#     [[ 379.18272206, 279.42537118]],
#     [[ 401.13528924, 152.63481501]],
#     [[ 404.87858273, 144.09410682]],
#     [[ 407.63982291, 147.13153018]],
#     [[ 397.67367652, 152.92933582]]))

# print(array)
# # print(array.shape)

# array_flatt = array.flatten()

# # print(array_flatt)

# array_flatt_reshape = array_flatt.reshape(array_flatt.size/2,2)
# # print(array_flatt_reshape)

# # print(array_flatt_reshape.T)

# final_array = array_flatt_reshape.T

# print(final_array)

# direct_array = array.flatten().reshape(2,12).T

# # print(direct_array)

# ==============================

from py_trees.blackboard import Blackboard

while True:

    blackboard = Blackboard()
    result = blackboard.set("foo", "bar")
    foo = blackboard.get("foo")

    print(foo)
