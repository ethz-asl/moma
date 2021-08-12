#! usr/bin/env python
import rospy
from ocs2_msgs.msg import mpc_target_trajectories
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def callback(msg):
    global frame_id
    global out_publisher
    path = Path()
    path.header.frame_id = frame_id
    for i in range(len(msg.timeTrajectory)):
        if i == 0:
            path.header.stamp = rospy.Time.from_sec(msg.timeTrajectory[0])
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = rospy.Time.from_sec(msg.timeTrajectory[i])
        pose_stamped.pose.position.x = msg.stateTrajectory[i].value[0]
        pose_stamped.pose.position.y = msg.stateTrajectory[i].value[1]
        pose_stamped.pose.position.z = msg.stateTrajectory[i].value[2]
        pose_stamped.pose.orientation.x = msg.stateTrajectory[i].value[3]
        pose_stamped.pose.orientation.y = msg.stateTrajectory[i].value[4]
        pose_stamped.pose.orientation.z = msg.stateTrajectory[i].value[5]
        pose_stamped.pose.orientation.w = msg.stateTrajectory[i].value[6]
        path.poses.append(pose_stamped)
    out_publisher.publish(path)


rospy.init_node("mpc_target_relay_node")
frame_id = rospy.get_param("~frame_id")
in_topic = rospy.get_param("~in_topic")
out_topic  = rospy.get_param("~out_topic")

in_subscriber = rospy.Subscriber(in_topic, mpc_target_trajectories, callback, queue_size=10)
out_publisher = rospy.Publisher(out_topic, Path, queue_size=1)

rospy.spin()
