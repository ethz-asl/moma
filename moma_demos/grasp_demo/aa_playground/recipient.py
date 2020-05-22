#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("aa_emitter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def callback(data):
    pub = rospy.Publisher('aa_emitter2', String, queue_size=10)
    # rospy.init_node('aa_emitter2', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    listener()
