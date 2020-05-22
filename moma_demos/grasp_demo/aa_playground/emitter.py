#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from time import sleep

def talker():
    pub = rospy.Publisher('aa_emitter', String, queue_size=10)
    rospy.init_node('aa_emitter', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
