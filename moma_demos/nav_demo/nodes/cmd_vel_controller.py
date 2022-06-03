#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class VelControllerNode(object):
    """Moves ridgeback to desired pose via cmd_vel topic.
    """
    def __init__(self,distance,choice):
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.distance = distance
        self.back_or_forth = choice
        self.execute_cb()

    def execute_cb(self):
        vel_msg = Twist()
        speed = 0.2 # 20cm/s 0.2
        isForward = self.back_or_forth
        if(isForward == 'true'):
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)

        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < self.distance):
            #Publish the velocity
            self.velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Move goal achieved!")

def main():
    rospy.init_node("vel_controller_node")
    VelControllerNode()
    rospy.spin()

if __name__ == "__main__":
    main()
