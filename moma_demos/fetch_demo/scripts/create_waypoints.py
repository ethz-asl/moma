from __future__ import print_function
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.spatial.transform import Rotation


def point_clicked(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orn = msg.pose.pose.orientation
    rot = Rotation([orn.x, orn.y, orn.z, orn.w])
    yaw = rot.as_euler("xyz", degrees=True)[2]
    print("[{}, {}, {}]".format(x, y, yaw))


def main():
    rospy.init_node("waypoint_generator")
    sub = rospy.Subscriber(
        "/initialpose", PoseWithCovarianceStamped, callback=point_clicked
    )
    rospy.spin()


if __name__ == "__main__":
    main()
