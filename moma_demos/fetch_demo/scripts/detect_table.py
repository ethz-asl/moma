import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

rospy.init_node("test")

topic_name = "/camera/depth/image_rect_raw"
cv_bridge = CvBridge()


while True:
    img_msg = rospy.wait_for_message(topic_name, Image)
    img = cv_bridge.imgmsg_to_cv2(img_msg)

    num_pixels = np.logical_and(img < 1000, img > 0).sum()
    num_valid = (img > 0).sum()

    print(float(num_pixels) / num_valid)
    rospy.sleep(0.1)
