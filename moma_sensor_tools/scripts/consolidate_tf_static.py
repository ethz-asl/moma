#!/usr/bin/env python3
import rospy
from tf2_msgs.msg import TFMessage


class TfStaticRepublisher:
    def __init__(self):
        self.transforms_id = []
        self.tf_msg = TFMessage()
        self.subscriber = rospy.Subscriber(
            "/tf_static", TFMessage, self._tf_cb, queue_size=100
        )
        self.publisher = rospy.Publisher(
            "/tf_static", TFMessage, latch=True, queue_size=100
        )

    def _tf_cb(self, msg):
        for trans in msg.transforms:
            if (
                trans.header.frame_id + trans.child_frame_id not in self.transforms_id
            ):  # use parent + child as a tf index
                self.tf_msg.transforms.append(trans)
                self.transforms_id.append(trans.header.frame_id + trans.child_frame_id)

        if self.publisher.get_num_connections() != 0:
            self.publisher.publish(self.tf_msg)


if __name__ == "__main__":
    rospy.init_node("tf_static_republisher")
    node = TfStaticRepublisher()
    rospy.spin()
