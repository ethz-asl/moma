#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import Trigger, TriggerResponse
import rosbag
from datetime import datetime
import os

class ImageSaverNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('image_saver_node', anonymous=True)

        # Fetch the topic names from parameters
        # image_topics_str = rospy.get_param('~image_topics', 
        #                                    '/rs_435_2/color/image_raw, \
        #                                    /rs_435_2/depth/image_rect_raw \
        #                                    /rs_435_2/infra1/image_rect_raw \
        #                                     /rs_435_3/color/image_raw, \
        #                                    /rs_435_3/depth/image_rect_raw \
        #                                    /rs_435_3/infra1/image_rect_raw \
        #                                    /royale_cam_flexx2_vga_1/gray_image_0 \
        #                                    /royale_cam_flexx2_vga_1/gray_image_1 \
        #                                     /royale_cam_flexx2_vga_1/gray_image_1_mono8')
        image_topics_str =    '/rs_435_2/color/image_raw, \
            /rs_435_2/depth/image_rect_raw, \
            /rs_435_2/infra1/image_rect_raw, \
            /rs_435_3/color/image_raw, \
            /rs_435_3/depth/image_rect_raw, \
            /rs_435_3/infra1/image_rect_raw'

        # camera_info_topics_str = rospy.get_param('~camera_info_topics', 
        #                                          '/rs_435_2/color/camera_info, \
        #                                          /rs_435_2/depth/camera_info \
        #                                         /rs_435_2/infra1/camera_info, \
        #                                         /rs_435_3/color/camera_info, \
        #                                          /rs_435_3/depth/camera_info \
        #                                         /rs_435_3/infra1/camera_info \
        #                                             /royale_cam_flexx2_vga_1/camera_info')
        camera_info_topics_str = '/rs_435_2/color/camera_info, \
                                    /rs_435_2/depth/camera_info, \
                                /rs_435_2/infra1/camera_info, \
                                /rs_435_3/color/camera_info, \
                                    /rs_435_3/depth/camera_info, \
                                /rs_435_3/infra1/camera_info'

        self.save_directory = rospy.get_param('~save_directory', '/root/moma_ws/bags')

        # Split the comma-separated topic names into lists
        self.image_topics = [topic.strip() for topic in image_topics_str.split(',') if topic.strip()]
        self.camera_info_topics = [topic.strip() for topic in camera_info_topics_str.split(',') if topic.strip()]

        # Validate the parameters
        # if len(self.image_topics) != len(self.camera_info_topics):
        #     rospy.logerr('The number of image topics must be equal to the number of camera_info topics.')
        #     rospy.signal_shutdown('Invalid parameters')
        #     return

        # Create internal storage for the messages
        # assert len(self.image_topics) == len(self.camera_info_topics), "Number of image topics and camera_info topics must be equal!"
        self.num_streams_img = len(self.image_topics)
        self.num_streams_info = len(self.camera_info_topics)
        self.last_images = [None] * self.num_streams_img
        self.last_camera_infos = [None] * self.num_streams_info
        # Subscribe to the image topics
        for i in range(len(self.image_topics)):
            print('Image stream: ' + self.image_topics[i])
            rospy.Subscriber(self.image_topics[i], Image, self.image_callback, i)
            # rospy.Subscriber(self.camera_info_topics[i], CameraInfo, self.camera_info_callback, i)
        for i in range(len(self.camera_info_topics)):
            print('Info stream: ' + self.camera_info_topics[i])
            # rospy.Subscriber(self.image_topics[i], Image, self.image_callback, i)
            rospy.Subscriber(self.camera_info_topics[i], CameraInfo, self.camera_info_callback, i)

        # Create a service to trigger the saving to a ROSbag
        self.close_bag_service = rospy.Service('close_bag', Trigger, self.close_bag)
        self.save_service = rospy.Service('save_snapshots_to_bag', Trigger, self.save_snapshots_to_bag)
        # create directory
        if not os.path.exists(self.save_directory):
            os.makedirs(self.save_directory)

        # Create a ROSbag file with the current timestamp
        timestamp = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        self.bag_filename = os.path.join(self.save_directory, f'images_{timestamp}.bag')
        self.bag = rosbag.Bag(self.bag_filename, 'w')

    def image_callback(self, msg, camera_index):
        self.last_images[camera_index] = msg

    def camera_info_callback(self, msg, camera_index):
        self.last_camera_infos[camera_index] = msg

    def close_bag(self, req):
        self.bag.close()
    
    def save_snapshots_to_bag(self, req):
        if any(img is None for img in self.last_images) or any(info is None for info in self.last_camera_infos):
            rospy.logwarn('Not all images or camera infos are received yet.')
            return TriggerResponse(success=False, message='Not all images or camera infos are received yet.')

        timestamp = rospy.Time.now()

        try:
            for i in range(len(self.image_topics)):
                if self.last_images[i] is not None:
                    img = self.last_images[i]
                    img.header.stamp = timestamp
                    self.bag.write(self.image_topics[i], img)
            for i in range(len(self.camera_info_topics)):
                if self.last_camera_infos[i] is not None:
                    info = self.last_camera_infos[i]
                    info.header.stamp = timestamp
                    self.bag.write(self.camera_info_topics[i], self.last_camera_infos[i])
        finally:
            print('stored at ', timestamp)
        return TriggerResponse(success=True, message=f'Successfully created snapshot')

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ImageSaverNode()
    node.run()
