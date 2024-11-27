#!/usr/bin/env python3

import cv_bridge
import rospy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import std_srvs.srv

from robot_helpers.ros import tf
from robot_helpers.ros.conversions import *
from vgn.perception import UniformTSDFVolume
import vgn.srv
from vgn.utils import *
import numpy as np
import message_filters

class UniformTSDFServer:
    def __init__(self):
        self.load_parameters()
        tf.init()
        self.init_topics()
        self.advertise_services()
        self.cv_bridge = cv_bridge.CvBridge()
        self.integrate = False
        rospy.loginfo("TSDF server with static cam setup ready")

    def load_parameters(self):
        self.frame_id = rospy.get_param("~frame_id")
        self.length = rospy.get_param("~length")
        self.resolution = rospy.get_param("~resolution")
        self.depth_scaling = rospy.get_param("~depth_scaling")

        self.cam_frame_ids = rospy.get_param("~camera/frame_ids")
        self.info_topics = rospy.get_param("~camera/info_topics")
        self.depth_topics = rospy.get_param("~camera/depth_topics")

        self.intrinsics = {}
        for camera_name, info_topic in zip(self.cam_frame_ids, self.info_topics):
            msg = rospy.wait_for_message(info_topic, CameraInfo)
            self.intrinsics[camera_name] = from_camera_info_msg(msg)

        print(f"cam_frame_ids: {self.cam_frame_ids}")
        print(f"info_topics: {self.info_topics}")
        print(f"self.depth_topics: {self.depth_topics}")
        print(f"self.intrinsics: {self.intrinsics}")

    def init_topics(self):
        self.scene_cloud_pub = rospy.Publisher("scene_cloud", PointCloud2, queue_size=1)
        self.map_cloud_pub = rospy.Publisher("map_cloud", PointCloud2, queue_size=1)

        subscribers = []
        for topic in self.depth_topics:
            subscriber = message_filters.Subscriber(topic, Image)
            subscribers.append(subscriber)

        self.sync = message_filters.ApproximateTimeSynchronizer(subscribers, queue_size=10, slop=0.1)
        self.sync.registerCallback(self.sensor_cb)


    def advertise_services(self):
        rospy.Service("reset_map", std_srvs.srv.Empty, self.reset)
        rospy.Service("toggle_integration", std_srvs.srv.SetBool, self.toggle)
        rospy.Service("get_scene_cloud", vgn.srv.GetSceneCloud, self.get_scene_cloud)
        rospy.Service("get_map_cloud", vgn.srv.GetMapCloud, self.get_map_cloud)

    def reset(self, req):
        self.tsdf = UniformTSDFVolume(self.length, self.resolution)
        return std_srvs.srv.EmptyResponse()

    def toggle(self, req):
        # update transforms
        self.integrate = req.data
        return std_srvs.srv.SetBoolResponse(success=True)

    def sensor_cb(self, *msgs):
        if self.integrate:
            for msg in msgs:
                depth = (
                    self.cv_bridge.imgmsg_to_cv2(msg).astype(np.float32)
                    * self.depth_scaling
                )
                image_frame_id = msg.header.frame_id
                extrinsic = tf.lookup(
                    image_frame_id, self.frame_id, msg.header.stamp, rospy.Duration(0.1)
                )
                self.tsdf.integrate(depth, self.intrinsics[image_frame_id], extrinsic)

    def get_scene_cloud(self, req):
        scene_cloud = self.tsdf.get_scene_cloud()
        points = np.asarray(scene_cloud.points)
        msg = to_cloud_msg(self.frame_id, points)
        self.scene_cloud_pub.publish(msg)
        res = vgn.srv.GetSceneCloudResponse()
        res.scene_cloud = msg
        return res

    def get_map_cloud(self, req):
        map_cloud = self.tsdf.get_map_cloud()
        points = np.asarray(map_cloud.points)
        distances = np.asarray(map_cloud.colors)[:, [0]]
        msg = to_cloud_msg(self.frame_id, points, distances=distances)
        self.map_cloud_pub.publish(msg)
        res = vgn.srv.GetMapCloudResponse()
        res.voxel_size = self.tsdf.voxel_size
        res.map_cloud = msg
        return res


if __name__ == "__main__":
    rospy.init_node("tsdf_server")
    UniformTSDFServer()
    rospy.spin()
