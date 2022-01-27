#!/usr/bin/env python3
import copy
import rospy
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from moma_mission.srv import KeypointsPerception, KeypointsPerceptionResponse, KeypointsPerceptionRequest
from object_keypoints_ros.msg import Keypoint, Keypoints
from cv_bridge import CvBridge

from object_keypoints_ros.srv import KeypointsDetection, KeypointsDetectionRequest, KeypointsDetectionResponse
from moma_mission.missions.piloting.valve_fitting import Camera, ValveFitter, ValveModel, FeatureMatcher

# TODO placing here all the params for quick testing, then either in yaml or rosparam
NUM_SPOKES = 3
NUM_KEYPOINTS = 4
RESOLUTION = [1280, 760]
BASE_FRAME = "panda_link0"
CAMERA_FRAME = "hand_eye_color_optical_frame"
INTRINSICS = np.array([[923.01777, 0.0, 626.94979], 
                       [0.0, 921.45109, 364.38512],
                       [0.0, 0.0, 1.0]])
DISTORTION = np.array([923.01777, 921.45109, 626.94979, 364.38512])
DEPTH_WDW_SIZE = 5

class PerceptionModule:
    def __init__(self):
        self.valve_fitter = ValveFitter(NUM_SPOKES)
        self.cameras = []
        self.keypoints_observations =  []
        self.previous_obs_time = -np.inf
        self.bridge = CvBridge()

        # TODO: to replace with config from file
        self.K = INTRINSICS
        self.D = DISTORTION
        self.R = RESOLUTION

        # joints velocity
        self.total_velocity = np.inf
        self.total_velocity_count = 0
        self.still = False
        self.triangulated = False

        # Images
        self.depth = None
        self.depth_header = None
        self.rgb = None

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ROS
        self.marker_pub = rospy.Publisher("/perception/triangulated_keypoints", MarkerArray, queue_size=1)
        self.keypoints_sub = rospy.Subscriber("/object_keypoints_ros/keypoints", Keypoints, self._keypoints_callback, queue_size=1)
        self.image_sub = rospy.Subscriber("/hand_eye/color/image_raw_throttle", Image, self._image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("/hand_eye/depth/image_rect_raw_throttle", Image, self._depth_callback, queue_size=1)
        self.detection_srv_client = rospy.ServiceProxy("/object_keypoints_ros/detect", KeypointsDetection)
        self.trigger_detectio_srv = rospy.Service("/perception/perceive", KeypointsPerception, self._perception_cb)


    def _make_marker(self, id, frame_id, x, y, z):
        marker = Marker()
        marker.id = id
        marker.header.frame_id = frame_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y 
        marker.pose.position.z = z 
        return marker

    def _depth_callback(self, msg):
        MM_2_METER_FACTOR = 0.001 #this is the only multiplier that seems to have sense
        self.depth_header = msg.header
        self.depth = MM_2_METER_FACTOR * self.bridge.imgmsg_to_cv2(msg, msg.encoding)   

    def _image_callback(self, msg):
        self.rgb = msg

    def _perception_cb(self, req: KeypointsPerceptionRequest):
        res = KeypointsPerceptionResponse()
        res.keypoints = self.perceive()
        return res
    
    def perceive(self):
        """
        Uses a keypoint detection service and depth information to perceive 3D points
        :return: 3D keypoint poses
        """
        try:
            self.detection_srv_client.wait_for_service(timeout=3)
        except rospy.ROSException as exc:
            rospy.logwarn("Service {} not available yet".format(self.detection_srv_client.resolved_name))
            return

        req = KeypointsDetectionRequest()
        if self.depth is None:
            rospy.logwarn("No depth image received yet, skipping detection")
            return
        req_depth = copy.deepcopy(self.depth) # store locally the current depth image as well
        if self.rgb is None:
            rospy.logwarn("No rgb image received yet, skipping detection")
            return
        req.rgb = self.rgb

        res : KeypointsDetectionResponse
        res = self.detection_srv_client.call(req)
        n_kpts = len(res.detection.keypoints) 
        if n_kpts != NUM_KEYPOINTS:
            rospy.logwarn("Not enough keypoints. Returned only {}/{}".format(n_kpts, NUM_KEYPOINTS))
            return
        
        marker_array = MarkerArray()
        pose_array = PoseArray()

        # let it fail if not found
        trans = self.tf_buffer.lookup_transform(BASE_FRAME, CAMERA_FRAME, self.depth_header.stamp, timeout=rospy.Duration(3.0))
        t_w_cam = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        q = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        R_w_cam = Rotation.from_quat(q).as_matrix()

        for i, kpt in enumerate(res.detection.keypoints):
            print("Processing keypoint {} {}".format(kpt.x, kpt.y))

            # take a window of depths and extract the median to remove outliers
            x_min, x_max = max(int(kpt.x) - DEPTH_WDW_SIZE, 0), min(int(kpt.x) + DEPTH_WDW_SIZE, RESOLUTION[0]-1)
            y_min, y_max = max(int(kpt.y) - DEPTH_WDW_SIZE, 0), min(int(kpt.y) + DEPTH_WDW_SIZE, RESOLUTION[1]-1)

            # # take depth window
            depth_wdw = req_depth[y_min: y_max, x_min: x_max]
            depth_wdw = np.reshape(depth_wdw, (-1,))
            depth_wdw = depth_wdw[depth_wdw>0]  # threshold invalid values
            depth_wdw = depth_wdw[depth_wdw<5]  # threshold far away values

            P_cam = np.zeros((3,))
            P_cam[2] = np.median(depth_wdw)
            
            # retrieve the global x, y coordinate of the point given z 
            # TODO not accounting for distortion --> would be better to just operate on undistorted images
            P_cam[0] = (kpt.x - self.K[0,2]) * P_cam[2] / self.K[0, 0]
            P_cam[1] = (kpt.y - self.K[1,2]) * P_cam[2] / self.K[1, 1]
            
            P_base = R_w_cam @ P_cam + t_w_cam 
            marker = self._make_marker(i, BASE_FRAME, P_base[0], P_base[1], P_base[2])
            marker_array.markers.append(marker)
            pose_array.poses.append(marker.pose)
            print("Keypoint at {}".format(P_cam))
            
        self.marker_pub.publish(marker_array)
        return pose_array
            
        

    def _keypoints_callback(self, msg: Keypoints):
        """ 
        Should try to fit keypoints to the valve model. Unused for the moment being... 
        Do not use it for now
        """
        rospy.logwarn_once("_keypoints_callback unused")
        return

        # assert that the robot is not moving
        if len(self.keypoints_observations) == 2 and not self.triangulated:
            rospy.loginfo("Collected 2 keypoints... estimating pose")
    
            matcher = FeatureMatcher()
            matches = matcher.match(self.cameras[0], self.cameras[1], 
                                    self.keypoints_observations[0], self.keypoints_observations[1])

            # reshuffle keypoints to match
            # map 0, 1, 2, 3 to their location in the original image
            self.keypoints_observations[1][matches.astype(int), :] = self.keypoints_observations[1][[0, 1, 2, 3], :]

            self.valve_fitter.add_observation(self.cameras[0], self.keypoints_observations[0], weights=np.ones((4,)))
            self.valve_fitter.add_observation(self.cameras[0], self.keypoints_observations[0], weights=np.ones((4,)))
            valve = self.valve_fitter.optimize(method='triangulation')
            
            marker_array = MarkerArray()
            marker_array.frame_id = "panda_link0"
            rospy.loginfo("Valve triangulated")
            rospy.loginfo("Keypoints are:\n{}".format(valve.keypoints))

            for i in range(valve.keypoints.shape[1]):
                marker = self._make_marker(i, valve.keypoints[0, i], valve.keypoints[1, i], valve.keypoints[2, i])
                marker_array.markers.append(marker)
            self.marker_pub.publish(marker_array)
            self.triangulated = True
        
        if self.still and (msg.header.stamp.to_sec() - self.previous_obs_time) > 10.0 :
            rospy.loginfo("Robot is still, taking one observation!")
            self.previous_obs_time = msg.header.stamp.to_sec()
        else:
            return

        n_kpts = len(msg.keypoints)
        keypoints = np.zeros((n_kpts, 2))
        for i, kpt in enumerate(msg.keypoints):
            keypoints[i, 0] = kpt.x
            keypoints[i, 1] = kpt.y

        # get the camera transform at the detection time step, fail if cannot find trans
        T = np.eye(4)
        # TODO(giuseppe) remove hard coded transforms
        trans = self.tf_buffer.lookup_transform("panda_link0", "hand_eye_color_optical_frame", msg.header.stamp, timeout=rospy.Duration(3.0))
        T[:3, 0] = trans.transform.translation.x
        T[:3, 1] = trans.transform.translation.y
        T[:3, 2] = trans.transform.translation.z
        q = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        rot = Rotation.from_quat(q).as_matrix()
        T[:3, :3] = rot

        camera = Camera(K=self.K, D=self.D, resolution=self.R)
        camera.set_transform(T)

        rospy.loginfo("Adding new keypoints set\n{}".format(keypoints))
        self.keypoints_observations.append(keypoints)
        self.cameras.append(camera)


if __name__ == "__main__":
    rospy.init_node("perception")
    perception = PerceptionModule()
    rospy.spin()
