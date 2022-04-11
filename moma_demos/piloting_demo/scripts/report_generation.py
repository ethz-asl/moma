#! /usr/bin/python

from cmath import inf
import os
import cv2
import csv
import numpy as np

import tf2_py as tf2
import rospy
import json
import uuid
import rosbag
from tqdm import tqdm
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, Quaternion

from moma_mission.utils.transforms import tf_to_se3

# static configuration
MAP_FRAME = "tracking_camera_odom"
VALVE_FRAME = "valve_gt"
IMAGE_TOPIC = "/versavis/cam0/image_raw_throttle"
IMAGES_DELTA_TIME = 5  # take a picture each 10 seconds
BASE_LINK_FRAME = "base_link"
ODOM_TOPIC = "/camera/odom/sample"
CAMERA_UUID = str(uuid.uuid4())
CAMERA_FOV = {  # https://www.intelrealsense.com/depth-camera-d435i/
    "height": 69.0,
    "vertical": 42.0,
}
CAMERA_IMAGE_SIZE = {"width": 1280, "height": 720}
SENSORS_LIST = {
    "rslidar": ["16 Beans Lidar Sensor", str(uuid.uuid4())],
    "realsense_t265": ["Realsense Tracking Camera", str(uuid.uuid4())],
    "imu": ["XSense Imu", str(uuid.uuid4())],
}
WRENCH_TOPIC = "/panda/franka_state_controller/F_ext"


class ReportGenerator:
    def __init__(self, bag_path, report_base_dir):
        self.startDate = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.endDate = self.startDate
        self.robot_uuid = str(uuid.uuid4())
        self.sync_id = 1667993927
        self.task_uuid = str(uuid.uuid4())
        self.inspection_plan_uuid = str(uuid.uuid4())
        self.inspection_task_uuids = [str(uuid.uuid4())]
        self.inspection_type = "visual"  # visual, contact, TBD
        self.map_file = "map.pcd"

        # open the bag
        print("[Report Generation]: Opening rosbag at {}".format(bag_path))
        self.bag = rosbag.Bag(bag_path, mode="r")

        self.report_dir = os.path.join(
            report_base_dir, self.startDate + "_inspection_plan"
        )

        print("[Report Generation]: Creating folder structure.")
        os.makedirs(self.report_dir, exist_ok=True)

        self.tf_tree, self.tf_times = self.__init_tf_tree()

    def __init_tf_tree(self):
        """Fills up a tf tree from a rosbag"""
        print("[Report Generation]: Reading tf info...")
        tf_tree = tf2.BufferCore(rospy.Duration(360000.0))
        tf_topics = ["/tf", "/tf_static"]
        tf_msgs_count = self.bag.get_message_count(tf_topics)
        print(
            "[Report Generation]: Reading {} transforms from the bag".format(
                tf_msgs_count
            )
        )

        tf_times = []
        with tqdm(total=tf_msgs_count) as progress_bar:
            for topic, message, t in self.bag.read_messages(topics=tf_topics):
                tf_times.append(t)
                for tf_message in message.transforms:
                    if topic == "/tf_static":
                        tf_tree.set_transform_static(tf_message, topic)
                    else:
                        tf_tree.set_transform(tf_message, topic)
            progress_bar.update(1)
        return tf_tree, tf_times

    # Utilities
    def get_files(self):
        return {
            "pictures_metadata": "pictures_metadata.csv",
            "pictures_folder": "pictures",
            "telemetry_data": "localization_telemetry.csv",
            "haptic_data": "haptic_sensing.csv",
        }

    def get_sensors(self):
        sensors = []
        for sensor_frame, sensor_info in SENSORS_LIST.items():
            sensor_entry = {}
            sensor_entry["name"] = sensor_frame
            sensor_entry["uuid"] = sensor_info[1]
            sensor_entry["description"] = sensor_info[0]
            sensor_entry["tf"] = {
                "translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0},
            }
            try:
                tf_transform = self.tf_tree.lookup_transform_core(
                    target_frame=BASE_LINK_FRAME,
                    source_frame=sensor_frame,
                    time=rospy.Time(0),
                )
                sensor_entry["tf"]["translation"][
                    "x"
                ] = tf_transform.transform.translation.x
                sensor_entry["tf"]["translation"][
                    "y"
                ] = tf_transform.transform.translation.y
                sensor_entry["tf"]["translation"][
                    "z"
                ] = tf_transform.transform.translation.z
                sensor_entry["tf"]["rotation"]["x"] = tf_transform.transform.rotation.x
                sensor_entry["tf"]["rotation"]["y"] = tf_transform.transform.rotation.y
                sensor_entry["tf"]["rotation"]["z"] = tf_transform.transform.rotation.z
                sensor_entry["tf"]["rotation"]["w"] = tf_transform.transform.rotation.w
            except Exception as exc:
                print(exc)
                print(
                    "[Report Generation]: Failed to get transform from {} to {}".format(
                        sensor_frame, BASE_LINK_FRAME
                    )
                )
            sensors.append(sensor_entry)
        return sensors

    def extract_config(self):
        print("[Report Generation]: Writing top level config file.")
        config_file = os.path.join(self.report_dir, "config.json")

        report_config = {
            "startDate": self.startDate,
            "endDate": self.endDate,
            "syncId": self.sync_id,
            "uuid": self.robot_uuid,
            "inspectionPlanUuid": self.inspection_plan_uuid,
            "insepctionTaskUuids": self.inspection_task_uuids,
            "type": self.inspection_type,
            "map": self.map_file,
            "files": self.get_files(),
            "sensors": self.get_sensors(),
            "pictures": {
                "folder": "pictures",
                "sensor_uuid": CAMERA_UUID,
                "format": "jpg",
                "size": CAMERA_IMAGE_SIZE,
                "fov": CAMERA_FOV,
            },
        }

        with open(config_file, "w") as outfile:
            json.dump(report_config, outfile, indent=2)

    def extract_telemetry(self):
        print("[Report Generation]: Extracting telemetry information.")
        telemetry_csv_file = os.path.join(self.report_dir, "localization_telemetry.csv")
        with open(telemetry_csv_file, "w") as f:
            writer = csv.writer(f)
            for t in self.tf_times:
                try:
                    tf_transform = self.tf_tree.lookup_transform_core(
                        target_frame=MAP_FRAME, source_frame=BASE_LINK_FRAME, time=t
                    )
                    translation = [
                        tf_transform.transform.translation.x,
                        tf_transform.transform.translation.y,
                        tf_transform.transform.translation.z,
                    ]
                    rotation = [
                        tf_transform.transform.rotation.x,
                        tf_transform.transform.rotation.y,
                        tf_transform.transform.rotation.z,
                        tf_transform.transform.rotation.w,
                    ]
                    entry = [t.to_sec(), self.task_uuid] + translation + rotation
                    writer.writerow(entry)
                except Exception as exc:
                    print(exc)

    def extract_odometry(self):
        print("[Report Generation]: Extracting odometry information.")
        telemetry_csv_file = os.path.join(self.report_dir, "localization_telemetry.csv")
        odom_msgs_count = self.bag.get_message_count(ODOM_TOPIC)

        with tqdm(total=odom_msgs_count) as progress_bar:
            with open(telemetry_csv_file, "w") as f:
                writer = csv.writer(f)
                for topic, message, t in self.bag.read_messages(topics=ODOM_TOPIC):
                    translation = [
                        message.pose.pose.position.x,
                        message.pose.pose.position.y,
                        message.pose.pose.position.z,
                    ]
                    rotation = [
                        message.pose.pose.orientation.x,
                        message.pose.pose.orientation.y,
                        message.pose.pose.orientation.z,
                        message.pose.pose.orientation.w,
                    ]
                    entry = [t.to_sec(), self.task_uuid] + translation + rotation
                    writer.writerow(entry)
                    progress_bar.update(1)

    def extract_pictures(self):
        print("[Report Generation]: Extracting pictures and saving metadata.")
        pictures_folder = os.path.join(self.report_dir, "pictures")
        pictures_csv_file = os.path.join(self.report_dir, "pictures_metadata.csv")
        os.makedirs(pictures_folder, exist_ok=True)

        bridge = CvBridge()
        t_prev = -inf
        img_idx = 0
        with open(pictures_csv_file, "w") as f:
            writer = csv.writer(f)
            for topic, message, t in self.bag.read_messages(topics=IMAGE_TOPIC):
                if (t.to_sec() - t_prev) > IMAGES_DELTA_TIME:
                    t_prev = t.to_sec()

                    cam_translation = [0.0, 0.0, 0.0]
                    cam_rotation = [0.0, 0.0, 0.0, 1.0]

                    try:
                        tf_transform = self.tf_tree.lookup_transform_core(
                            target_frame="odom",
                            source_frame=message.header.frame_id,
                            time=rospy.Time(0),
                        )
                        cam_translation = [
                            tf_transform.transform.translation.x,
                            tf_transform.transform.translation.y,
                            tf_transform.transform.translation.z,
                        ]
                        cam_rotation = [
                            tf_transform.transform.rotation.x,
                            tf_transform.transform.rotation.y,
                            tf_transform.transform.rotation.z,
                            tf_transform.transform.rotation.w,
                        ]
                    except Exception as exc:
                        pass

                    # object info
                    obj_type = ["none"]
                    obj_position = [0.0, 0.0, 0.0]
                    obj_rotation = [0.0, 0.0, 0.0, 1.0]
                    obj_info = [0.0, 0.0, 0.0]
                    meta = (
                        ["DSC{:06d}.jpg".format(img_idx), t_prev, self.task_uuid]
                        + cam_translation
                        + cam_rotation
                        + obj_type
                        + obj_position
                        + obj_rotation
                        + obj_info
                    )
                    writer.writerow(meta)
                    img_idx += 1

                    # Save the image as well
                    cv2_img = bridge.imgmsg_to_cv2(message, "rgb8")
                    cv2.imwrite(
                        os.path.join(self.report_dir, "pictures", meta[0]), cv2_img
                    )

    def extract_wrench(self):
        print("[Report Generation]: Extracting haptic wrench information.")
        haptic_csv_file = os.path.join(self.report_dir, "haptic_sensing.csv")
        with open(haptic_csv_file, "w") as f:
            writer = csv.writer(f)
            for topic, message, t in self.bag.read_messages(topics=WRENCH_TOPIC):
                valve_transform = tf_to_se3(
                    self.tf_tree.lookup_transform_core(
                        target_frame=VALVE_FRAME,
                        source_frame=MAP_FRAME,
                        time=t,
                    )
                )
                obj_position = valve_transform.translation
                rotation_axis = valve_transform.rotation[:, 2]
                angle = [0.0]
                T_v_ee = tf_to_se3(
                    self.tf_tree.lookup_transform_core(
                        target_frame=VALVE_FRAME,
                        source_frame=message.header.frame_id,
                        time=t,
                    )
                )
                torque = message.wrench.force.x * np.linalg.norm(T_v_ee.translation)
                entry = (
                    [t.to_sec(), self.task_uuid]
                    + obj_position
                    + rotation_axis
                    + angle
                    + torque
                )
                writer.writerow(entry)

    def run(self):
        self.extract_config()
        self.extract_pictures()
        # extract_telemetry(bag, report_dir) this uses tf, available when localizing against map
        self.extract_odometry()
        self.extract_wrench()


if __name__ == "__main__":
    print("[Report Generation]: Generating sample report")
    report_generator = ReportGenerator(
        bag_path="/home/giuseppe/storage/bags/2022-02-15/2022-02-15-13-17-11_smb_sensors.bag",
        report_base_dir="/home/giuseppe/storage",
    )
    report_generator.run()
