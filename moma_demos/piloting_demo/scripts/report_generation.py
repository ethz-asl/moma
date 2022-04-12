#! /usr/bin/python

from cmath import inf
import os
import cv2
import csv
import numpy as np
import argparse

import tf2_py as tf2
import rospy
import json
import uuid
import rosbag
from tqdm import tqdm
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError

from moma_mission.utils.transforms import tf_to_se3

# static configuration
MAP_FRAME = "map"  # For local navigation: "tracking_camera_odom"
OBJECT_TYPE = "valve"
OBJECT_FRAME = "valve_wheel_center"
IMAGE_TOPIC = "/object_keypoints_ros/result_img"
IMAGES_DELTA_TIME = 5  # take a picture each 10 seconds
BASE_LINK_FRAME = "base_link"
ODOM_TOPIC = "/base_odom"
ROBOT_UUID = "4558a82a-1e3d-4c40-8938-5c9281e57314"
CAMERA_FOV = {  # https://www.intelrealsense.com/depth-camera-d435i/
    "height": 69.0,
    "vertical": 42.0,
}
CAMERA_IMAGE_SIZE = {"width": 1280, "height": 720}
SENSORS_LIST = {
    "rslidar": [
        "16 Beans Lidar Sensor",
        "148ddf1c-da28-4d33-b85b-d3ddc445e58a",
        "rslidar",
    ],
    "realsense_t265": [
        "Realsense Tracking Camera",
        "8de3129f-7a0c-4f84-8a34-77f0a1eebd31",
        "realsense_t265",
    ],
    "imu": ["XSense Imu", "4902be61-2dfe-4140-b3c5-ed9a49e32a91", "imu"],
    "realsense_d435i": [
        "RealSense D435i Depth Camera",
        "47255b80-561e-41b6-bbef-2916da6426e6",
        "hand_eye_color_optical_frame",
    ],
}
CAMERA_UUID = SENSORS_LIST["realsense_d435i"][1]
WRENCH_TOPIC = "/panda/franka_state_controller/F_ext"
JOINT_STATES_TOPIC = "/joint_states"
JOINT_FINGER_NAME = "panda_finger_joint1"
GRIPPER_OPEN_THRESHOLD = 0.03


class ReportGenerator:
    def __init__(self, bag_path, report_base_dir):
        # open the bag
        print("[Report Generation]: Opening rosbag at {}".format(bag_path))
        self.bag = rosbag.Bag(bag_path, mode="r")

        self.startDate = datetime.fromtimestamp(self.bag.get_start_time()).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )
        self.endDate = datetime.fromtimestamp(self.bag.get_end_time()).strftime(
            "%Y-%m-%d_%H-%M-%S"
        )
        self.robot_uuid = ROBOT_UUID
        self.sync_id = 1667993927
        self.task_uuid = str(uuid.uuid4())
        self.inspection_plan_uuid = str(uuid.uuid4())
        self.inspection_task_uuids = [str(uuid.uuid4())]
        self.inspection_type = "visual"  # visual, contact, TBD
        self.map_file = "map.pcd"

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
        for sensor_name, (
            sensor_description,
            sensor_uuid,
            sensor_frame,
        ) in SENSORS_LIST.items():
            sensor_entry = {}
            sensor_entry["name"] = sensor_name
            sensor_entry["uuid"] = sensor_uuid
            sensor_entry["description"] = sensor_description
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
            writer.writerow(
                ["stamp", "task_uuid", "p_x", "p_y", "p_z", "q_x", "q_y", "q_z", "q_w"]
            )
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
                writer.writerow(
                    [
                        "stamp",
                        "task_uuid",
                        "p_x",
                        "p_y",
                        "p_z",
                        "q_x",
                        "q_y",
                        "q_z",
                        "q_w",
                    ]
                )
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
            writer.writerow(
                [
                    "file_name",
                    "stamp",
                    "task_uuid",
                    "cam_pos_x",
                    "cam_pos_y",
                    "cam_pos_z",
                    "cam_rot_x",
                    "cam_rot_y",
                    "cam_rot_z",
                    "cam_rot_w",
                    "obj_type",
                    "obj_pos_x",
                    "obj_pos_y",
                    "obj_pos_z",
                    "obj_rot_x",
                    "obj_rot_y",
                    "obj_rot_z",
                    "obj_rot_w",
                    "obj_info_0",
                    "obj_info_1",
                    "obj_info_2",
                ]
            )
            for topic, message, t in self.bag.read_messages(topics=IMAGE_TOPIC):
                if (t.to_sec() - t_prev) > IMAGES_DELTA_TIME:
                    t_prev = t.to_sec()

                    cam_translation = [0.0, 0.0, 0.0]
                    cam_rotation = [0.0, 0.0, 0.0, 1.0]

                    try:
                        tf_transform = self.tf_tree.lookup_transform_core(
                            target_frame=MAP_FRAME,
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
                    except tf2.LookupException:
                        print(
                            f"[extract_pictures] At time {t} the camera was not localized yet"
                        )

                    # object info
                    obj_type = [OBJECT_TYPE]
                    obj_position = [0.0, 0.0, 0.0]
                    obj_rotation = [0.0, 0.0, 0.0, 1.0]
                    obj_info = [0.0, 0.0, 0.0]

                    try:
                        obj_transform = self.tf_tree.lookup_transform_core(
                            target_frame=OBJECT_FRAME,
                            source_frame=MAP_FRAME,
                            time=t,
                        )
                        obj_position = [
                            obj_transform.transform.translation.x,
                            obj_transform.transform.translation.y,
                            obj_transform.transform.translation.z,
                        ]
                        obj_rotation = [
                            obj_transform.transform.rotation.x,
                            obj_transform.transform.rotation.y,
                            obj_transform.transform.rotation.z,
                            obj_transform.transform.rotation.w,
                        ]
                    except tf2.LookupException:
                        print(
                            f"[extract_pictures] At time {t} the object was not localized yet"
                        )

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
            writer.writerow(
                [
                    "stamp",
                    "task_uuid",
                    "obj_position_x",
                    "obj_position_y",
                    "obj_position_z",
                    "rotation_axis_x",
                    "rotation_axis_y",
                    "rotation_axis_z",
                    "angle",
                    "torque",
                ]
            )
            gripper_closed = False
            for topic, msg, t in self.bag.read_messages(
                topics=[WRENCH_TOPIC, JOINT_STATES_TOPIC]
            ):
                if topic == WRENCH_TOPIC:
                    try:
                        obj_transform = tf_to_se3(
                            self.tf_tree.lookup_transform_core(
                                target_frame=OBJECT_FRAME,
                                source_frame=MAP_FRAME,
                                time=t,
                            )
                        )
                        obj_position = obj_transform.translation
                        rotation_axis = obj_transform.rotation[:, 2]
                        angle = 0.0
                        T_v_ee = tf_to_se3(
                            self.tf_tree.lookup_transform_core(
                                target_frame=OBJECT_FRAME,
                                source_frame=msg.header.frame_id,
                                time=t,
                            )
                        )
                        torque = (
                            msg.wrench.force.x * np.linalg.norm(T_v_ee.translation)
                            if gripper_closed
                            else 0.0
                        )
                        entry = (
                            [t.to_sec(), self.task_uuid]
                            + list(obj_position)
                            + list(rotation_axis)
                            + [angle]
                            + [torque]
                        )
                        writer.writerow(entry)
                    except tf2.ExtrapolationException:
                        print(
                            f"[extract_wrench] Skipping time {t} as there is no valid tf"
                        )
                    except tf2.LookupException:
                        print(
                            f"[extract_wrench] Skipping time {t} as the object was not detected yet"
                        )
                if topic == JOINT_STATES_TOPIC:
                    finger_joint = msg.name.index(JOINT_FINGER_NAME)
                    finger_state = msg.position[finger_joint]
                    gripper_closed = finger_state < GRIPPER_OPEN_THRESHOLD

    def run(self):
        self.extract_config()
        self.extract_pictures()
        # extract_telemetry(bag, report_dir) this uses tf, available when localizing against map
        self.extract_odometry()
        self.extract_wrench()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generates a report.")
    parser.add_argument("bag_path", help="Path to the bag file")
    parser.add_argument(
        "report_base_dir", help="Directory in which the report should be stored"
    )
    args = parser.parse_args()

    print("[Report Generation]: Generating sample report")
    report_generator = ReportGenerator(
        bag_path=args.bag_path,
        report_base_dir=args.report_base_dir,
    )
    report_generator.run()
