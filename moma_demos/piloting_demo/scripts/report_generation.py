#!/usr/bin/python3

from cmath import inf
import os
import cv2
import csv
import numpy as np
import argparse

import shutil
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
PLAN_UUID_TOPIC = "/plan_uuid"
TASK_UUID_TOPIC = "/task_uuid"
SYNC_ID_TOPIC = "/sync_id"
MAP_FRAME = "map"  # For local navigation: "tracking_camera_odom"
OBJECT_TYPE = "valve"
OBJECT_FRAME = "valve_wheel_center"
OBJECT_IMAGE_TOPIC = "/object_keypoints_ros/result_img"
IMAGE_TOPIC = "/image"
IMAGES_DELTA_TIME = 0  # take a picture every ... seconds
BASE_LINK_FRAME = "base_link"
ODOM_TOPIC = "/mavsdk_ros/local_position"
ROBOT_UUID = "f09df66e-cc30-4e11-92e8-fa2f5d7d19e5"
CAMERA_FOV = {  # https://www.intelrealsense.com/depth-camera-d435i/
    "horizontal": 69.0,
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
        "hand_eye_color_frame",
    ],
}
CAMERA_UUID = SENSORS_LIST["realsense_d435i"][1]
WRENCH_TOPIC = "/panda/franka_state_controller/F_ext"
OBJECT_PATH_INVERTED_TOPIC = "/valve_path_inverted"
OBJECT_ANGLE_TOPIC = "/valve_angle"
JOINT_STATES_TOPIC = "/joint_states"
JOINT_FINGER_NAME = "panda_finger_joint1"
GRIPPER_OPEN_THRESHOLD = 0.03
DATE_FORMAT = "%Y_%m_%d-%H_%M_%S"


class ReportGenerator:
    def __init__(self, bag_path, report_base_dir):
        # open the bag
        print("[Report Generation]: Opening rosbag at {}".format(bag_path))
        self.bag = rosbag.Bag(bag_path, mode="r")

        self.startDate = datetime.fromtimestamp(self.bag.get_start_time()).strftime(
            DATE_FORMAT
        )
        self.endDate = datetime.fromtimestamp(self.bag.get_end_time()).strftime(
            DATE_FORMAT
        )
        self.robot_uuid = ROBOT_UUID
        self.plan_uuid = None
        self.task_uuids = {}
        self.sync_id = None
        self.inspection_type = "visual"  # visual, contact, TBD
        self.map_file = "map.pcd"

        self.report_dir = os.path.join(
            report_base_dir, self.startDate + "_inspection_plan"
        )

        print("[Report Generation]: Creating folder structure.")
        os.makedirs(self.report_dir, exist_ok=True)

        self.objects = None

        self.__init_tf_tree()
        self.__init_uuids()

    def __init_tf_tree(self):
        """Fills up a tf tree from a rosbag"""
        print("[Report Generation]: Reading tf info...")
        self.tf_tree = tf2.BufferCore(rospy.Duration(360000.0))
        tf_topics = ["/tf", "/tf_static"]
        tf_msgs_count = self.bag.get_message_count(tf_topics)
        print(
            "[Report Generation]: Reading {} transforms from the bag".format(
                tf_msgs_count
            )
        )

        self.tf_times = []
        self.object_times = []
        previous_object_time = rospy.Time(0)
        with tqdm(total=tf_msgs_count) as progress_bar:
            for topic, message, t in self.bag.read_messages(topics=tf_topics):
                self.tf_times.append(t)
                if (
                    len(
                        [
                            tf
                            for tf in message.transforms
                            if tf.child_frame_id == OBJECT_FRAME
                        ]
                    )
                    > 0
                ):
                    # BUG the static transform for the same object may appear multiple times in the tf_msgs
                    if t - previous_object_time > rospy.Duration(1):
                        self.object_times.append(t)
                        previous_object_time = t
                    else:
                        print(
                            f"[Report Generation]: Skipping object at time {t} as it is likely a duplicate"
                        )
                for tf_message in message.transforms:
                    if topic == "/tf_static":
                        self.tf_tree.set_transform_static(tf_message, topic)
                    else:
                        self.tf_tree.set_transform(tf_message, topic)
            progress_bar.update(1)

    # Utilities
    @property
    def files(self):
        return {
            "config": "config.json",
            "pictures_metadata": "pictures_metadata.csv",
            "pictures_folder": "pictures",
            "telemetry_data": "localization_telemetry.csv",
            "haptic_data": "haptic_sensing.csv",
            "objects": "objects.csv",
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

    def __init_uuids(self):
        print("[Report Generation]: Reading UUIDs.")
        for topic, message, t in self.bag.read_messages(
            topics=[PLAN_UUID_TOPIC, TASK_UUID_TOPIC, SYNC_ID_TOPIC]
        ):
            if topic == PLAN_UUID_TOPIC:
                if self.plan_uuid is None:
                    self.plan_uuid = message.data
                elif self.plan_uuid != message.data:
                    raise Exception(
                        "Found multiple differing plan uuids in the same bag file"
                    )

            if topic == TASK_UUID_TOPIC:
                if message.data not in self.task_uuids.keys():
                    # Map task uuids to starting times
                    self.task_uuids[message.data] = t

            if topic == SYNC_ID_TOPIC:
                if self.sync_id is None:
                    self.sync_id = message.data
                elif self.sync_id != message.data:
                    raise Exception(
                        "Found multiple differing sync ids in the same bag file"
                    )

    def get_task_uuid(self, time, allow_empty=False):
        """Get the task uuid for the given time"""
        matching_task_uuid = None
        matching_task_uuid_time = rospy.Time(0)
        for task_uuid, task_uuid_time in self.task_uuids.items():
            if time >= task_uuid_time > matching_task_uuid_time:
                matching_task_uuid = task_uuid
                matching_task_uuid_time = task_uuid_time

        if matching_task_uuid is None and not allow_empty:
            raise Exception(f"No task uuid known at time {time}")

        return matching_task_uuid

    def get_obj_ref(self, time, allow_empty=False, next=False):
        """
        Get the object ref for the given time or before, but within the same task_uuid as the given time

        :param next: Get the next object ref after the given time, but within the same task_uuid as the given time
        """
        matching_obj_ref = None
        matching_obj_time = (
            rospy.Time(0) if not next else rospy.Time(self.bag.get_end_time())
        )
        for obj_ref, obj in self.objects.items():
            if (not next and (time >= obj["time"] > matching_obj_time)) or (
                next and (time <= obj["time"] < matching_obj_time)
            ):
                matching_obj_ref = obj_ref
                matching_obj_time = obj["time"]

        if matching_obj_ref is None:
            if allow_empty:
                return None
            else:
                raise Exception(f"No object ref known at time {time}")

        # Check that object was already detected at the time
        # and that it is not the object from the previous task
        # we are returning here
        if (
            self.get_task_uuid(time, allow_empty)
            != self.objects[matching_obj_ref]["task_uuid"]
        ):
            if allow_empty:
                return None
            else:
                raise Exception(
                    f"Object known at time {time} is still a dangling leftover from the previous task, no object known yet that corresponds to the current task"
                )

        return matching_obj_ref

    def extract_config(self):
        print("[Report Generation]: Writing top level config file.")
        config_file = os.path.join(self.report_dir, self.files["config"])

        report_config = {
            "startDate": self.startDate,
            "endDate": self.endDate,
            "syncId": self.sync_id,
            "uuid": self.robot_uuid,
            "inspectionPlanUuid": self.plan_uuid,
            "inspectionTaskUuids": list(self.task_uuids.keys()),
            "type": self.inspection_type,
            "map": self.map_file,
            "files": self.files,
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

    def extract_objects(self):
        print("[Report Generation]: Extracting object information.")
        objects_csv_file = os.path.join(self.report_dir, self.files["objects"])
        with open(objects_csv_file, "w") as f:
            writer = csv.writer(f)
            # writer.writerow(
            #     [
            #         "stamp",
            #         "task_uuid",
            #         "obj_ref",
            #         "obj_type",
            #         "p_x",
            #         "p_y",
            #         "p_z",
            #         "q_x",
            #         "q_y",
            #         "q_z",
            #         "q_w",
            #         "rotation_axis_x",
            #         "rotation_axis_y",
            #         "rotation_axis_z",
            #         "obj_info_0",
            #         "obj_info_1",
            #         "obj_info_2",
            #     ]
            # )

            self.objects = {}

            # BUG read only at object_times
            # If we read all tf_times, the object appears already at the beginning of the bag (where it was not even detected)
            obj_ref = 0
            for t in self.object_times:
                # object info
                obj_ref += 1
                obj_type = OBJECT_TYPE
                obj_info = [0.0, 0.0, 0.0]

                try:
                    obj_transform = self.tf_tree.lookup_transform_core(
                        target_frame=MAP_FRAME,
                        source_frame=OBJECT_FRAME,
                        time=t,
                    )
                    obj_se3 = tf_to_se3(obj_transform)

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
                    obj_rotation_axis = obj_se3.rotation[:, 2]

                    task_uuid = self.get_task_uuid(t)
                    entry = (
                        [t.to_sec(), task_uuid, obj_ref, obj_type]
                        + obj_position
                        + obj_rotation
                        + list(obj_rotation_axis)
                        + obj_info
                    )
                    writer.writerow(entry)
                    self.objects[obj_ref] = {"time": t, "task_uuid": task_uuid}
                except tf2.ExtrapolationException:
                    print(
                        f"[extract_objects] Skipping time {t} as there is no valid tf"
                    )
                except tf2.LookupException:
                    print(
                        f"[extract_objects] Skipping time {t} as the object was not localized yet"
                    )

    # def extract_telemetry(self):
    #     print("[Report Generation]: Extracting telemetry information.")
    #     telemetry_csv_file = os.path.join(self.report_dir, self.files["telemetry_data"])
    #     with open(telemetry_csv_file, "w") as f:
    #         writer = csv.writer(f)
    #         # writer.writerow(
    #         #     ["stamp", "task_uuid", "p_x", "p_y", "p_z", "q_x", "q_y", "q_z", "q_w"]
    #         # )
    #         for t in self.tf_times:
    #             try:
    #                 tf_transform = self.tf_tree.lookup_transform_core(
    #                     target_frame=MAP_FRAME, source_frame=BASE_LINK_FRAME, time=t
    #                 )
    #                 translation = [
    #                     tf_transform.transform.translation.x,
    #                     tf_transform.transform.translation.y,
    #                     tf_transform.transform.translation.z,
    #                 ]
    #                 rotation = [
    #                     tf_transform.transform.rotation.x,
    #                     tf_transform.transform.rotation.y,
    #                     tf_transform.transform.rotation.z,
    #                     tf_transform.transform.rotation.w,
    #                 ]
    #                 entry = [t.to_sec(), self.get_task_uuid(t)] + translation + rotation
    #                 writer.writerow(entry)
    #             except Exception as exc:
    #                 print(exc)

    def extract_odometry(self):
        print("[Report Generation]: Extracting odometry information.")
        telemetry_csv_file = os.path.join(self.report_dir, self.files["telemetry_data"])
        odom_msgs_count = self.bag.get_message_count(ODOM_TOPIC)

        with tqdm(total=odom_msgs_count) as progress_bar:
            with open(telemetry_csv_file, "w") as f:
                writer = csv.writer(f)
                # writer.writerow(
                #     [
                #         "stamp",
                #         "task_uuid",
                #         "p_x",
                #         "p_y",
                #         "p_z",
                #         "q_x",
                #         "q_y",
                #         "q_z",
                #         "q_w",
                #     ]
                # )
                for topic, message, t in self.bag.read_messages(topics=ODOM_TOPIC):
                    task_uuid = self.get_task_uuid(t, True)
                    if task_uuid is None:
                        rospy.loginfo(
                            f"Skipping odometry at time {t} as there is no corresponding task running"
                        )
                        continue
                    assert message.header.frame_id == MAP_FRAME
                    assert message.child_frame_id == BASE_LINK_FRAME
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
                    entry = [t.to_sec(), task_uuid] + translation + rotation
                    writer.writerow(entry)
                    progress_bar.update(1)

    def extract_pictures(self):
        print("[Report Generation]: Extracting pictures and saving metadata.")
        pictures_folder = os.path.join(self.report_dir, self.files["pictures_folder"])
        pictures_csv_file = os.path.join(
            self.report_dir, self.files["pictures_metadata"]
        )
        os.makedirs(pictures_folder, exist_ok=True)

        bridge = CvBridge()
        t_prev = -inf
        with open(pictures_csv_file, "w") as f:
            writer = csv.writer(f)
            # writer.writerow(
            #     [
            #         "file_name",
            #         "stamp",
            #         "task_uuid",
            #         "cam_pos_x",
            #         "cam_pos_y",
            #         "cam_pos_z",
            #         "cam_rot_x",
            #         "cam_rot_y",
            #         "cam_rot_z",
            #         "cam_rot_w",
            #         "obj_ref",
            #     ]
            # )
            img_idx = 0
            for topic, message, t in self.bag.read_messages(
                topics=[IMAGE_TOPIC, OBJECT_IMAGE_TOPIC]
            ):
                if (t.to_sec() - t_prev) > IMAGES_DELTA_TIME:
                    t_prev = t.to_sec()

                    obj_ref = 0
                    if topic == OBJECT_IMAGE_TOPIC:
                        # Get the corresponding object of this image
                        # Note that it might also be a different object due to a failed perception
                        # This is why we take into account only the very last image before a successful perception
                        obj_ref = self.get_obj_ref(t, allow_empty=True, next=True)
                        # A failed perception at the end of the task
                        if obj_ref is None:
                            continue

                        # Keep only the last matching image for the given object
                        found_another_matching_image = False
                        for i, (_, _, inner_t) in enumerate(
                            self.bag.read_messages(
                                topics=OBJECT_IMAGE_TOPIC,
                                start_time=t,
                            )
                        ):
                            # Skip outer message (duplicate)
                            if i == 0:
                                continue
                            # Note that get_obj_ref also ensures that the object is within the same task_uuid
                            if (
                                self.get_obj_ref(inner_t, allow_empty=True, next=True)
                                == obj_ref
                            ):
                                found_another_matching_image = True
                                break
                        # If we found another matching image, it will be considered during the next iteration
                        if found_another_matching_image:
                            continue

                    cam_translation = [0.0, 0.0, 0.0]
                    cam_rotation = [0.0, 0.0, 0.0, 1.0]

                    try:
                        tf_transform = self.tf_tree.lookup_transform_core(
                            target_frame=MAP_FRAME,
                            source_frame=message.header.frame_id.replace(
                                "_optical", ""
                            ),
                            time=t,
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

                    meta = (
                        [
                            "DSC{:06d}.jpg".format(img_idx),
                            t.to_sec(),
                            self.get_task_uuid(t),
                        ]
                        + cam_translation
                        + cam_rotation
                        + [obj_ref]
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
        haptic_csv_file = os.path.join(self.report_dir, self.files["haptic_data"])
        with open(haptic_csv_file, "w") as f:
            writer = csv.writer(f)
            # writer.writerow(
            #     [
            #         "stamp",
            #         "task_uuid",
            #         "obj_ref",
            #         "angle",
            #         "torque",
            #     ]
            # )
            gripper_closed = False
            object_path_inverted = None
            object_angle = 0.0
            for topic, msg, t in self.bag.read_messages(
                topics=[
                    WRENCH_TOPIC,
                    JOINT_STATES_TOPIC,
                    OBJECT_PATH_INVERTED_TOPIC,
                    OBJECT_ANGLE_TOPIC,
                ]
            ):
                if topic == WRENCH_TOPIC:
                    task_uuid = self.get_task_uuid(t, True)
                    if task_uuid is None:
                        rospy.loginfo(
                            f"Skipping wrench measurement at time {t} as there is no corresponding task running"
                        )
                        continue

                    if gripper_closed:
                        obj_ref = self.get_obj_ref(t)

                        T_v_ee = tf_to_se3(
                            self.tf_tree.lookup_transform_core(
                                target_frame=OBJECT_FRAME,
                                source_frame=msg.header.frame_id,
                                time=t,
                            )
                        )

                        assert object_path_inverted is not None
                        torque = msg.wrench.force.x * np.linalg.norm(T_v_ee.translation)
                        torque = -torque if object_path_inverted else torque

                        entry = (
                            [t.to_sec(), task_uuid]
                            + [obj_ref]
                            + [object_angle]
                            + [torque]
                        )
                        writer.writerow(entry)
                if topic == JOINT_STATES_TOPIC:
                    finger_joint = msg.name.index(JOINT_FINGER_NAME)
                    finger_state = msg.position[finger_joint]
                    gripper_closed = finger_state < GRIPPER_OPEN_THRESHOLD
                if topic == OBJECT_PATH_INVERTED_TOPIC:
                    object_path_inverted = msg.data
                if topic == OBJECT_ANGLE_TOPIC:
                    object_angle = msg.data

    def run(self):
        self.extract_config()
        self.extract_objects()
        self.extract_pictures()
        # extract_telemetry(bag, report_dir) this uses tf, available when localizing against map
        self.extract_odometry()
        self.extract_wrench()

    def compress(self, full_output_path=None):
        """
        Compress the report folder. Using the same folder name if no path provided.
        """
        full_output_path = (
            self.report_dir if full_output_path is None else full_output_path
        )
        shutil.make_archive(full_output_path, "zip", self.report_dir)


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
    report_generator.compress()
