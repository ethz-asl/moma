#! /usr/bin/python

from cmath import inf
import os
import cv2
import sys
import csv

from cv2 import CV_16S
from nbformat import write
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

# static configuration
print("[Report Generation]: Parsing static configuration")
ROOT_DIR = "/home/giuseppe/storage"
BAG = "/home/giuseppe/storage/bags/2022-01-17/2022-01-17-16-30-09_valve_perception.bag"
MAP_FRAME = "tracking_camera_odom"
IMAGE_TOPIC = "/hand_eye/color/image_raw_throttle"
BASE_LINK = "base_link"
CAMERA_UUID = str(uuid.uuid4())
SENSORS_LIST = {"rslidar": ["16 Beans Lidar Sensor", str(uuid.uuid4())],
                "realsense_t265": ["Realsense Tracking Camera", str(uuid.uuid4())],
                "imu": ["XSense Imu", str(uuid.uuid4())]}


startDate = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
endDate = startDate
robot_uuid = str(uuid.uuid4())
sync_id = 1667993927
task_uuid = str(uuid.uuid4())
inspection_plan_uuid = str(uuid.uuid4())
inspection_task_uuids = [str(uuid.uuid4())]
inspection_type = "visual"  # visual, contact, TBD
map_file = "map.pcd"


# Utilities
def get_files():
    return {
        "pictures_metadata": "pictures_metadata.csv",
        "pictures_folder": "pictures",
        "telemetry_data": "localization_telemetry.csv",
    }


def get_tf_tree(bag: rosbag.Bag):
    """ Fills up a tf tree from a rosbag """
    tf_tree = tf2.BufferCore(rospy.Duration(360000.0))
    tf_topics = ["/tf", "/tf_static"]
    tf_msgs_count = bag.get_message_count(tf_topics)
    print("[Report Generation]:  Reading {} transforms from the bag".format(
        tf_msgs_count))

    times = []
    with tqdm(total=tf_msgs_count) as progress_bar:
        for topic, message, t in bag.read_messages(topics=tf_topics):
            times.append(t)
            for tf_message in message.transforms:
                if topic == '/tf_static':
                    tf_tree.set_transform_static(tf_message, topic)
                else:
                    tf_tree.set_transform(tf_message, topic)
        progress_bar.update(1)
    return tf_tree, times


def get_sensors():
    sensors = []
    for sensor_frame, sensor_info in SENSORS_LIST.items():
        sensor_entry = {}
        sensor_entry["name"] = sensor_frame
        sensor_entry["uuid"] = sensor_info[1]
        sensor_entry["description"] = sensor_info[0]
        sensor_entry["tf"] = {"translation": {"x": 0.0, "y": 0.0, "z": 0.0},
                              "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}}
        try:
            tf_transform = tf_tree.lookup_transform_core(
                target_frame=BASE_LINK, source_frame=sensor_frame, time=rospy.Time(0))
            sensor_entry["tf"]["translation"]["x"] = tf_transform.transform.translation.x
            sensor_entry["tf"]["translation"]["y"] = tf_transform.transform.translation.y
            sensor_entry["tf"]["translation"]["z"] = tf_transform.transform.translation.z
            sensor_entry["tf"]["rotation"]["x"] = tf_transform.transform.rotation.x
            sensor_entry["tf"]["rotation"]["y"] = tf_transform.transform.rotation.y
            sensor_entry["tf"]["rotation"]["z"] = tf_transform.transform.rotation.z
            sensor_entry["tf"]["rotation"]["w"] = tf_transform.transform.rotation.w
        except Exception as exc:
            print(exc)
            print("[Report Generation]: Failed to get transform from {} to {}".format(
                sensor_frame, BASE_LINK))
        sensors.append(sensor_entry)
    return sensors


def extract_telemetry(bag: rosbag.Bag, root_dir):
    telemetry_csv_file = os.path.join(root_dir, "localization_telemetry.csv")
    with open(telemetry_csv_file, 'w') as f:
        writer = csv.writer(f)
        for t in times:
            try:
                tf_transform = tf_tree.lookup_transform_core(
                    target_frame=MAP_FRAME, source_frame=BASE_LINK, time=t)
                translation = [tf_transform.transform.translation.x,
                               tf_transform.transform.translation.y,
                               tf_transform.transform.translation.z]
                rotation = [tf_transform.transform.rotation.x,
                            tf_transform.transform.rotation.y,
                            tf_transform.transform.rotation.z,
                            tf_transform.transform.rotation.w]
                entry = [t.to_sec(), task_uuid] + translation + rotation
                writer.writerow(entry)
            except Exception as exc:
                print(exc)


def extract_pictures(bag: rosbag.Bag, root_dir):
    bridge = CvBridge()
    images_delta_time = 5  # take a picture each 10 seconds
    t_prev = -inf
    img_idx = 0
    pictures_csv_file = os.path.join(root_dir, "pictures_metadata.csv")
    with open(pictures_csv_file, 'w') as f:
        writer = csv.writer(f)
        for topic, message, t in bag.read_messages(topics=IMAGE_TOPIC):
            if (t.to_sec() - t_prev) > images_delta_time:
                t_prev = t.to_sec()

                translation = [0.0, 0.0, 0.0]
                rotation = [0.0, 0.0, 0.0, 1.0]

                try:
                    tf_transform = tf_tree.lookup_transform_core(
                        target_frame="odom", source_frame=message.header.frame_id, time=rospy.Time(0))
                    translation = [tf_transform.transform.translation.x,
                                   tf_transform.transform.translation.y,
                                   tf_transform.transform.translation.z]
                    rotation = [tf_transform.transform.rotation.x,
                                tf_transform.transform.rotation.y,
                                tf_transform.transform.rotation.z,
                                tf_transform.transform.rotation.w]
                except Exception as exc:
                    pass

                meta = ["DSC{:06d}.jpg".format(
                    img_idx), t_prev, task_uuid] + translation + rotation
                writer.writerow(meta)
                img_idx += 1

                # Save the image as well
                cv2_img = bridge.imgmsg_to_cv2(message, "rgb8")
                cv2.imwrite(os.path.join(
                    root_dir, "pictures", meta[0]), cv2_img)


# open the bag
print("[Report Generation]: Opening rosbag at {}".format(BAG))
if not os.path.isfile(BAG):
    print("No file {} was found".format(BAG))
    sys.exit(0)
bag = rosbag.Bag(BAG, mode='r')

# fill the tf tree to retrieve transforms
print("[Report Generation]: Reading tf info...")
tf_tree, times = get_tf_tree(bag)

report_config = {
    "startDate": startDate,
    "endDate": endDate,
    "syncId": sync_id,
    "uuid": robot_uuid,
    "inspectionPlanUuid": inspection_plan_uuid,
    "insepctionTaskUuids": inspection_task_uuids,
    "type": inspection_type,
    "map": map_file,
    "files": get_files(),
    "sensors": get_sensors(),
    "pictures": {
        "folder": "pictures",
        "sensor_uuid": CAMERA_UUID,
        "format": "jpg",
        "size": {
            "width": 1280,
            "height": 720
        },
        "fov": {  # https://www.intelrealsense.com/depth-camera-d435i/
            "height": 69.0,
            "vertical": 42.0
        }
    }
}

print("[Report Generation]: Creating folder structure.")
report_dir = os.path.join(ROOT_DIR, startDate + "_inspection_plan")
os.makedirs(report_dir, exist_ok=True)

print("[Report Generation]: Writing top level config file.")
config_file = os.path.join(report_dir, "config.json")
with open(config_file, "w") as outfile:
    json.dump(report_config, outfile, indent=2)

print("[Report Generation]: Extracting pictures and saving metadata.")
pictures_folder = os.path.join(report_dir, "pictures")
os.makedirs(pictures_folder, exist_ok=True)
extract_pictures(bag, report_dir)

print("[Report Generation]: Extracting telemety information.")
extract_telemetry(bag, report_dir)
