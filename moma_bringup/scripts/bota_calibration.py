#!/usr/bin/env python

from six.moves import input # wait for input from keyboard

import rospy
import tf2_ros
import yaml
import rospkg
import tf2_geometry_msgs # for tf2 to find posestamped 

from geometry_msgs.msg import PoseStamped
from rokubimini_msgs.msg import Reading

from moma_utils.ros.panda_client import PandaArmClient, PandaGripperClient

"""
repeatibility test

poses:
  - z_down
  - x_down
  - y_down

repeats: 10

procedure:
  for pose in poses:
    for i in range(repeats):
      go_to_ready()
      wait(2)
      go_to_pose(pose)
      wait(5)
      record_sensor_reading()
      save(pose, i)

"""


class BotaCalibration:
    """send bota to many poses"""

    def __init__(self) -> None:
        rospy.init_node("bota_calibration")

        self.arm_ = PandaArmClient()
        self.planning_frame_ = self.arm_.planning_frame
        self.ee_frame_ = self.arm_.eef_link
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_)
        #TODO get poses from yaml
        
        fake_transform = self.tf_buffer_.lookup_transform(self.ee_frame_, self.planning_frame_, time=rospy.Time(0), timeout=rospy.Duration(1))
        rospy.loginfo(f"transform ? {fake_transform}")
        self.arm_.go_to_ready()
        self.arm_.set_planning_pipeline("pilz_industrial_motion_planner")
        
    def load_poses(self, file_name : str = "bota_poses.yaml") -> None:
        pkg = rospkg.RosPack()
        
        yaml_file = (
            pkg.get_path("moma_bringup") + "/config/" + file_name
        )
        pose_dict = yaml.safe_load(open(yaml_file))["poses"]
        return pose_dict

        # for name, pose_data in pose_dict.items():
        #     input("Press enter to go to the next poose")
        #     pose_msg = self.parse_pose(pose_data)
        #     success = self.send_pose(pose_msg)
            
        #     if not success:
        #         rospy.logwarn(f"Failed")
        #     self.arm_.go_to_ready()

        #     data_log = []

        # for pose_name, pose_data in pose_dict.items():
        #     pose_msg = self.parse_pose(pose_data)
        #     success = self.send_pose(pose_msg)

        #     if not success:
        #         rospy.logwarn(f"Failed to reach pose: {pose_name}")
        #         continue

        #     rospy.sleep(5.0)

        #     sample = self.record_data(pose_name, pose_msg)
        #     if sample:
        #         data_log.append(sample)

        # self.save_data(data_log)


    def parse_pose(self, pose_data) -> PoseStamped:

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.ee_frame_

        pose.pose.position.x = pose_data["position"]["x"]
        pose.pose.position.y = pose_data["position"]["y"]
        pose.pose.position.z = pose_data["position"]["z"]

        pose.pose.orientation.x = pose_data["orientation"]["x"]
        pose.pose.orientation.y = pose_data["orientation"]["y"]
        pose.pose.orientation.z = pose_data["orientation"]["z"]
        pose.pose.orientation.w = pose_data["orientation"]["w"]
        
        return pose


    def send_pose(self, pose: PoseStamped) -> bool:
        """send pose in ee frame, then send to robot in planning frame"""
        pose_transformed = self.transform_pose(pose, self.planning_frame_)
        return self.arm_.go_to_pose_goal(pose_transformed.pose)

        
    def transform_pose(self, pose : PoseStamped, target_frame : str) -> PoseStamped:
        try:
            pose_transformed = self.tf_buffer_.transform(
                pose,
                target_frame,
                rospy.Duration(1.0)
            )
            return pose_transformed
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn(f"TF tranform error: {exc}")
            return None

    
    def record_data(self, pose_name: str, pose_msg: PoseStamped, timeout=2.0):
        """Record a single sensor reading from the Bota F/T sensor."""
        try:
            data_msg = rospy.wait_for_message("/bus0/ft_sensor0/", Reading, timeout=timeout)
        except rospy.ROSException:
            rospy.logwarn("Timed out waiting for sensor data.")
            return None

        data_entry = {
            "pose_name": pose_name,
            "position": {
                "x": pose_msg.pose.position.x,
                "y": pose_msg.pose.position.y,
                "z": pose_msg.pose.position.z,
            },
            "orientation": {
                "x": pose_msg.pose.orientation.x,
                "y": pose_msg.pose.orientation.y,
                "z": pose_msg.pose.orientation.z,
                "w": pose_msg.pose.orientation.w,
            },
            "wrench": {
                "force": {
                    "x": data_msg.wrench.wrench.force.x,
                    "y": data_msg.wrench.wrench.force.y,
                    "z": data_msg.wrench.wrench.force.z,
                },
                "torque": {
                    "x": data_msg.wrench.wrench.torque.x,
                    "y": data_msg.wrench.wrench.torque.y,
                    "z": data_msg.wrench.wrench.torque.z,
                },
            },
            "temperature": data_msg.temperature.temperature
        }

        return data_entry
    

    def save_data(self, data_list, file_path="bota_calibration_data.yaml"):
        with open(file_path, "w") as file:
            yaml.dump({"samples": data_list}, file, default_flow_style=False)

    def run_repeatability_test(self, repeats=10):
        pose_dict = self.load_poses()
        results = []

        for pose_name in ["z_down", "x_down", "y_down"]:
            if pose_name not in pose_dict:
                rospy.logwarn(f"{pose_name} not in provided pose dict.")
                continue

            pose_msg = self.parse_pose(pose_dict[pose_name])

            for trial in range(repeats):
                rospy.loginfo(f"Trial {trial+1} for {pose_name}")

                self.arm_.go_to_ready()
                rospy.sleep(2.0)

                if not self.send_pose(pose_msg):
                    rospy.logwarn(f"Failed to reach pose {pose_name}")
                    continue

                rospy.sleep(5.0)
                sample = self.record_data(f"{pose_name}_{trial+1}", pose_msg)
                if sample:
                    results.append(sample)

        self.save_data(results)

def main():

    try:
        calib = BotaCalibration()
        calib.run_repeatability_test()

    except rospy.ROSInterruptException or KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
