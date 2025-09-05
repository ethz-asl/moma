# moma_bringup

This package contains launch files to start interfaces with the real robots.
See the [wiki](https://github.com/ethz-asl/moma/wiki/Robots) for more information about our platforms.

### Handeye calibration (panda + intel realsense 435)
IMPORTANT: this guide is only valid in the branch giulio/machine_hall_setup.

#### Preliminaries
The launch file panda_real.launch brings up the real panda arm and the wrist camera. 

Instead of setting the calibration by publishing a static transform, we leverage the panda_arm.xacro, which includes the intel realsense gazebo plugin. This takes care of loading the 3D model for the camera and places it on the robot. Importantly, the plugin assumes that the root of the realsense camera is the frame **wrist_camera_bottom_screw_frame** and that is parent is **panda_link8**.

The laungh file panda_calibration.launch launches the apriltag detector and the easy handeye node. The standard easy handeye node provides little automation, we therefore use the master branch of the fork at https://github.com/giuschio/ros2_handeye_calibration/tree/master (despite being called ros2, it has a ros1 branch as well).

#### Calibration parameters and procedure
As per the preliminaries, we calibrate the following frames
``` xml
  <!-- new args for frame IDs -->
  <arg name="robot_base_frame" default="panda_link0" />
  <arg name="robot_effector_frame" default="panda_link8" />
  <arg name="camera_base_link" default="wrist_camera_bottom_screw_frame" />
```

After starting **panda_real.launch**, start **panda_calibration.launch**. It will start three windows:
- the main calibration window
- a planning window
- an rqt window that visualizes the apriltag detection from the camera

To start the calibration, press **Check starting pose** in the planning window. This will initialize a buffer of end-effector poses around the current one. Once that is done, press **Next pose**. This will move the end-effector. If the tag is in view, press **Take sample** in the other window. This will take a picture and, if appropriate, calculate the calibration. Repeat N times. Each time, the node calculates the calibration and outputs it, as well as the translation and rotation delta relative to the previous calibration. This allows the user to monitor convergence.

#### After calibration
Take the translation and rpy calibration values and paste them in **panda_real.launch** here (wrist_calibration_rpy and wrist_calibration_xyz):
```xml
<!-- Launch control -->
  <include file="$(find moma_bringup)/launch/components/franka_control.launch">
    <arg name="robot_ip" value="$(arg robot_ip)" />
    <!--
    publish_default_realsense_extrinsics: whether to publish the default extrinsics between the realsense camera
      frames (rgb-depth). When a real camera is connected, the realsense2 package will publish the actual extrinsics
      that are factory-calibrated for that camera

    ATTENTION: the root of the camera model is the wrist_camera_bottom_screw_frame, so that is
    the one we need to calibrate
    -->
    <arg name="xacro_args"
      value=
      "use_fixed_camera:=false 
      use_wrist_camera:=true 
      publish_default_realsense_extrinsics:=false 
      use_bota:=false
      wrist_calibration_rpy:='0.05886170941712955 -1.5489516731575133 2.3023482849455768'
      wrist_calibration_xyz:='0.023815825061890047 -0.025524294680885358 0.04051021299479383'" />
  </include>
```

Then, you can start the **panda_real.launch** again, to visually verify the calibration.