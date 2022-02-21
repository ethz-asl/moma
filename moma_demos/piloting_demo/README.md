# piloting_demo

## Simulation

There are two options of running the simulation, either using a global path planner / navigation or using a whole-body MPC controller.

## Installation
In order to run the simulation the SMB stack needs to be cloned. This is reflected in the `moma_piloting.repos` which should be merged using vcstool.

1. Merge the package dependencies with vcstool from the `src` directory of your catking workspace
    ```
    vcs import --input moma/moma_piloting.repos
    ```

2. Update dependencies of the smb stack through rosdep: 

    ```
    rosdep install --from-paths src --ignore-src --skip-keys="pinocchio" -r -y
    ```
    We skip `pinocchio` as we rely on a custom installation which enables the collision detection. Refer to [the installation script](../../install_dependencies.sh) for further information on the installation procedure.

3. At the moment, rosdep does not find some of the packages required in the smb stack. Then it is necessary to iterate the build process and install all missing packages with `sudo apt-get install ros-noetic-<pkgname>` until all packages were resolved. This is an annoying procedure, that we could improve changing the smb stack at a later stage.

5. Install the [`mavsdk-piloting`](https://github.com/fada-catec/piloting-mavsdk) package:
    - Download and install the repo outside of the catkin workspace: 
        ```
        git clone https://github.com/fada-catec/piloting-mavsdk
        cd piloting-mavsdk
        mkdir install
        mkdir build && cd build
        cmake .. -DCMAKE_INSTALL_PREFIX=<path-to-piloting-mavsdk>/install
        make -j4
        make install
        ```
    - Add the install path to the environment variables (better add this line to the `.bashrc`):
        ```
        export LD_LIBRARY_PATH=<path-to-piloting-mavsdk>/install/lib:$LD_LIBRARY_PATH
        export CMAKE_PREFIX_PATH=<path-to-piloting-mavsdk>/install:$CMAKE_PREFIX_PATH
        ```

4. Build the `piloting_demo` package (see above point if something goes wrong) from whithin the catkin workspace.
    ```
    catkin build pilotin_demo
    ```

## Before running the demo

### A detour on frames and their meaning
It is non-trivial to get all the frames right for such a demo. Many frames are involved. Here we review the main ones and what their meaning is in simulation and real world experiments:

- `base_link`: the origin of the robot kinematic chain. Is specified in the center of the moving base, with the x axis poining forward.  
- `map`: the frame used by localization to align with respect to the global map (aka a pre-loaded point cloud of the environment). In real experiments, global information should be defined in this frame since this is the frame which is supposed to stay consistent over time. For example,a target goal for the base is expressed in this frame. 
- `tracking_camera_odom`, `odom`: these are the links with respect to wich we publich the base odometry. As in MPC the kinematic chain starts at the `base_link`, the odometry coming from here is an odometry source that can be used straight away as base position measurements. How do we obtain an odometry at the base link (more or less at the geometrical center of the robot) if there is no odometry sensor in there? 
  - _simulation_: in simulation, the odometry plugin is located at the same location of the tracking camera frame (which is not in the base). This allows 0-shot sim to real.  Nevertheless, a node called `odometry_conversion_node` is used to convert this odometry to position/velocity measurements centered at the base link (like the sensor would be in the base link) allowing controllers to subscribe to this topic instead. The node reads the static transfrom from the sensor location to the base link and uses that to translate twist and position odometry measurements
  - _real world_: same setup as in simulation, but a real Realsense Tracking cam is publishing the desired measurements. Note that we still need the `odometry_conversion_node` to run.
- `world`: this frame allows to represent ground truth in simulation and not have to run the localization pipeline. The world corresponds to the ground truth position of the robot. In other words, one can say that if the localization pipeline would be flawless, the two frames should coincides.
  - _simulation_: in sim `world` means where Gazebo thinks objects are. Unfortunately is not Gazebo that publishes all transformations with respect to its internal notion of world but its us, using some `static_transform_publish` or simple `/tf` topics. We are therefore free to choose the notion of world as long as we are consistent? Not really, because gazebo plugins (e.g camera that publish some point cloud) have embed this internal gazebo notion of world. As we have no power about that, we have to agree and change some plugin to publish the transform that links our robot to `world`. This is currently supported in the odometry gazebo plugin spawned inside the smb.
  - _real world_: in real robot experiments there is no such thing as `world` but only a `map` (output from localization pipeline) and `odom` (or `tracking_camera_odom`). In this case, we are allowed to use a custom published transform from `world` to `map` (e.g identity). 
- `panda_link8`: last link of panda arm, right before the end effector
- `tool_frame`: accessory frame used as the target end effector pose for manipulation tasks
- `object_frame`: ground truth location or estimation of the object pose (should be wrt `map` or `world` frames).

![alt text](docs/frames.png)

**Important Remark**: the tf tree might not look as one would always expect with the root being world, and all other frames, logically placed in the tree path. This is bacause of the constraint imposed by `tf_ros` that the transforms cannot have more then a parent node (it must be a tree).  
  


## Path Planner / Navigation

### Simulation
In Terminal 1 run the  __gazebo simulation__: 
```
roslaunch piloting_demo superpanda_sim.launch mpc:=true
```
In Terminal 2 run the __operator pc__ (visualization + control_gui):
```
roslaunch piloting_demo superpanda_operator.launch
```
The last step should spawn something like this in rviz (note the additional navigation pluing):

![alt text](docs/navigation_demo.png)

Make sure the correct odom frame is set in the navigation panel. After that, one can click on Edit to move the 2d interactive marker to set a new base pose, and after that Start Planning to generate a global path which is then followed by the base (tracking is not perfect and the base will stop moving in a certain tolerance).

More info about the SMB navigation and control stack here: https://ethz-robotx.github.io/SuperMegaBot/.

### Real Robot

TODO

### ground Robot Control Station (gRCS)
Supervision of robotics operation is managed by the ground Robot Control Station. The program in `piloting-mavsdk-ros` is a ros bridge between the control station and the robot (called _robotic engine_) such that communciation can happen via ROS on the client side (state machine) and MAV implementation details are not exposed.

The gRCS can be tested using the [docker image](https://drive.google.com/drive/folders/1YUcn2Whun1ZgzAuddewTMnQ7mcIUk6fn?usp=sharing) (tested and working in Ubuntu20). Follow the README to install and run the docker image. 

The buttons in the image are the most relevant for establishing a new connection.

![alt text](docs/gRCS_connection_buttons.png)

Once the gRCS gui is started, an inspection plan need to uploaded for a mission to start. Press on the _Load Inspection Plan_ button. The docker image should already contain a inspection file that can be used. Then The GCS can connect to the robot by specifing its ip, port and identifer. Open the connection dialog window pressing on the _Robotic System Communication Options_ button and inspect the settings. Note that the settings should match with the config file used in `piloting-mavsdk-ros` to establish the connection.

![alt text](docs/gRCS_communication_options.png)




