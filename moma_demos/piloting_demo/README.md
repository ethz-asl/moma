# piloting_demo

## Simulation

There are two options of running the simulation, either using a global path planner / navigation or using a whole-body MPC controller.

## Installation
In order to run the simulation the SMB stack needs to be cloned. This is reflected in the `moma_piloting.repos` which should be merged using vcstool.

1. Merge the package dependencies with vcstool from the `src` directory of your catking workspace
    ```
    vcs import --input moma/moma_piloting.repos
    ```
    If you need to also install packages for SLAM, you need additional dependency for the `smb_slam` package. These can be installed running:
    ```
    vcs import --input https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
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




