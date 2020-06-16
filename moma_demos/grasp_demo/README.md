# grasp_demo

## Install

This package has a few python dependencies. It is recommended to install them in a virtual environment in the project root:

```bash
cd moma
virtualenv -p /usr/bin/python2 --system-site-packages .venv
source .venv/bin/activate
cd ./moma_demos/grasp_demo
pip install -r requirements.txt
```

**Requirement for reacitivity**:
- py_trees
- py_trees_msgs
- py_trees_ros
- rqt_py_trees
Install versions according to [Github] (https://github.com/splintered-reality/py_trees_ros)

## Run

### Real Robot
To run the demo on the real robot, use the command

```bash
mon launch grasp_demo grasp_demo.launch
```
### Simulated Robot 
The demo can also be run in simulation (Gazebo). For this, run

**Terminal 1**: Start roscore
```bash
roscore
```

**Terminal 2**: Start Simulation
```bash
roslaunch grasp_demo grasp_demo.launch
```

**Terminal 3**: Modify parameter entries
```bash
rosparam set /moma_demo/parameter value
```

| **Parameter**             | **Values** [recom. (others)]                      | **Type** | **Description**                                                                     |
|---------------------------|---------------------------------------------------|----------|-------------------------------------------------------------------------------------|
| grasp_selection_method    | box (auto, manual)                                | string   | Change way how the grasp selection works (box for reactivity)                       |
| grasp_target              | mouse, (orange, mouse, bowle, cup                 | string   | Entry from the yolo_coco_names class list.                                          |
| occlusion_margin          | 100                                               | int      | Size of boundry box for occlusion node (lower getting errors, higher less reactive) |
| tick_tock_times           | 2.0                                               | float    | "Tick Tock" frequency of the behaviour tree                                         |
| tracking_method           | kcf, (csrt, boosting, mil, tld, mediaflow, mosse) | string   | Change tracker for tracking node from perception branche                            |
| tracking_motion_threshold | 30                                                | int      | "Motion" gets summed up, moving if sum > threshold                                  |
| tracking_threshold_length | 20                                                | int      | Max length of "motion", motion(t=<(now-20)) is not accounted any more               |

***Terminal 4**: Move Gazebo Models
```bash
rosrun grasp_demo move_target.py x y d t target
```
where
| **Parameter** | **Values** [recom. (others)] | **Type** | **Description**                                           |
|---------------|------------------------------|----------|-----------------------------------------------------------|
| x             | 1                            | int      | x-direction in Gazebo, has no influence in distance       |
| y             | 0                            | int      | y-direction in Gazebo, has no influence in distance       |
| d             | 0.1                          | float    | distance in meters in Gazebo                              |
| t             | 2.0                          | float    | duration of motion in seconds. Influence on acceleration. |
| target        | s, (c)                       | string   | s: sphere--> orange, c*: cube --> mouse                   |
*Currently only the orange sphere "s" works nicely*

Of course, you can also replace the command `roslaunch` with `mon launch` if you prefer that.
