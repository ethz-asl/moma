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

## Run

Frist, launch the hardware drivers and nodes.

```bash
roslaunch grasp_demo grasp_demo.launch
```

* To run the demo in Gazebo, add `simulation_mode:=true` to the above command.

Next, for interacting with the demo through Rviz, run 

```
 rosrun grasp_demo run_bt.py __ns:=manipulator
```

Or alternatively, to run the demo continuously, use

```
rosrun grasp_demo run_plan.py __ns:=manipulator
```

## To Do

- [ ] Semantic grasp selection
