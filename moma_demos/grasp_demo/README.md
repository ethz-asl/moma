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

To run the demo on the real robot, use the command

```bash
mon launch grasp_demo grasp_demo.launch
```

The demo can also be run in simulation (Gazebo). For this, run

```bash
mon launch grasp_demo grasp_demo.launch simulation_mode:=true
```

Of course, you can also replace the command `mon launch` with `roslaunch` if you prefer that.
