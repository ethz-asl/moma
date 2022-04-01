# moma_utils

Utilities that can be shared among moma projects.

- `ros_control.py`: Wrappers around various ROS control services.
- `ros_conversions.py`: Various conversions to/from ROS messages.
- `transform.py`: Rigid spatial transformations in Python.

## Installation

Install the requirements within an activated virtual environment.

```
pip install -r requirements.txt
```

### Catkin

```
catkin build moma_utils
```

### Python-only

This package can also be installed as a stand-alone Python package.
Note that importing some of the classes might fail due to missing ROS dependencies.

```
cd /path/to/moma_utils
pip install -e .
```
