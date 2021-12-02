# moma_gazebo

Launch files, models and worlds for Gazebo simulation.

## Panda Example

To run the demo:

```bash
roslaunch moma_gazebo panda_example.launch
```

## Royal Panda Example

To run the demo:

```bash
roslaunch moma_gazebo royalpanda_example.launch
```

## Panda Piloting Example

Build the following required packages:
- `panda_mpc`

Optional packages:
- `path_admittance_controller`

To run the demo:
```bash
roslaunch moma_gazebo panda_piloting.launch
```
