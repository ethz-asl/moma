# kinova_valve_opening

This package contains the state machine for the valve opening task

### Run the simulation 

On separate terminals:

1. `roslaunch kinova_common run_sim.launch`
2. `roslaunch kinova_valve_opening mission.launch simulation:=true`

### Run on the hardware

On separate terminals (Robot):

1. `roslaunch kinova_robot robot.launch`
2. `roslaunch kinova_valve_opening components.launch`

On separate terminals (Operator):

1. `roslaunch kinova_valve_opening mission.launch`
