# moma_mission

This package contains the state machine for mobile manipulator missions.
The core library is not related to a specific mission but is intended to implement general purposes states (relying also on implemented controllers).


Each mission can be implemented as a module of the library and further extend it with 'mission dependent' states. See the [piloting module](src/moma_mission/missions/piloting) for an example.


Then creating a mission is a matter of concatenating states using the `smach` API. The `StateRos` wraps the smach state adding functionality like retrieving params after the state name. This way, parameters can be well structured in the same param file and namescoped with nice readability. See the [piloting mission file](config/state_machine/piloting.yaml) for an example.