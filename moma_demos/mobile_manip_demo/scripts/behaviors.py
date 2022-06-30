"""Implement various py trees behaviors for object pick and place."""

from typing import Any, List

from mobile_manip_demo import robot_interface
import py_trees as pt


# import the robot interface where ROS bindings are defined


class AtPos(pt.behaviour.Behaviour):
    """Check if object is at position."""

    def __init__(
        self,
        name: str,
        object_and_pos: list,
        world_interface: Any,
        verbose: bool = False,
    ):
        self.object = int(object_and_pos[0])
        self.pos = sim.Pos(
            object_and_pos[1][0], object_and_pos[1][1], object_and_pos[1][2]
        )
        super().__init__(name, world_interface, verbose)

    def update(self):
        if self.world_interface.is_close(self.object, self.pos):
            self.success()
        else:
            self.failure()
        return self.state


class Move(pt.behaviour.Behaviour):
    """Navigate to the input goal."""

    def __init__(self, name: str, goal_ID: str, world_interface: Any):
        super().__init__(name)
        self.interface = world_interface
        self.goal = goal_ID

    def initialise(self):
        self.interface.initialize_navigation(self.goal)

    def update(self) -> pt.common.Status:
        status = self.interface.get_navigation_status()
        if status == 0 or status == 1:
            return pt.common.Status.RUNNING
        elif status == 3:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def terminate(self, new_status: pt.common.Status):
        if new_status == pt.common.Status.INVALID:
            self.interface.cancel_goal()
