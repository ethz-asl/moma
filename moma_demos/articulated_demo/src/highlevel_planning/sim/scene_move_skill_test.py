from highlevel_planning.sim.scene_base import SceneBase
from highlevel_planning.sim.cupboard_test import Cupboard
from highlevel_planning.sim.room_door_test import RoomDoor


class SceneMoveSkill(SceneBase):
    def __init__(self, world, base_dir, graspOffset, elementToAdd, restored_objects=None):
        SceneBase.__init__(self, world, base_dir, restored_objects)

        if restored_objects is None:
            self._world.add_plane()
            self.objects = dict()
            
            if elementToAdd == "cupboard":
                cupboard = Cupboard(
                        world,
                        pos_=[0.0, 2.0, 0.0],
                        orient_=[0.0, 0.0, 0.0, 1.0],
                        base_dir=base_dir,
                        grasp_offset = graspOffset
                        )
                self.objects["cupboard"] = cupboard.get_info()
                
            if elementToAdd == "roomdoor":
                roomdoor = RoomDoor(
                        world,
                        pos_=[0.0, 2.0, 0.0],
                        orient_=[0.0, 0.0, 0.0, 1.0],
                        base_dir=base_dir,
                        grasp_offset = graspOffset
                        )
                self.objects["roomdoor"] = roomdoor.get_info()
                
            self.add_objects()
