from highlevel_planning.sim.scene_base import SceneBase

from highlevel_planning.sim.cupboard_configurable import Cupboard
from highlevel_planning.sim.room_door_configurable import RoomDoor
from highlevel_planning.sim.sliding_door_configurable import SlidingDoor
from highlevel_planning.sim.sliding_lid_configurable import SlidingLid
from highlevel_planning.sim.removable_lid_configurable import RemovableLid

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
                
            if elementToAdd == "slidingdoor":
                slidingdoor = SlidingDoor(
                        world,
                        pos_=[0.0, 2.0, 0.0],
                        orient_=[0.0, 0.0, 0.0, 1.0],
                        base_dir=base_dir,
                        grasp_offset = graspOffset
                        )
                self.objects["slidingdoor"] = slidingdoor.get_info()

            if elementToAdd == "slidinglid":
                slidinglid = SlidingLid(
                        world,
                        pos_=[0.0, 2.0, 0.0],
                        orient_=[0.0, 0.0, 0.0, 1.0],
                        base_dir=base_dir,
                        grasp_offset = graspOffset
                        )
                self.objects["slidinglid"] = slidinglid.get_info()

            if elementToAdd == "removablelid":
                removablelid = RemovableLid(
                        world,
                        pos_=[0.0, 2.0, 0.0],
                        orient_=[0.0, 0.0, 0.0, 1.0],
                        base_dir=base_dir,
                        grasp_offset = graspOffset
                        )
                self.objects["removablelid"] = removablelid.get_info()
                
            self.add_objects()
