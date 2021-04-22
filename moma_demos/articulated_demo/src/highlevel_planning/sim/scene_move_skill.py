from highlevel_planning.sim.scene_base import SceneBase
from highlevel_planning.sim.cupboard import Cupboard


class SceneMoveSkill(SceneBase):
    def __init__(self, world, base_dir, restored_objects=None):
        SceneBase.__init__(self, world, base_dir, restored_objects)

        if restored_objects is None:
            self._world.add_plane()
            self.objects = dict()
            cupboard = Cupboard(
                world,
                pos_=[0.0, 2.0, 0.0],
                orient_=[0.0, 0.0, 0.0, 1.0],
                base_dir=base_dir,
            )
            self.objects["cupboard"] = cupboard.get_info()
            self.add_objects()
