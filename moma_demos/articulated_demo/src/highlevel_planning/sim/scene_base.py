class SceneBase:
    def __init__(self, world, base_dir, restored_objects=None):
        self._world = world

        if restored_objects is not None:
            self.objects = restored_objects

    def add_objects(self):
        print("---------------------------")
        for key, obj in self.objects.items():
            if self.objects[key].model is None:
                self.objects[key].model = self._world.add_model(
                    obj.urdf_path, obj.init_pos, obj.init_orient, scale=obj.scale
                )
            print("Added object " + key + ". ID: " + str(obj.model.uid))
        print("---------------------------")
