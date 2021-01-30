import pybullet as p
import os
import pybullet_data
import numpy as np
import time
from math import ceil
import atexit


class World(object):
    def __init__(self, sleep=True):
        self.sleep_flag = sleep

        self.sim_time = 0.0
        self.f_s = 240.0
        self.T_s = 1.0 / float(self.f_s)

        self.collision_checker = None
        self.cross_uid = ()

        self.forces = []

        self.colors = {
            "green": (123.0 / 255.0, 159.0 / 255.0, 53.0 / 255.0),
            "red": (170.0 / 255.0, 57.0 / 255.0, 57.0 / 255.0),
            "blue": (34.0 / 255.0, 102.0 / 255.0, 102.0 / 255.0),
            "yellow": (1.0, 1.0, 0.0),
        }

        self.velocity_setter = None
        atexit.register(self.close)

    def sleep(self, seconds):
        if self.sleep_flag:
            time.sleep(seconds)

    def step_one(self):
        raise NotImplementedError

    def step_seconds(self, secs):
        for _ in range(int(ceil(secs * self.f_s))):
            self.step_one()
            self.sleep(self.T_s)

    def close(self):
        raise NotImplementedError


class _Model:
    def __init__(self, physics_client):
        self._physics_client = physics_client
        self.uid = 0
        self.name = ""
        self.link_name_to_index = dict()

    def load(self, path, position, orientation, scale, elementToAdd):
        model_path = os.path.expanduser(path)
        
        if elementToAdd == "lid":
            self.uid = p.loadURDF(
                    model_path,
                    position,
                    orientation,
                    globalScaling=scale,
                    useFixedBase=False,						#CHANGE BACK !!!!!!!
                    physicsClientId=self._physics_client,
            )            
        else:
            self.uid = p.loadURDF(
                    model_path,
                    position,
                    orientation,
                    globalScaling=scale,
                    useFixedBase=True,						#CHANGE BACK !!!!!!!
                    physicsClientId=self._physics_client,
            )
            
        self.name = p.getBodyInfo(self.uid)

        for i in range(p.getNumJoints(self.uid)):
            info = p.getJointInfo(self.uid, i)
            name = info[12] if type(info[12]) is str else info[12].decode("utf-8")
            self.link_name_to_index[name] = i

    def remove(self):
        p.removeBody(self.uid)


class WorldPybullet(World):
    def __init__(self, style="gui", sleep=True, load_objects=True, savedir=None):
        super(WorldPybullet, self).__init__(sleep)
        if not load_objects:
            assert savedir is not None

        if style == "gui":
            self.physics_client = p.connect(p.GUI)
        elif style == "shared":
            self.physics_client = p.connect(p.SHARED_MEMORY)
        elif style == "direct":
            self.physics_client = p.connect(p.DIRECT)
        else:
            raise ValueError

        if load_objects:
            p.resetSimulation(self.physics_client)
        else:
            p.restoreState(fileName=os.path.join(savedir, "state.bullet"))
            p.removeAllUserDebugItems()

        p.setGravity(0, 0, -9.81, self.physics_client)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

    def add_model(self, path, position, orientation, scale=1.0, elementToAdd=None):
        model = _Model(self.physics_client)
        model.load(path, position, orientation, scale, elementToAdd)
        return model

    def del_model(self, model):
        model.remove()

    def draw_cross(self, point):
        if len(self.cross_uid) > 0:
            p.removeUserDebugItem(self.cross_uid[0])
            p.removeUserDebugItem(self.cross_uid[1])
            p.removeUserDebugItem(self.cross_uid[2])
        start1 = point - np.array([0.1, 0.0, 0.0])
        end1 = point + np.array([0.1, 0.0, 0.0])
        start2 = point - np.array([0.0, 0.1, 0.0])
        end2 = point + np.array([0.0, 0.1, 0.0])
        start3 = point - np.array([0.0, 0.0, 0.1])
        end3 = point + np.array([0.0, 0.0, 0.1])
        color = np.array([255, 0, 0]) / 255.0
        width = 1.0
        lifetime = 0
        uid1 = p.addUserDebugLine(
            start1.tolist(), end1, color.tolist(), width, lifetime
        )
        uid2 = p.addUserDebugLine(
            start2.tolist(), end2, color.tolist(), width, lifetime
        )
        uid3 = p.addUserDebugLine(
            start3.tolist(), end3, color.tolist(), width, lifetime
        )
        self.cross_uid = (uid1, uid2, uid3)

    def draw_arrow(self, point, direction, color, length=0.2, replace_id=None):
        """ Accepts a point and a direction in world frame and draws it in the simulation """
        tip = point + direction / np.linalg.norm(direction) * length
        color = self.colors[color]
        width = 4.5
        lifetime = 0
        if replace_id is not None:
            arrow_id = p.addUserDebugLine(
                point.tolist(),
                tip.tolist(),
                color,
                width,
                lifetime,
                replaceItemUniqueId=replace_id,
            )
        else:
            arrow_id = p.addUserDebugLine(
                point.tolist(), tip.tolist(), color, width, lifetime
            )
        return arrow_id

    def step_one(self):
        for frc in self.forces:
            p.applyExternalForce(frc[0], frc[1], frc[2], frc[3], frc[4])
        if self.velocity_setter is not None:
            self.velocity_setter()
        p.stepSimulation()
        if self.collision_checker is not None:
            self.collision_checker()

    @staticmethod
    def add_plane():
        return p.loadURDF("plane.urdf")

    def close(self):
        print("Closing world")
        p.disconnect(self.physics_client)
