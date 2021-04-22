import pybullet as p
from highlevel_planning.tools.util import ObjectInfo
import numpy as np
from scipy.spatial.transform import Rotation as R
import os


class Cupboard:
    def __init__(self, world, pos_, orient_, base_dir):
        self.urdf = os.path.join(
            base_dir, "data/models/cupboard_drawers/cupboard_drawers.urdf"
        )
        self.model = world.add_model(self.urdf, position=pos_, orientation=orient_)
        self.pos = pos_
        self.orient = orient_

        rot = R.from_quat(orient_)
        yaw = rot.as_euler("xyz", degrees=False)
        self.nav_angle = yaw[2] + np.pi * 3.0 / 2.0

        drawer_link_idx = []
        self.handle_link_idx = [0] * 4
        for i in range(p.getNumJoints(self.model.uid)):
            info = p.getJointInfo(self.model.uid, i)
            joint_name = info[1] if type(info[1]) is str else info[1].decode("utf-8")
            # print(info)
            if "drawer_joint" in joint_name and len(joint_name) == 13:
                drawer_link_idx.append(i)
            if "drawer_handle_dummy_joint" in joint_name:
                handle_num = int(joint_name.split("drawer_handle_dummy_joint")[1])
                self.handle_link_idx[handle_num - 1] = info[16]
        for i in drawer_link_idx:
            p.setJointMotorControl2(
                self.model.uid, i, controlMode=p.VELOCITY_CONTROL, force=0.0
            )

    def get_info(self):
        grasp_orient = R.from_euler("xzy", [180, 0, -45], degrees=True)
        return ObjectInfo(
            urdf_path_=self.urdf,
            init_pos_=np.array(self.pos),
            init_orient_=np.array(self.orient),
            grasp_pos_={
                link: [np.array([0.0, 0.0, 0.0])] for link in self.handle_link_idx
            },
            grasp_orient_={
                link: [grasp_orient.as_quat()] for link in self.handle_link_idx
            },
            model_=self.model,
            nav_angle_=self.nav_angle,
            nav_min_dist_=0.9,			#CHANGE BACK TO 0.9
            grasp_links_=self.handle_link_idx,
        )
