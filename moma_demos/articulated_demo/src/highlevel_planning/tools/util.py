from scipy.spatial.transform import Rotation as R
import numpy as np
import pybullet as p


def quat_from_rpy(orient_rpy):
    orient_rep = R.from_euler("xyz", orient_rpy.tolist())
    return orient_rep.as_quat()


def quat_from_rpy_yaw_first(orient_rpy):
    orient_rep = R.from_euler("zxy", orient_rpy.tolist())
    return orient_rep.as_quat()


def quat_from_rotvec(rotvec):
    orient_rep = R.from_rotvec(rotvec)
    return orient_rep.as_quat()


def quat_from_mat(mat):
    orient_rep = R.from_dcm(mat)
    return orient_rep.as_quat()


def rotate_orient(orig, axis="z", deg=0.0):
    r = R.from_quat(orig)
    assert axis == "x" or axis == "y" or axis == "z", "Invalid axis"
    op = R.from_euler(axis, deg, degrees=True)
    res = op * r
    return res.as_quat()


def homogenous_trafo(translation, rotation):
    assert type(translation) is np.ndarray
    assert len(translation) == 3
    assert type(rotation) is R
    t = np.concatenate((rotation.as_dcm(), translation.reshape(-1, 1)), axis=1)
    t = np.concatenate((t, np.array([[0.0, 0.0, 0.0, 1.0]])), axis=0)
    return t


def pos_and_orient_from_hom_trafo(hom_trafo):
    return hom_trafo[:3, 3], R.from_dcm(hom_trafo[:3, :3]).as_quat()


def invert_hom_trafo(hom_trafo):
    res = np.eye(4)
    res[:3, :3] = np.transpose(hom_trafo[:3, :3])
    res[:3, 3] = -np.matmul(np.transpose(hom_trafo[:3, :3]), hom_trafo[:3, 3])
    return res


def get_combined_aabb(uid):
    """
    By default, the getAABB function offered by pybullet only outputs the bounding
    box for a single link (the base link by default). This function combines the 
    AABBs of all links an object has.
    
    Args:
        uid (int): The UID of the object we would like to compute the bounding box of.
    """
    num_joints = p.getNumJoints(uid)
    aabb_min, aabb_max = p.getAABB(uid, linkIndex=-1)
    aabb_min, aabb_max = np.array(aabb_min), np.array(aabb_max)
    for i in range(num_joints):
        aabb_local_min, aabb_local_max = p.getAABB(uid, linkIndex=i)
        aabb_local_min, aabb_local_max = (
            np.array(aabb_local_min),
            np.array(aabb_local_max),
        )
        aabb_min = np.minimum(aabb_min, aabb_local_min)
        aabb_max = np.maximum(aabb_max, aabb_local_max)
    return aabb_min, aabb_max


def get_object_position(object_name, scene_objects, knowledge_base):
    if object_name in scene_objects:
        pos, _ = p.getBasePositionAndOrientation(scene_objects[object_name].model.uid)
        return np.array(pos)
    elif object_name in knowledge_base.lookup_table:
        return knowledge_base.lookup_table[object_name]
    else:
        raise ValueError("Invalid object")


class IKError(Exception):
    pass


class SkillExecutionError(Exception):
    pass


class ObjectInfo:
    def __init__(
        self,
        urdf_path_,
        init_pos_,
        init_orient_,
        init_scale_=1.0,
        grasp_links_=None,
        grasp_pos_=None,
        grasp_orient_=None,
        model_=None,
        nav_angle_=None,
        nav_min_dist_=None,
    ):
        if grasp_links_ is None:
            grasp_links_ = list()
        if grasp_pos_ is None:
            grasp_pos_ = dict()
        if grasp_orient_ is None:
            grasp_orient_ = dict()
        assert type(grasp_pos_) is dict
        assert type(grasp_orient_) is dict
        assert type(grasp_links_) is list
        self.urdf_path = urdf_path_
        self.init_pos = init_pos_
        self.init_orient = init_orient_
        self.scale = init_scale_
        self.grasp_pos = grasp_pos_
        self.grasp_orient = grasp_orient_
        self.model = model_
        self.nav_angle = nav_angle_
        self.nav_min_dist = nav_min_dist_
        self.grasp_links = grasp_links_
