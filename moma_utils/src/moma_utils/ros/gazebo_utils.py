#! /usr/bin/env python

from __future__ import annotations

import rospy
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel, GetModelProperties, GetModelPropertiesRequest
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

"""
functions for gazebo

the attach_model and dettach_model require:
https://github.com/pal-robotics/gazebo_ros_link_attacher

make sure the plugin "libgazebo_ros_link_attacher.so" is loaded in the world
"""

def model_exists(model_name: str) -> bool:
    """
    check if model already exists in gazebo
    """
    try:
        rospy.wait_for_service('/gazebo/get_model_properties', timeout=5)
        get_model_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
        
        req = GetModelPropertiesRequest()
        req.model_name = model_name

        res = get_model_properties(req)        
        if res.success:
            # rospy.loginfo(f"model '{model_name}' exists.")
            return True
        else:
            # rospy.logwarn(f"model '{model_name}' does not exist.")
            return False

    except rospy.ServiceException as err:
        rospy.logerr(f"Service call failed: {err}")
        return False
    except rospy.ROSException as err:
        rospy.logerr(f"Service timed out: {err}")
        return False

def teleport_model(model_name: str, new_pose: Pose) -> bool:
    """
    teleport named model in gazebo to new pose
    """
    rospy.wait_for_service("/gazebo/set_model_state")
    try:
        service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = new_pose
        res = service(model_state)
        return res.success
    except rospy.ServiceException as err:
        rospy.logerr(f"Failed to teleport model: {err}")
        return False


def spawn_model(model_name: str, model_path: str, pose: Pose) -> bool:
    """
    spawn a model in gazebo at the specified pose
    :param model_name: Name to assign the spawned model
    :param model_path: Path to the model's SDF file
    :param pose: Pose to spawn the model at
    """
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    try:
        with open(model_path, 'r') as f:
            model_xml = f.read()

        service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        res = service(model_name, model_xml, "", pose, "world")
        return res.success
    except (rospy.ServiceException, IOError) as err:
        rospy.logerr(f"Failed to spawn model: {err}")
        return False

def delete_model(model_name: str) -> bool:
    """
    delete named model in gazebo
    """
    rospy.wait_for_service("/gazebo/delete_model")
    if model_exists(model_name):
        try:
            service = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
            res = service(model_name)
            return res.success
        except rospy.ServiceException as err:
            rospy.logerr(f"Service call failed: {err}")
            return False
    else: 
        rospy.logerr(f"model doesn't exist")
        return False


def attach_models(model1: str, link1: str, model2: str, link2: str):
    """
    use gazebo_ros_link_attacher to attach 2 models and 2 links with a fixed joint
    """
    rospy.wait_for_service("/link_attacher_node/attach")
    try:
        service = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
        req = AttachRequest()
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2

        res = service.call(req)
        return res.ok
    except rospy.ServiceException as err:
        rospy.logerr(f"Service call failed: {err}")
        return False


def detach_models(model1: str, link1: str, model2: str, link2: str):
    """
    use gazebo_ros_link_attacher to detach 2 models and 2 links with a fixed joint
    """
    rospy.wait_for_service("/link_attacher_node/detach")
    try:
        service = rospy.ServiceProxy("link_attacher_node/detach", Attach)

        req = AttachRequest()
        rospy.loginfo(req)
        req.model_name_1 = model1
        req.link_name_1 = link1
        req.model_name_2 = model2
        req.link_name_2 = link2

        res = service(req)
        return res.ok
    except rospy.ServiceException as err:
        rospy.logerr(f"Service call failed: {err}")
        return False
