#!/usr/bin/env python

import rospy
import tf2_ros

import moma_utils.ros.conversions as utils
from moma_utils.spatial import Transform
from moma_utils.ros.panda import PandaArmClient, PandaGripperClient
from moma_utils.ros.moveit import MoveItClient

from stacking_demo.srv import (
    MoveToTower, MoveToTowerRequest, MoveToTowerResponse,
    PlanEasyGrasp, PlanEasyGraspRequest, PlanEasyGraspResponse,
    MoveToObjID, MoveToObjIDRequest, MoveToObjIDResponse
)

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from geometry_msgs.msg import PoseStamped, TransformStamped

class MoveServices:
    def __init__(self) -> None:
        """service servers to handle different arm movements"""

        rospy.init_node("move_node", anonymous=True)
        self.moveit_ = MoveItClient("panda_arm")
        self.arm_ = PandaArmClient()   
        self.gripper_ = PandaGripperClient()

        self.load_parameters()

        # start servers here
        self.object_srv = rospy.Service("move_to_object", Trigger, self.move_to_object)
        self.object_id_srv = rospy.Service("move_to_object_id", MoveToObjID, self.move_to_object_id)
        self.return_object_srv = rospy.Service("return_object", MoveToObjID, self.return_object)
        self.home_srv = rospy.Service("move_to_home", Trigger, self.move_to_home)
        self.middle_srv_ = rospy.Service("move_to_middle", Trigger, self.move_to_middle)
        self.tower_srv_ = rospy.Service("move_to_tower", MoveToTower, self.move_to_tower)
        self.remove_obj_tower_srv_ = rospy.Service("remove_object_from_tower", MoveToTower, self.remove_object_from_tower)
        self.plan_easy_grasp_srv = rospy.Service("plan_easy_grasp", PlanEasyGrasp, self.plan_easy_grasp)

        rospy.spin()
    
    def load_parameters(self) -> None:
        self.vel_scaling_ = rospy.get_param("moma_demo/arm_velocity_scaling_drop", 0.3)
        
        self.base_frame_ = rospy.get_param("moma_demo/base_frame_id", "panda_link0")
        self.object_frame_ = rospy.get_param("moma_demo/object_frame_id", "object_base")
        self.object_0_frame_ = rospy.get_param("moma_demo/object_0_frame_id", "object_0_base")
        self.object_1_frame_ = rospy.get_param("moma_demo/object_1_frame_id", "object_1_base")
        self.object_2_frame_ = rospy.get_param("moma_demo/object_2_frame_id", "object_2_base")

        self.middle_frame_ = rospy.get_param("moma_demo/middle_frame_id", "middle")
        self.tower_frame_ = rospy.get_param("moma_demo/tower_frame_id", "tower_base")

        self.object_height_ = rospy.get_param("moma_demo/object_height", 0.04)
        self.tower_height_ = rospy.get_param("moma_demo/tower_height", 0.08)
        self.drop_offset_z_ = rospy.get_param("moma_demo/drop_offset_z", 0.04)
        self.pre_grasp_offset_ = rospy.get_param("moma_demo/pre_grasp_offset", -0.04)

    def get_transform(self, target_frame: str, source_frame: str = "panda_link0"):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        try:
            transform: TransformStamped = tf_buffer.lookup_transform(
                source_frame, target_frame, rospy.Time(0), rospy.Duration(1.0)
            )    
            return transform
        except tf2_ros.LookupException:
            rospy.logerr(f"Transform from {source_frame} to {target_frame} not found")
            return None
        
    def move_to_object(self, req: TriggerRequest) -> TriggerResponse:
        """move arm to object position service handler"""
        rospy.loginfo("Moving arm to object position for pre grasp")

        target_pub = rospy.Publisher("object_target", PoseStamped, queue_size=10)
        
        # # with pre grasp offset!
        target_stamped = self.get_transform(self.object_frame_)
        target = utils.from_transform_msg(target_stamped.transform)
        target = target * Transform.translation([0.0, 0.0, -self.pre_grasp_offset_])
        pre_target = target * Transform.translation([0.0, 0.0, -0.1]) #approach from top
        
        # make sure gripper is open
        self.gripper_.release()

        # go to pre-target above object
        target_pub.publish(utils.to_pose_stamped_msg(pre_target, self.base_frame_))
        success = self.moveit_.goto(pre_target, self.vel_scaling_)

        # go to actual grasp pose
        target_pub.publish(utils.to_pose_stamped_msg(target, self.base_frame_))
        success = self.moveit_.goto(target, self.vel_scaling_)

        # grasp object
        self.gripper_.grasp()

        # go to pre-target above object
        target_pub.publish(utils.to_pose_stamped_msg(pre_target, self.base_frame_))
        success = self.moveit_.goto(pre_target, self.vel_scaling_)

        if self.arm_.has_error:
            rospy.loginfo(f"Robot error, aborting")
            return TriggerResponse(success=False)
        
        return TriggerResponse(success=success)
    
    def move_to_object_id(self, req: MoveToObjIDRequest) -> MoveToObjIDResponse:
        """move arm to object position service handler"""
        if req.obj_id == 0:
            obj_frame = self.object_0_frame_
        elif req.obj_id == 1:
            obj_frame = self.object_1_frame_
        elif req.obj_id == 2:
            obj_frame = self.object_2_frame_        
        
        rospy.loginfo("Moving arm to object position for pre grasp")

        target_pub = rospy.Publisher("object_target", PoseStamped, queue_size=10)
        
        # # with pre grasp offset!
    
        target_stamped = self.get_transform(obj_frame)
        target = utils.from_transform_msg(target_stamped.transform)
        target = target * Transform.translation([0.0, 0.0, -self.pre_grasp_offset_])
        pre_target = target * Transform.translation([0.0, 0.0, -0.1]) #approach from top
        
        # make sure gripper is open
        self.gripper_.release()

        # go to pre-target above object
        target_pub.publish(utils.to_pose_stamped_msg(pre_target, self.base_frame_))
        success = self.moveit_.goto(pre_target, self.vel_scaling_)

        # go to actual grasp pose
        target_pub.publish(utils.to_pose_stamped_msg(target, self.base_frame_))
        success = self.moveit_.goto(target, self.vel_scaling_)

        # grasp object
        self.gripper_.grasp()

        # go to pre-target above object
        target_pub.publish(utils.to_pose_stamped_msg(pre_target, self.base_frame_))
        success = self.moveit_.goto(pre_target, self.vel_scaling_)

        if self.arm_.has_error:
            rospy.loginfo(f"Robot error, aborting")
            return MoveToObjIDResponse(success=False)
        
        return MoveToObjIDResponse(success=success)
    
    def return_object(self, req: MoveToObjIDRequest) -> MoveToObjIDResponse:
        """move arm to object position service handler"""
        if req.obj_id == 0:
            obj_frame = self.object_0_frame_
        elif req.obj_id == 1:
            obj_frame = self.object_1_frame_
        elif req.obj_id == 2:
            obj_frame = self.object_2_frame_        
        
        rospy.loginfo("Moving arm to object position for pre grasp")
        
        target_pub = rospy.Publisher("object_target", PoseStamped, queue_size=10)
        
        # # with pre grasp offset!
        target_stamped = self.get_transform(obj_frame)
        target = utils.from_transform_msg(target_stamped.transform)
        target = target * Transform.translation([0.0, 0.0, -self.pre_grasp_offset_])
        pre_target = target * Transform.translation([0.0, 0.0, -0.1]) #approach from top

        # go to pre-target above object
        target_pub.publish(utils.to_pose_stamped_msg(pre_target, self.base_frame_))
        success = self.moveit_.goto(pre_target, self.vel_scaling_)

        # go to actual grasp pose
        target_pub.publish(utils.to_pose_stamped_msg(target, self.base_frame_))
        success = self.moveit_.goto(target, self.vel_scaling_)

        # release object
        self.gripper_.release()

        # go to pre-target above object
        target_pub.publish(utils.to_pose_stamped_msg(pre_target, self.base_frame_))
        success = self.moveit_.goto(pre_target, self.vel_scaling_)

        if self.arm_.has_error:
            rospy.loginfo(f"Robot error, aborting")
            return MoveToObjIDResponse(success=False)
        
        return MoveToObjIDResponse(success=success)
    

    def move_to_middle(self, req: TriggerRequest) -> TriggerResponse:
        """move arm to middle position service handler"""
        rospy.loginfo("Moving arm to middle position to measure force-torque")

        target_pub = rospy.Publisher("middle_target", PoseStamped, queue_size=10)
        
        target_stamped = self.get_transform(self.middle_frame_)
        target = utils.from_transform_msg(target_stamped.transform) # to custom transform
        target_pub.publish(utils.to_pose_stamped_msg(target, self.base_frame_))

        success = self.moveit_.goto(target, self.vel_scaling_)

        ## Here you can add extra movements or storing the FT reading

        if self.arm_.has_error:
            rospy.loginfo(f"Robot error, aborting")
            return TriggerResponse(success=False)
        
        return TriggerResponse(success=success)
        
    def move_to_home(self, req: TriggerRequest) -> TriggerResponse:
        """move arm back to safe position"""
        rospy.loginfo("Moving arm to safe position")
        success = self.moveit_.goto("ready", self.vel_scaling_)
        return TriggerResponse(success=success)

    def move_to_tower(self, req: MoveToTowerRequest) -> MoveToTowerResponse: 
        """move arm to tower service handler"""
        rospy.loginfo("Moving arm to tower offset position")

        target_pub = rospy.Publisher("tower_target", PoseStamped, queue_size=10)
        # move arm to tower pose
        
        # calc pose with y offset, drop offset, and tower_height
        target_stamped = self.get_transform(self.tower_frame_)
        target = utils.from_transform_msg(target_stamped.transform)
        target = target * Transform.translation([req.y_offset, 0.0, (-self.drop_offset_z_-self.tower_height_)])
        pre_target = target * Transform.translation([0.0, 0.0, -0.1]) #approach from top

        # go to pre-target above object
        target_pub.publish(utils.to_pose_stamped_msg(pre_target, self.base_frame_))
        success = self.moveit_.goto(pre_target, self.vel_scaling_)

        # go to actual pose
        target_pub.publish(utils.to_pose_stamped_msg(target, self.base_frame_))
        success = self.moveit_.goto(target, self.vel_scaling_)
        
        # release object
        self.gripper_.release()
        
        # go to pre-target above object
        target_pub.publish(utils.to_pose_stamped_msg(pre_target, self.base_frame_))
        success = self.moveit_.goto(pre_target, self.vel_scaling_)

        if self.arm_.has_error:
            rospy.loginfo(f"Robot error, aborting")
            return MoveToTowerResponse(success=False)
        
        return MoveToTowerResponse(success=success)
    
    def remove_object_from_tower(self, req: MoveToTowerRequest) -> MoveToTowerResponse: 
        """move arm to tower service handler"""
        rospy.loginfo("Moving arm to tower offset position")

        target_pub = rospy.Publisher("tower_target", PoseStamped, queue_size=10)
        # move arm to tower pose
        
        # calc pose with y offset, drop offset, and tower_height
        target_stamped = self.get_transform(self.tower_frame_)
        target = utils.from_transform_msg(target_stamped.transform)
        target = target * Transform.translation([req.y_offset, 0.0, (-self.drop_offset_z_-self.tower_height_)])
        pre_target = target * Transform.translation([0.0, 0.0, -0.1]) #approach from top
        
        # open gripper
        self.gripper_.release()

        # go to pre-target above object
        target_pub.publish(utils.to_pose_stamped_msg(pre_target, self.base_frame_))
        success = self.moveit_.goto(pre_target, self.vel_scaling_)

        # go to actual pose
        target_pub.publish(utils.to_pose_stamped_msg(target, self.base_frame_))
        success = self.moveit_.goto(target, self.vel_scaling_)
        
        # close gripper
        self.gripper_.grasp()
        
        # go to pre-target above object
        target_pub.publish(utils.to_pose_stamped_msg(pre_target, self.base_frame_))
        success = self.moveit_.goto(pre_target, self.vel_scaling_)

        if self.arm_.has_error:
            rospy.loginfo(f"Robot error, aborting")
            return MoveToTowerResponse(success=False)
        
        return MoveToTowerResponse(success=success)
    
    def plan_easy_grasp(self, req: PlanEasyGraspRequest) -> PlanEasyGraspResponse:
        """plan hardcoded grasp"""

        # with middle of object! -> might need to tune this value
        target_stamped = self.get_transform(self.object_frame_)
        target = utils.from_transform_msg(target_stamped.transform)
        target = target * Transform.translation([0.0, 0.0, -self.object_height_/2.0])
        pose = utils.to_pose_msg(target)        

        grasp_msg = PoseStamped()
        grasp_msg.header.stamp = rospy.Time.now()
        grasp_msg.header.frame_id = self.base_frame_
        
        grasp_msg.pose = pose

        # success = self.moveit_.goto(target, self.vel_scaling_)

        return PlanEasyGraspResponse(target_grasp_pose=grasp_msg)
    

if __name__ == "__main__":
    MoveServices()
