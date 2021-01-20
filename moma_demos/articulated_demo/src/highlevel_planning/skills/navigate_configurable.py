import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation as R
from highlevel_planning.tools.util import (
    homogenous_trafo,
    invert_hom_trafo,
    pos_and_orient_from_hom_trafo,
)

class SkillNavigate:
    def __init__(self, scene, robot, offset_pos, offset_angle):
    
        self.robot_ = robot
        self.robot_uid_ = robot.model.uid
        self.scene_ = scene
        
        self.offset_pos = offset_pos
        self.offset_angle = offset_angle

    def _check_collisions(self):
        for _, obj in self.scene_.objects.items():
            temp = p.getClosestPoints(self.robot_uid_, obj.model.uid, distance=0.5)
            for elem in temp:
                contact_distance = elem[8]
                if contact_distance < 0.0:
                    # print("There is a collision")
                    return True
        return False

    def _move(self, pos, orient):
        p.resetBasePositionAndOrientation(
            self.robot_uid_, pos.tolist(), orient.tolist()
        )

    def move_to_object(self, target_name, nav_min_dist=None):
        target_id = self.scene_.objects[target_name].model.uid

        # Get the object position
        temp = p.getBasePositionAndOrientation(target_id)
        target_pos = np.array(temp[0]) + np.array(self.offset_pos)					# It is positioning with offset

        # Get valid nav angles
        nav_angle = self.scene_.objects[target_name].nav_angle
        if nav_min_dist is None:
            nav_min_dist = self.scene_.objects[target_name].nav_min_dist
        
        # ----- nav_min_dist adjustment based on articulated object type -----
        
        if target_name == 'roomdoor':
            
            nav_min_dist = nav_min_dist - 0.25
            
        if target_name == 'slidingdoor':
            
            nav_min_dist = nav_min_dist - 0.25
            
        if target_name == 'slidinglid':
            
            nav_min_dist = nav_min_dist - 0.7
            
        if target_name == 'removablelid':
            
            nav_min_dist = nav_min_dist - 0.7
            
        if target_name == 'dishwasher':
            
            nav_min_dist = nav_min_dist - 0.25
            
        # Move there
        return self.move_to_pos(target_name, target_id, target_pos, nav_angle, nav_min_dist)

    def move_to_pos(self, target_name, target_id, target_pos, nav_angle=None, nav_min_dist=None):
        assert len(target_pos) == 3
        assert type(target_pos) is np.ndarray
        
        nj = p.getNumJoints(target_id)
        
        self.robot_.to_start()
               
        # Get robot position
        temp = p.getBasePositionAndOrientation(self.robot_uid_)
        robot_pos = np.array(temp[0])
        robot_orient = R.from_quat(temp[1])

        # Get position and orientation of any object in the robot hand w.r.t the robot base
        object_in_hand_uid = self._find_object_in_hand()
        T_rob_obj = self._get_object_relative_pose(
            object_in_hand_uid, robot_pos, robot_orient
        )

        if nav_angle is None:
            alphas = np.arange(0.0, 2.0 * np.pi, 2.0 * np.pi / 10.0)
        else:
            alphas = np.array([nav_angle])
        if nav_min_dist is None:
            radii = np.arange(0.4, 2.0, 0.1)
        else:
            radii = nav_min_dist + np.arange(0.4, 2.0, 0.1)

        # Iterate through points on circles around the target
        # First vary the radius
        for r in radii:

            # Then vary the angle
            for alpha in alphas:
                
                for counter in range(nj):
                    p.resetJointState(target_id, counter, -0.0, 0.0) 

                direction_vec = np.array([np.cos(alpha), np.sin(alpha), 0])
                robot_pos[:2] = target_pos[:2] + r * direction_vec[:2]
                rotation = R.from_euler("z", np.pi + alpha + self.offset_angle, degrees=False)		# Here offset angle is added
                robot_orient = rotation.as_quat()						
                print("robot pose: ", robot_pos)
                # Put robot into this position
                self._move(robot_pos, robot_orient)
                if not self._check_collisions():
                    # Move object into robot's hand
                    self._set_object_relative_pose(
                        object_in_hand_uid, robot_pos, robot_orient, T_rob_obj
                    )

                    return True
        return False
        
#-----------------------------------------------------------------------------------

    def move_to_object_and_grasp(self, target_name, link_idx, grasp_id, sk_grasp, nav_min_dist=None):
        target_id = self.scene_.objects[target_name].model.uid

        # Get the object position
        temp = p.getBasePositionAndOrientation(target_id)
        target_pos = np.array(temp[0]) + np.array(self.offset_pos)					# It is positioning with offset

        # Get valid nav angles
        #nav_angle = self.scene_.objects[target_name].nav_angle
        nav_angle = None
        
        if nav_min_dist is None:
            nav_min_dist = self.scene_.objects[target_name].nav_min_dist

        # Move there
        return self.move_to_pos_and_grasp(target_pos, target_name, link_idx, grasp_id, sk_grasp, nav_angle, nav_min_dist)
        
    def move_to_pos_and_grasp(self, target_pos, target_name, link_idx, grasp_id, sk_grasp, nav_angle=None, nav_min_dist=None):
        assert len(target_pos) == 3
        assert type(target_pos) is np.ndarray      

        self.robot_.to_start()

        # Get robot position
        temp = p.getBasePositionAndOrientation(self.robot_uid_)
        robot_pos = np.array(temp[0])
        robot_orient = R.from_quat(temp[1])

        # Get position and orientation of any object in the robot hand w.r.t the robot base
        object_in_hand_uid = self._find_object_in_hand()
        T_rob_obj = self._get_object_relative_pose(
            object_in_hand_uid, robot_pos, robot_orient
        )

        if nav_angle is None:
            alphas = np.arange(np.pi, 2.0 * np.pi, 2.0 * np.pi / 10.0)
        else:
            alphas = np.array([nav_angle])
        if nav_min_dist is None:
            radii = np.arange(0.4, 2.0, 0.1)
        else:
            radii = nav_min_dist + np.arange(-0.2, 2.0, 0.1)

        # Iterate through points on circles around the target
        # First vary the radius
        print(nav_angle)
        for r in radii:
            # Then vary the angle
            print("Current radii: ", r)
            print("Alphas: ", alphas)
            for alpha in alphas:
                print("Current alpha: ", alpha)
                direction_vec = np.array([np.cos(alpha), np.sin(alpha), 0])
                robot_pos[:2] = target_pos[:2] + r * direction_vec[:2]
                rotation = R.from_euler("z", np.pi + alpha + self.offset_angle, degrees=False)		# Here offset angle is added
                robot_orient = rotation.as_quat()						

                # Put robot into this position
                self._move(robot_pos, robot_orient)
                pos, orient = sk_grasp.compute_grasp(target_name, link_idx, grasp_id)
                pos_pre = pos - np.matmul(
                    R.from_quat(orient).as_dcm(), np.array([0.0, 0.0, sk_grasp._pregrasp_z_offset])
                    )
                pos_pre_joints = self.robot_.ik(pos_pre, orient)
                    
                if not self._check_collisions() and not(pos_pre_joints.tolist() is None):
                    # Move object into robot's hand
                    self._set_object_relative_pose(
                        object_in_hand_uid, robot_pos, robot_orient, T_rob_obj
                    )

                    return True
        return False
        
#-----------------------------------------------------------------------
    def _find_object_in_hand(self):
        # Determine which object is in the robot's hand
        object_in_hand_uid = None
        for _, obj in self.scene_.objects.items():
            temp = p.getClosestPoints(
                self.robot_uid_,
                obj.model.uid,
                distance=0.01,
                linkIndexA=self.robot_.link_name_to_index["panda_leftfinger"],
            )
            if len(temp) > 0:
                if object_in_hand_uid is not None:
                    raise RuntimeError(
                        "Don't know how to deal with more than one object in robot's hand"
                    )
                object_in_hand_uid = obj.model.uid
        return object_in_hand_uid

    def _get_object_relative_pose(self, object_in_hand_uid, robot_pos, robot_orient):
        T_rob_obj = None
        if object_in_hand_uid is not None:
            # Get object position
            temp = p.getBasePositionAndOrientation(object_in_hand_uid)
            held_object_pos = np.array(temp[0])
            held_object_orient = R.from_quat(temp[1])

            # Compute object pose relative to robot
            r_O_O_obj = held_object_pos
            C_O_obj = held_object_orient
            T_O_obj = homogenous_trafo(r_O_O_obj, C_O_obj)
            r_O_O_rob = robot_pos
            C_O_rob = robot_orient
            T_O_rob = homogenous_trafo(r_O_O_rob, C_O_rob)

            T_rob_obj = np.matmul(invert_hom_trafo(T_O_rob), T_O_obj)

            # Check result
            T_test = np.matmul(T_O_rob, T_rob_obj)
            assert np.all(T_test - T_O_obj < 1e-12)

        return T_rob_obj

    def _set_object_relative_pose(
        self, object_in_hand_uid, robot_pos, robot_orient, T_rob_obj
    ):
        if object_in_hand_uid is not None:
            r_O_O_rob = robot_pos
            C_O_rob = R.from_quat(robot_orient)
            T_O_rob = homogenous_trafo(r_O_O_rob, C_O_rob)

            T_O_obj = np.matmul(T_O_rob, T_rob_obj)
            (held_object_pos, held_object_orient) = pos_and_orient_from_hom_trafo(
                T_O_obj
            )
            p.resetBasePositionAndOrientation(
                object_in_hand_uid,
                held_object_pos.tolist(),
                held_object_orient.tolist(),
            )


def get_nav_in_reach_description():
    action_name = "nav-in-reach"
    action_params = [
        ["current_pos", "navgoal"],
        ["goal_pos", "navgoal"],
        ["rob", "robot"],
    ]
    action_preconditions = [
        ("at", True, ["current_pos", "rob"]),
        ("has-grasp", True, ["goal_pos"]),
    ]
    action_effects = [
        ("in-reach", True, ["goal_pos", "rob"]),
        ("in-reach", False, ["current_pos", "rob"]),
        ("at", True, ["goal_pos", "rob"]),
        ("at", False, ["current_pos", "rob"]),
    ]
    return (
        action_name,
        {
            "params": action_params,
            "preconds": action_preconditions,
            "effects": action_effects,
        },
    )


def get_nav_at_description():
    action_name = "nav-at"
    action_params = [
        ["current_pos", "navgoal"],
        ["goal_pos", "navgoal"],
        ["rob", "robot"],
    ]
    action_preconditions = [("at", True, ["current_pos", "rob"])]
    action_effects = [
        ("at", True, ["goal_pos", "rob"]),
        ("at", False, ["current_pos", "rob"]),
        ("in-reach", False, ["current_pos", "rob"]),
    ]
    return (
        action_name,
        {
            "params": action_params,
            "preconds": action_preconditions,
            "effects": action_effects,
        },
    )
