from highlevel_planning.sim.scene_move_skill import SceneMoveSkill
from highlevel_planning.skills.navigate import SkillNavigate
from highlevel_planning.skills.grasping import SkillGrasping
from highlevel_planning.skills.move import SkillMove
from highlevel_planning.tools.config import ConfigYaml
from highlevel_planning.tools import run_util

import numpy as np
import os

EPS = 1e-6
BASEDIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def drawer_example(sk_grasp, sk_nav, robot, scene, world):
    # Run move skill
    sk_nav.move_to_object("cupboard")

    print(robot.get_wrist_force_torque())

    # Grasp the cupboard handle
    res = sk_grasp.grasp_object("cupboard", link_idx=3)
    if not res:
        print("Grasping the handle failed.")
        return

    # Drive back
    robot.update_velocity([-0.1, 0.0, 0.0], 0.0)
    world.step_seconds(2)
    robot.stop_driving()

    # Release
    sk_grasp.release_object()
    robot.to_start()


def drawer_example_auto(sk_grasp, sk_nav, sk_move, robot, scene):
    # Run move skill
    sk_nav.move_to_object("cupboard", nav_min_dist=1.0)

    # Grasp the cupboard handle
    res = sk_grasp.grasp_object("cupboard", link_idx=3)
    if not res:
        print("Grasping the handle failed.")
        return

    # Run the move skill
    sk_move.move_object(0.3, np.array([0.5, 0.0, -1.0]))
    # sk_move.move_object(0.3, np.array([-0.5, 0.0, -0.5]))     # GT

    # Release
    sk_grasp.release_object()
    robot.to_start()

def main():
    # Command line arguments
    args = run_util.parse_arguments()

    # Load existing simulation data if desired
    savedir = os.path.join(BASEDIR, "data", "sim")
    objects, robot_mdl = run_util.restore_pybullet_sim(savedir, args)

    # Load config file
    cfg = ConfigYaml(os.path.join(BASEDIR, "config", "main.yaml"))

    # Create world
    robot, scene = run_util.setup_pybullet_world(
        SceneMoveSkill, BASEDIR, savedir, objects, args, cfg, robot_mdl
    )

    # Set up skills
    sk_grasp = SkillGrasping(scene, robot, cfg)
    sk_nav = SkillNavigate(scene, robot)
    sk_move = SkillMove(scene, robot, 0.02, robot._world.T_s)
    
    # ---------- Run examples -----------
	
    # drawer_example(sk_grasp, sk_nav, robot, scene, world)
    drawer_example_auto(sk_grasp, sk_nav, sk_move, robot, scene)

    # -----------------------------------

    robot._world.step_seconds(50)


if __name__ == "__main__":
    main()
