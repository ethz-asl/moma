import argparse
import os
import pybullet as p
import pickle
import numpy as np

from highlevel_planning.sim.world import WorldPybullet
from highlevel_planning.sim.robot_arm import RobotArmPybullet
from highlevel_planning.skills import pddl_descriptions


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-r",
        "--reuse-objects",
        action="store_true",
        help="if given, the simulation does not reload objects. Objects must already be present.",
    )
    parser.add_argument(
        "-s",
        "--sleep",
        action="store_true",
        help="if given, the simulation will sleep for each update step, to mimic real time execution.",
    )
    parser.add_argument(
        "-m",
        "--method",
        action="store",
        help="determines in which mode to connect to pybullet. Can be 'gui', 'direct' or 'shared'.",
        default="gui",
    )
    args = parser.parse_args()
    return args


def restore_pybullet_sim(savedir, args):
    objects = None
    robot_mdl = None
    if args.reuse_objects:
        with open(os.path.join(savedir, "objects.pkl"), "rb") as pkl_file:
            objects, robot_mdl = pickle.load(pkl_file)
    return objects, robot_mdl


def setup_pybullet_world(scene_object, basedir, savedir, objects, args, cfg, robot_mdl):
    # Create world
    world = WorldPybullet(
        style=args.method,
        sleep=args.sleep,
        load_objects=not args.reuse_objects,
        savedir=savedir,
    )
    scene = scene_object(world, basedir, restored_objects=objects)

    # Spawn robot
    robot = RobotArmPybullet(world, cfg, basedir, robot_mdl)
    robot.reset()

    robot.to_start()
    world.step_seconds(0.5)

    # Save world
    if not args.reuse_objects:
        if not os.path.isdir(savedir):
            os.makedirs(savedir)
        with open(os.path.join(savedir, "objects.pkl"), "wb") as output:
            pickle.dump((scene.objects, robot.model), output)
        p.saveBullet(os.path.join(savedir, "state.bullet"))

    return robot, scene
