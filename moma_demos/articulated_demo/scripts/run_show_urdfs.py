from highlevel_planning.sim.world import WorldPybullet

import os


def main():
    
    # Create world
    world = WorldPybullet(style="gui", sleep=True)
    world.add_plane()

    # ======== Cupboard ====================================

    #cupboard_mdl = world.add_model(
    #    os.path.join(os.getcwd(), "data/models/cupboard_drawers/cupboard_drawers.urdf"),
    #    position=[0.0, 0.0, 0.0],
    #    orientation=[0.0, 0.0, 0.0, 1.0],
    #)

    #drawer_link_idx = []
    #for i in range(p.getNumJoints(cupboard_mdl.uid)):
    #    info = p.getJointInfo(cupboard_mdl.uid, i)
    #    joint_name = info[1]
    #    if "drawer_joint" in joint_name and len(joint_name) == 13:
    #        drawer_link_idx.append(i)

    #for i in drawer_link_idx:
    #    p.setJointMotorControl2(
    #        cupboard_mdl.uid, i, controlMode=p.VELOCITY_CONTROL, force=0.0
    #    )

    # ======== Container ======================================
    
    #container_mdl = world.add_model("data/models/container/container_no_lid.urdf", position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0,1.0])
    
    #container_mdl = world.add_model(os.path.join(os.getcwd(), "moma/moma_demos/articulated_demo/data/models/container/container_no_lid.urdf"), position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0,1.0])
    
    #lid_mdl = world.add_model("data/models/container/lid.urdf", position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0,1.0])
    
    #lid_mdl = world.add_model(os.path.join(os.getcwd(), "moma/moma_demos/articulated_demo/data/models/container/lid.urdf"), position=[0.0, 0.0, 0.2], orientation=[0.0, 0.0, 0.0,1.0])

    #container_sliding_mdl = world.add_model("data/models/container/container_sliding_lid.urdf", position=[0.0, 0.5, 0.0], 	orientation=[0.0, 0.0, 0.0,1.0])
    
    #container_sliding_mdl = world.add_model(os.path.join(os.getcwd(), "moma/moma_demos/articulated_demo/data/models/container/container_sliding_lid.urdf"), position=[0.0, 0.5, 0.0], 	orientation=[0.0, 0.0, 0.0,1.0])
    
    door_mdl = world.add_model(os.path.join(os.getcwd(), "moma/moma_demos/articulated_demo/data/models/door/door.urdf"), position=[1, 0.0, 0.0], orientation=[0.0, 0.0, 0.0,1.0])

    dishwasher_mdl = world.add_model(os.path.join(os.getcwd(), "moma/moma_demos/articulated_demo/data/models/dishwasher/dishwasher_door.urdf"), position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0,1.0])
    
    #sliding_door_mdl = world.add_model(os.path.join(os.getcwd(), "moma/moma_demos/articulated_demo/data/models/sliding_door/sliding_door.urdf"), position=[3, 0.0, 0.0], orientation=[0.0, 0.0, 0.0,1.0])
    
    #container_removable_mdl = world.add_model(os.path.join(os.getcwd(), "moma/moma_demos/articulated_demo/data/models/container/container_removable_lid.urdf"), position=[0.0, 0.0, 0.0], 	orientation=[0.0, 0.0, 0.0,1.0])
    
    #container_sliding_lid_on_table_mdl= world.add_model(os.path.join(os.getcwd(), "moma/moma_demos/articulated_demo/data/models/container/container_sliding_lid_on_table.urdf"), position=[0.0, -1.0, 0.0], 	orientation=[0.0, 0.0, 0.0,1.0])

    #container_removable_lid_on_table_mdl= world.add_model(os.path.join(os.getcwd(), "moma/moma_demos/articulated_demo/data/models/container/container_removable_lid_on_table.urdf"), position=[0.0, 1.0 , 0.0], 	orientation=[0.0, 0.0, 0.0,1.0])
    
    world.step_seconds(50)

    world.close()


if __name__ == "__main__":
    main()
