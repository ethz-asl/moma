This is to give an overview of the model description files for the mobile manipulation system used in this project.

Until 25.06.2020, a manually composed URDF file wased used. This is now replaced with `box_panda_hand.urdf.xacro` which imports from packages `moma_description` and `franka_description`. This is to keep the description files more maintainable.

To create the description files necessary for pybullet, run the script `data/models/parse_xacros.bash`. This will create all standard URDF files that pybullet expects, e.g. `box_panda_hand_pb.urdf` where all links based on ROS packages were replaced with absolute links.
