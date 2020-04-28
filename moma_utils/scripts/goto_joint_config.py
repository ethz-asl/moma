from grasp_demo.utils import create_robot_connection

joints = [
    -0.36651914,
    -1.79768913,
    1.04719755,
    1.79768913,
    0.27925268,
    0.2443461,
    1.34390352,
]

right_arm = create_robot_connection("yumi_right_arm")
right_arm.goto_joint_target(joints, max_velocity_scaling=0.5)

print("Should have reached joint target now.")
