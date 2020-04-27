def create_robot_connection(robot_name, simulation_mode=False):
    """Connect to the robot.

    Args:
        robot_name (str, ["panda", "yumi_left_arm", "yumi_right_arm"]): The name of the robot.
    """
    if robot_name == "panda":
        from panda_control.panda_commander import PandaCommander

        return PandaCommander(simulation_mode=simulation_mode)
    elif robot_name == "yumi_left_arm":
        if simulation_mode:
            raise NotImplementedError

        from yumi_controllers.yumi_commander import YumiCommander

        return YumiCommander().left_arm
    elif robot_name == "yumi_right_arm":
        if simulation_mode:
            raise NotImplementedError

        from yumi_controllers.yumi_commander import YumiCommander

        return YumiCommander().right_arm
    else:
        raise ValueError("Invalid robot_name")
