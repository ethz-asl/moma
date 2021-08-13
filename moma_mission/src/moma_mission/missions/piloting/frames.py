import rospy

class Frames:
    map_frame = "map"
    tool_frame = "end_effector"
    base_frame = "arm_base"
    valve_frame = "valve"
    valve_haptic_frame = "valve_estimated"

    @classmethod
    def init_from_ros(cls):
        try:
            cls.map_frame = rospy.get_param("~map_frame")
            cls.valve_frame = rospy.get_param("~valve_frame")
            cls.tool_frame = rospy.get_param("~tool_frame")
            cls.base_frame = rospy.get_param("~base_frame")
            return True
        except Exception as exc:
            rospy.logerr(exc)
            return False

    @classmethod
    def print_summary(cls):
        print("""================================\n
                 Valve opening frames data:\n
                 ================================\n
                 Map frame: \t{cls.map_frame}\n
                 Valve frame: \t{cls.valve_frame}\n
                 Base frame: \t{cls.base_frame}\n
                 Tool frame: \t{cls.tool_frame}\n
                 ================================\n""")
