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
            cls.map_frame = rospy.get_param("/piloting/map_frame")
            cls.valve_frame = rospy.get_param("/piloting/valve_frame")
            cls.tool_frame = rospy.get_param("/piloting/tool_frame")
            cls.base_frame = rospy.get_param("/piloting/base_frame")
            cls.odom_frame = rospy.get_param("/piloting/odom_frame")
            return True
        except Exception as exc:
            rospy.logerr(exc)
            return False

    @classmethod
    def print_summary(cls):
        print(f"""================================
Valve opening frames data:
================================
Map frame: \t{cls.map_frame}
Valve frame: \t{cls.valve_frame}
Base frame: \t{cls.base_frame}
Tool frame: \t{cls.tool_frame}
================================\n""")
