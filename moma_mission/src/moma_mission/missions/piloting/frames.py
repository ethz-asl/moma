import rospy

class Frames:
    map_frame = "map"
    tool_frame = "tool_frame"
    base_frame = "base_link"
    odom_frame = "tracking_camera_odom"
    valve_frame = "valve"
    valve_haptic_frame = "valve_estimated"

    @classmethod
    def print_summary(cls):
        print(f"""================================
Valve opening frames data:
================================
Map frame: \t{cls.map_frame}
Odome frame: \t{cls.odom_frame}
Base frame: \t{cls.base_frame}
Valve frame: \t{cls.valve_frame}
Tool frame: \t{cls.tool_frame}
================================\n""")
