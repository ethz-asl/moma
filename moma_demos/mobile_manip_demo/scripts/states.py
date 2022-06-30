"""Define SMACH states for the FSM."""

from mobile_manip_demo import robot_interface
import smach


# define state Grasp
class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["cube in hand"],
        )

    def execute(self, userdata):
        rospy.loginfo("Executing state GRASP")
        return "cube in hand"


# define state Navigate
class Navigate_to_Cube(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["reach cube1"],
        )

    def execute(self, userdata):
        rospy.loginfo("Executing state NAVIGATE to cube")
        return "reach cube1"


# define state Navigate
class Navigate_to_Place(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["reach place pose"],
        )

    def execute(self, userdata):
        rospy.loginfo("Executing state NAVIGATE to place pose")
        return "reach place pose"


# define state Place
class Place(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["cube on cupboard"],
        )

    def execute(self, userdata):
        rospy.loginfo("Executing state PLACE")
        return "cube on cupboard"
