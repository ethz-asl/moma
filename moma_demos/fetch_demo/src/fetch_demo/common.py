import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from moma_utils.ros_conversions import waypoint_to_pose_msg


class MovingActionServer(object):
    def __init__(self, action_name, ActionType):
        # Connection to navigation stack
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        self.move_base_client.wait_for_server()

        self.action_server = actionlib.SimpleActionServer(
            action_name, ActionType, execute_cb=self.action_callback, auto_start=False
        )
        self.action_server.start()
        rospy.loginfo(action_name + " action server started.")

    def action_callback(self):
        raise NotImplementedError("Should be implemented by subclass")

    def _visit_waypoint(self, waypoint):
        """
            Given a waypoint in the format [position x (m), position y (m), yaw (degrees)], this function
            invokes move_base to navigate the robot to the waypoint.
        """
        pose = waypoint_to_pose_msg(waypoint)
        navigation_goal = MoveBaseGoal(target_pose=pose)
        navigation_goal.target_pose.header.frame_id = "map"
        self.move_base_client.send_goal(navigation_goal)
        state = GoalStatus.PENDING
        while state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
            if self.action_server.is_preempt_requested():
                self.move_base_client.cancel_all_goals()
                self.action_server.set_preempted()
                return False
            rospy.sleep(0.25)
            state = self.move_base_client.get_state()

        if state is not GoalStatus.SUCCEEDED:
            return False

        rospy.loginfo("Reached approach waypoint.")
        return True

