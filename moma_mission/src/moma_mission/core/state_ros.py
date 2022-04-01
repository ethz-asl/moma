#!/usr/bin/env python
import smach
import rospy
import tf2_ros
import easydict as edict
import pinocchio as pin
from os.path import join

from moma_mission.utils import ros
from moma_mission.utils.transforms import tf_to_se3, pose_to_se3


class StateMachineContext(object):
    def __init__(self):
        self.ctx = edict.EasyDict()


# A global context accessible to all states inheriting from this state
global_context = StateMachineContext()


class StateRos(smach.State):
    """
    Base state adding minimal ROS functionality to the basic smach state
    """

    def __init__(
        self, outcomes=["Completed", "Failure"], input_keys=[], output_keys=[], ns=""
    ):
        smach.State.__init__(
            self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys
        )
        self.namespace = ns
        global global_context
        self.global_context = global_context

        # parse optional default outcome
        self.default_outcome = self.get_scoped_param("default_outcome", safe=True)
        if self.default_outcome not in outcomes + ["None"]:
            raise NameError(
                "Default outcome must be one of {}".format(outcomes + ["None"])
            )

        if self.default_outcome == "None":
            self.default_outcome = None

        if (
            self.default_outcome
            and self.default_outcome not in self.get_registered_outcomes()
        ):
            raise NameError(
                "{} is not in default outcomes".format(self.default_outcome)
            )  # prevent accidental typos

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def get_scoped_param(self, param_name, default=None, safe=True):
        """
        Get the parameter namespaced under the state name
        e.g get_scoped_param('/my_param') --> looks for '/state_name/my_param'
            get_scoped_param('my_param') --> looks for '/ros_namespace/state_name/my_param'
        """
        if param_name.startswith("/"):
            named_param = join(self.namespace, param_name)
        else:
            named_param = join(rospy.get_namespace(), self.namespace, param_name)

        if safe:
            return (
                ros.get_param_safe(named_param)
                if default is None
                else ros.get_param_safe(named_param, default)
            )
        else:
            if rospy.has_param(named_param):
                return (
                    rospy.get_param(named_param)
                    if default is None
                    else rospy.get_param(named_param, default)
                )
            else:
                return None

    def execute(self, ud):
        if self.default_outcome:
            rospy.loginfo(
                "{state} ==> {outcome} (using default outcome)".format(
                    state=self.namespace, outcome=self.default_outcome
                )
            )
            return self.default_outcome

        return self.run()

    def run(self):
        raise NotImplementedError("run must be implemented by the inheriting state")

    def set_context(self, key, data, overwrite=False):
        """
        Set a new data field in the global context accessible to all states which
        are derived from this state
        :param key: data key
        :param data: data content
        :param overwrite: if True, let the user overwrite data if already in global context
        :return: True if read was successful
        """
        if not overwrite and key in self.global_context.ctx.keys():
            rospy.logwarn(
                "Could not set data in global context. Key {} already exists".format(
                    key
                )
            )
            return False
        self.global_context.ctx[key] = data

    def get_transform(self, target, source):
        """Retrieve transform. Let it fail if unable to get the transform"""
        transform = self.tf_buffer.lookup_transform(
            target,
            source,
            rospy.Time(0),  # tf at first available time
            rospy.Duration(3),
        )
        return tf_to_se3(transform)

    def wait_until_reached(
        self,
        target_frame,
        target_pose,
        linear_tolerance=0.02,
        angular_tolerance=0.1,
        timeout=200,
        quiet=False,
    ):
        """
        Returns once the target pose has been reached
        """
        rospy.loginfo(
            "Reaching target ... linear tol={}, angular tol={}".format(
                linear_tolerance, angular_tolerance
            )
        )
        tolerance_met = False
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if tolerance_met:
                return True

            t_current = self.get_transform(
                target=target_pose.header.frame_id, source=target_frame
            )
            t_desired = pose_to_se3(target_pose.pose)
            error = pin.log6(
                t_current.actInv(t_desired)
            )  # motion that brings in 1 sec ee to target
            linear_error = max(abs(error.linear))
            angular_error = max(abs(error.angular))
            if linear_error < linear_tolerance and angular_error < angular_tolerance:
                tolerance_met = True

            elapsed = (rospy.Time.now() - start_time).secs
            rospy.loginfo_throttle(
                3.0,
                "Reaching target ... lin_err={}, ang_err={}, elapsed={}, timeout={}".format(
                    linear_error, angular_error, elapsed, timeout
                ),
            )

            if timeout != 0 and elapsed > timeout:
                if quiet:
                    rospy.logwarn(
                        "Timeout elapsed while reaching a pose. Current distance to target is: {}".format(
                            linear_error
                        )
                    )
                    return True
                else:
                    rospy.logerr("Timeout elapsed while reaching a pose")
                    return False

            rate.sleep()


if __name__ == "__main__":

    class StateA(StateRos):
        def __init__(self, ns=""):
            StateRos.__init__(self, ns="state_a")

        def run(self):
            try:
                param = self.get_scoped_param("my_param")
                self.set_context("a_data", param)
            except NameError as exc:
                rospy.logerr(exc)
                return "Failure"
            return "Completed"

    class StateB(StateRos):
        def __init__(self):
            StateRos.__init__(self, ns="state_b")

        def run(self):
            value = self.global_context.ctx.a_data
            rospy.loginfo("value is {}".format(value))
            return "Completed"

    class StateC(StateRos):
        def __init__(self):
            StateRos.__init__(self, ns="state_c")

        def run(self):
            print("I should not print because of default transition")
            return "Completed"

    rospy.init_node("state_ros_test")
    rospy.set_param("state_a/my_param", 1)
    rospy.set_param("state_a/default_outcome", "None")

    rospy.set_param("state_b/default_outcome", "None")

    rospy.set_param("state_c/default_outcome", "Failure")

    a = StateA()
    a.execute(ud=None)

    b = StateB()
    b.execute(ud=None)

    c = StateC()
    c.execute(ud=None)
