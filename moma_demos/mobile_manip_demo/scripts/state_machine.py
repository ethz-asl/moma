#!/usr/bin/env python

"""Build the FSM."""

from mobile_manip_demo import states
import rospy
import smach
import smach_ros


def state_machine(cube_ID: int):

    rospy.init_node("state_machine")

    # Parameters
    delivery = rospy.get_param("moma_demo/delivery_station")
    place_pose = rospy.get_param("moma_demo/place_pose")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["SUCCESS"])

    with sm:
        smach.StateMachine.add(
            f"Move-To cube{cube_ID}",
            states.Move(
                name=f"Move-To cube{cube_ID}",
                goal_ID=cube_ID,
                ref_frame="map",
                goal_pose=None,
                outcomes=[f"Cube{cube_ID} reached"],
            ),
            transitions={f"Cube{cube_ID} reached": f"Pick cube{cube_ID}"},
        )

        smach.StateMachine.add(
            f"Pick cube{cube_ID}",
            states.Pick(
                name=f"Pick cube{cube_ID}",
                goal_ID=cube_ID,
                goal_pose=None,
                outcomes=[f"Cube{cube_ID} in Hand"],
            ),
            transitions={f"Cube{cube_ID} in Hand": "Move-To delivery"},
        )

        smach.StateMachine.add(
            f"Move-To delivery",
            states.Move(
                name=f"Move-To delivery",
                goal_ID=None,
                ref_frame="map",
                goal_pose=delivery,
                outcomes=[f"Delivery pose reached"],
            ),
            transitions={f"Delivery pose reached": f"Place cube{cube_ID}"},
        )

        smach.StateMachine.add(
            f"Place cube{cube_ID}",
            states.Place(
                name=f"Place cube{cube_ID}",
                goal_ID=cube_ID,
                goal_pose=place_pose,
                outcomes=[f"Cube{cube_ID} palced"],
            ),
            transitions={f"Cube{cube_ID} palced": "SUCCESS"},
        )

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer("server_name", sm, "/SM_ROOT")
    sis.start()

    # Execute SMACH plan
    # outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    state_machine(2)
