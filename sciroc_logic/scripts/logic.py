#!/usr/bin/env python

import rospy

# importing the labrary for the creation of the state machine
import smach
import smach_ros


# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg._GoalStatus import GoalStatus

# navigation service message
from sciroc_navigation.srv import GoToPOI

# message for updating and the getting the state of the POI (Point of Interest)
from sciroc_poi_state.srv import UpdatePOIState, GetTableObject
from sciroc_poi_state.srv import UpdatePOIStateRequest, GetTableObjectRequest

# # people perception package
# from people_perception.msg import (
#     PeopleCounterAction,
#     PeopleCounterGoal,
#     PeopleCounterResult,
# )

# human robot interaction package
from sciroc_hri.msg import HRIAction, HRIGoal, HRIResult

from sciroc_objdet.msg import (
    ObjDetInterfaceAction,
    ObjDetInterfaceGoal,
    ObjDetInterfaceResult,
)

# Importing the states for the state machines
import phase1_states, phase2_states, phase3_states

if __name__ == "__main__":
    rospy.init_node("sciroc_state_machine")

    # Create a SMACH state machine
    Trial = smach.StateMachine(outcomes=["trial_finished"])

    with Trial:
        Phase1 = smach.StateMachine(outcomes=["phase1_finished", "end_of_trial"])

        # Open the container
        with Phase1:

            # Add states to the container
            smach.StateMachine.add(
                "NAVIGATE",
                phase1_states.Navigate(),
                transitions={
                    "shop_explore_done": "HRI(Speak)",
                    "at_POI": "DETECT_PEOPLE",
                },
            )

            smach.StateMachine.add(
                "DETECT_PEOPLE",
                phase1_states.PeoplePerception(),
                transitions={
                    "people_present": "HRI(Speak)",
                    "people_not_present": "DETECT_OBJECT",
                },
            )

            smach.StateMachine.add(
                "DETECT_OBJECT",
                phase1_states.ObjectDetection(),
                transitions={"object_detect_done": "SAVE_POI_STATE"},
                remapping={"current_poi": "current_poi"},
            )

            smach.StateMachine.add(
                "HRI(Speak)",
                phase1_states.HRI(),
                transitions={
                    "greeted": "DETECT_OBJECT",
                    "announced": "phase1_finished",
                    "announced_and_done": "end_of_trial"
                },
                remapping={"current_poi": "current_poi"},
            )

            smach.StateMachine.add(
                "SAVE_POI_STATE",
                phase1_states.POI_State(),
                transitions={"saved": "NAVIGATE"},
                remapping={
                    "current_poi": "current_poi",
                    "no_of_people": "no_of_people",
                    "no_of_object": "no_of_object",
                },
            )

        smach.StateMachine.add(
            "PHASE_1", Phase1, transitions={"phase1_finished": "PHASE_2", "end_of_trial":"trial_finished"}
        )

        Phase2 = smach.StateMachine(outcomes=["phase2_finished"])

        # Open the container
        with Phase2:
            smach.StateMachine.add(
                "NAVIGATE",
                phase2_states.Navigate(),
                transitions={
                    "at_require_order_table": "HRI(TakeOrder)",
                },
            )

            smach.StateMachine.add(
                "HRI(TakeOrder)",
                phase2_states.HRI(),
                transitions={"order_taken": "UPDATE_POI_STATE"},
            )

            smach.StateMachine.add(
                "UPDATE_POI_STATE",
                phase2_states.POI_State(),
                transitions={"updated": "phase2_finished"},
                remapping={
                    "current_poi": "current_poi",
                    "order_list": "order_list",
                },
            )

        smach.StateMachine.add(
            "PHASE_2",
            Phase2,
            transitions={
                "phase2_finished": "PHASE_3",
            },
        )

        Phase3 = smach.StateMachine(outcomes=["phase3_finished", "trial_done"])
        Phase3.userdata.task = "report order"

        # Open the container
        with Phase3:
            smach.StateMachine.add(
                "NAVIGATE",
                phase3_states.Navigate(),
                transitions={
                    "at_counter": "HRI(Speak)",
                    "at_current_serving_table": "HRI(Speak)",
                    "at_default_location": "trial_done",
                },
                remapping={"task": "task"},
            )

            smach.StateMachine.add(
                "HRI(Speak)",
                phase3_states.HRI(),
                transitions={
                    "order_reported": "CHECK_OBJECT",
                    "missing_reported": "CHECK_OBJECT",
                    "wrong_reported": "CHECK_OBJECT",
                    "wrong_and_missing_order_reported": "CHECK_OBJECT",
                    "object_taken": "NAVIGATE",
                    "order_delivered": "UPDATE_POI_STATE",
                },
                remapping={
                    "task": "task",
                    "missing_drinks": "missing_drinks",
                    "wrong_drinks": "wrong_drinks",
                },
            )

            smach.StateMachine.add(
                "CHECK_OBJECT",
                phase3_states.ObjectDetection(),
                transitions={
                    "correct_order": "HRI(Speak)",
                    "wrong_order": "HRI(Speak)",
                    "missing_order": "HRI(Speak)",
                    "wrong_and_missing_order": "HRI(Speak)",
                },
                remapping={"task": "task", "current_poi": "current_poi"},
            )

            smach.StateMachine.add(
                "UPDATE_POI_STATE",
                phase3_states.POI_State(),
                transitions={
                    "updated": "phase3_finished",
                    "final_update_done": "NAVIGATE",
                },
                remapping={"current_poi": "current_poi"},
            )
        smach.StateMachine.add(
            "PHASE_3",
            Phase3,
            transitions={"phase3_finished": "PHASE_2", "trial_done": "trial_finished"},
        )

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer(
            "server_name", Trial, "SciRoc2EP1 Logic State Machine"
        )
        sis.start()

        # Execute SMACH plan
        outcome = Trial.execute()

        # Wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()
