#!/usr/bin/env python

import rospy
import smach
from states import Navigate, POI_State, HRI, PeoplePerception, ObjectDetection

if __name__ == '__main__':
    rospy.init_node('sciroc_state_machine')

    poi = ['counter', 'table1', 'table2',
           'table3', 'table4', 'table5', 'table6']

    # Create a SMACH state machine
    Trial = smach.StateMachine(outcomes=['trial_finished'])

    with Trial:

        Phase1 = smach.StateMachine(
            outcomes=['phase1_finished'])
        Phase1.userdata.phase_value = 1

        # Open the container
        with Phase1:

            # Add states to the container
            smach.StateMachine.add('NAVIGATE', Navigate(poi),
                                   transitions={
                                       'shop_explore_done': 'HRI(Speak)', 'at_POI': 'DETECT_PEOPLE'},
                                   remapping={})

            smach.StateMachine.add('DETECT_PEOPLE',
                                   PeoplePerception(),
                                   transitions={
                                       'people_present': 'HRI(Speak)', 'people_not_precent': 'DETECT_OBJECT'},
                                   remapping={})

            smach.StateMachine.add('DETECT_OBJECT',
                                   ObjectDetection(),
                                   transitions={
                                       'object_detect_done': 'SAVE_POI_STATE'},
                                   remapping={})

            smach.StateMachine.add('HRI(Speak)',
                                   HRI(),
                                   transitions={
                                       'greeted': 'DETECT_OBJECT', 'announced': 'phase1_finished'},
                                   remapping={})

            smach.StateMachine.add('SAVE_POI_STATE',
                                   POI_State(),
                                   transitions={'saved': 'NAVIGATE'},
                                   remapping={})

        smach.StateMachine.add('PHASE_1', Phase1,
                               transitions={'phase1_finished': 'PHASE_2'})

        Phase2 = smach.StateMachine(
            outcomes=['phase2_finished', 'at_default_location'])
        Phase2.userdata.phase_value = 2

        # Open the container
        with Phase2:
            smach.StateMachine.add('NAVIGATE',
                                   Navigate(poi),
                                   transitions={
                                       'at_require_order_table': 'HRI(TakeOrder)', 'at_default_location': 'at_default_location'},
                                   remapping={})

            smach.StateMachine.add('HRI(TakeOrder)',
                                   HRI(),
                                   transitions={
                                       'order_taken': 'UPDATE_POI_STATE'},
                                   remapping={})

            smach.StateMachine.add('UPDATE_POI_STATE',
                                   POI_State(),
                                   transitions={'updated': 'phase2_finished'},
                                   remapping={})

        smach.StateMachine.add('PHASE_2', Phase2,
                               transitions={'phase2_finished': 'PHASE_3', 'at_default_location': 'trial_finished'})

        Phase3 = smach.StateMachine(
            outcomes=['phase3_finished'])
        Phase3.userdata.phase_value = 3
        Phase3.userdata.task = 'report order'

        # Open the container
        with Phase3:
            smach.StateMachine.add('NAVIGATION',
                                   Navigate(poi),
                                   transitions={
                                       'at_counter': 'HRI(Speak)', 'at_current_serving_table': 'HRI(Speak)'},
                                   remapping={})

            smach.StateMachine.add('HRI(Speak)',
                                   HRI(),
                                   transitions={'order_reported': 'CHECK_OBJECT', 'missing_reported': 'CHECK_OBJECT',
                                                'wrong_reported': 'CHECK_OBJECT', 'object_taken': 'NAVIGATE', 'order_delivered': 'UPDATE_POI_STATE'},
                                   remapping={})

            smach.StateMachine.add('CHECK_OBJECT',
                                   ObjectDetection(),
                                   transitions={
                                       'correct_order': 'HRI(Speak)', 'wrong_order': 'HRI(Speak)', 'missing_order': 'HRI(Speak)'},
                                   remapping={})

            smach.StateMachine.add('UPDATE_POI_STATE',
                                   POI_State(),
                                   transitions={'updated': 'phase3_finished'},
                                   remapping={})

        smach.StateMachine.add('PHASE_3', Phase3,
                               transitions={'phase3_finished': 'PHASE_2'})

    # Execute SMACH plan
    outcome = Trial.execute()
