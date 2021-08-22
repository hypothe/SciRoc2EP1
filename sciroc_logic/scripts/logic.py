#!/usr/bin/env python

import rospy
import smach
from states import Navigate, POI_State, HRI, PeoplePerception, ObjectDetection

if __name__ == '__main__':
    rospy.init_node('sciroc_state_machine')

    poi = ['counter', 'table1', 'table2',
           'table3', 'table4', 'table5', 'table6']

    # Create a SMACH state machine
    Trial = smach.StateMachine(outcomes=['finished', 'failed'])

    with Trial:

        Phase1 = smach.StateMachine(
            outcomes=['finished', 'failed'])
        Phase1.userdata.phase_value = 1

        # Open the container
        with Phase1:

            # Add states to the container
            smach.StateMachine.add('NAVIGATION', Navigate(
                poi), transitions={'shop_explore_done': 'HRI', 'at_POI': 'PEOPLE_PERCEPTION'},
                remapping={'phase_no': 'phase_value'})

            smach.StateMachine.add('PEOPLE_PERCEPTION',
                                   people_percept_state,
                                   transitions={
                                       'people_present': 'HRI', 'people_not_precent': 'OBJECT_PERCEPTION'})

            smach.StateMachine.add('OBJECT_PERCEPTION',
                                   object_percept_state,
                                   transitions={
                                       'object_detect_done': 'SAVE_POI_STATE'},
                                   remapping={'no_of_object': 'table_object_state', 'phase_no': 'phase_value'})

            smach.StateMachine.add('HRI', hri_state,
                                   transitions={'greeted': 'OBJECT_PERCEPTION', 'announced': 'finished'}, remapping={'phase_no': 'phase_value'})

            smach.StateMachine.add('SAVE_POI_STATE',
                                   poi_state_state,
                                   transitions={'saved': 'NAVIGATION'},
                                   remapping={'phase_no': 'phase_value'})

        smach.StateMachine.add('PHASE_1', Phase1,
                               transitions={'finished': 'PHASE_2'})

        Phase2 = smach.StateMachine(
            outcomes=['finished', 'failed'], output_keys=['phase_value'])
        Phase2.userdata.phase_value = 2

        # Open the container
        with Phase2:
            smach.StateMachine.add('NAVIGATION', Navigate(
                poi), {'take_order_done': 'finished', 'at_POI': 'TAKE_ORDER'},
                remapping={'phase_no': 'phase_value'})

            smach.StateMachine.add('TAKE_ORDER', hri_state,
                                   transitions={'order_taken': 'NAVIGATION'})

            smach.StateMachine.add('UPDATE_POI_STATE', poi_state_state,
                                   transitions={'updated': 'NAVIGATION'})

        Phase3 = smach.StateMachine(
            outcomes=['finished', 'failed'], output_keys=['phase_value', 'task'])
        Phase3.userdata.phase_value = 3
        Phase3.userdata.task = 'report order'

        # Open the container
        with Phase3:
            smach.StateMachine.add('NAVIGATION', Navigate(
                poi), {'at_counter': 'HRI', 'at_POI': 'HRI', 'at_default_location': 'finished'},
                remapping={'phase_no': 'phase_value', 'current_task': 'task'})
            smach.StateMachine.add('HRI', hri_state,
                                   transitions={'order_reported': 'OBJECT_PERCEPTION', 'object_taken': 'NAVIGATION', 'object_delivered': 'NAVIGATION'})
            smach.StateMachine.add('OBJECT_PERCEPTION',
                                   object_percept_state,
                                   transitions={
                                       'missing_object': 'HRI', 'object_ok': 'HRI'},
                                   remapping={'no_of_object': 'table_object_state', 'phase_no': 'phase_value'})

    # Execute SMACH plan
    outcome = Trial.execute()
