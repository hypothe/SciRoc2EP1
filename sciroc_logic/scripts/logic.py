#!/usr/bin/env python

from pickle import EMPTY_DICT
import rospy
import smach
from smach_ros import ServiceState, SimpleActionState
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from smach import CBState
from actionlib_msgs.msg._GoalStatus import GoalStatus
# Brings in the messages used by the go_to_poi service
from sciroc_navigation.srv import GoToPOI


# Creating the States

# NAVIGATION


class Navigate(smach.State):
    def __init__(self, poi):
        # from here we define the possible outcomes of the state.
        super().__init__(
            outcomes=['shop_explore_done', 'at_POI'], output_keys=['current_poi'], input_keys=['phase_no'])
        self.poi = poi[1:]
        self.counter = poi[0]

    def _call_POI_service(self):
        """To initialize and call the go_to_poi service
        """
        rospy.wait_for_service('go_to_poi_service')
        try:
            go_to_poi = rospy.ServiceProxy('go_to_poi_service', GoToPOI)
            result = go_to_poi(self.poi)

            if (result.result == 'goal reached'):
                print(result.result)
                return True
            else:
                print(
                    'Point of interest [{poi}] does not exist'.format(poi=poi))
                return False
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def get_table_to_serve(self):
        pass

    def execute(self, userdata):
        # Check what phase the robot is at
        if (userdata.phase_no == 1):
            # this is the code that is executed when in this state
            rospy.loginfo('Navigating to the next Point of Interest')
            if len(self.poi) == 0:
                next_poi = self.counter
                result = self.call_POI_service(next_poi)
                if result:
                    userdata.current_poi = next_poi
                    return 'shop_explore_done'
            else:
                next_poi = self.poi.pop()
                result = self.call_POI_service(next_poi)
                if result:
                    userdata.current_poi = next_poi
                    return 'at_POI'
        elif (userdata.phase_no == 2):
            pass

        elif (userdata.phase_no == 3):
            pass

# HUMAN ROBOT INTERACTION (HRI)

# Callback function for setting up the goal


def hri_goal_cb(userdata, goal):
    if (userdata.phase_no == 1):
        hri_goal = HRIAction()
        if userdata.current_poi == 'counter':
            hri_goal.task = 'announce'
        else:
            hri_goal.task = 'greet'
        return hri_goal
    elif(userdata.phase_no == 2):
        pass
    elif(userdata.phase_no == 3):
        pass


def hri_result_cb(userdata, status, result):
    if (userdata.phase_no == 1):
        if status == GoalStatus.SUCCEEDED:
            if userdata.current_poi == 'counter':
                return 'announced'
            else:
                return 'greeted'
    elif (userdata.phase_no == 2):
        pass
    elif (userdata.phase_no == 3):
        pass


hri_state = SimpleActionState('hri', HRIAction,
                              goal_cb=hri_goal_cb,
                              result_cb=hri_result_cb,
                              input_keys=['current_poi', 'phase_no'])

# PEOPLE PERCEPTION

# Setting up the goal message for People Perception
people_percept_goal = PeoplePerceptionAction()
people_percept_goal.mode = 'detect'

# the result callback for the people perception action


def people_percept_result_cb(userdata, status, result):
    if (userdata.phase_no == 1):
        if status == GoalStatus.SUCCEEDED:
            userdata.no_of_people = result.no_of_people
            if(result.no_of_people > 0):
                return 'people_present'
            else:
                return 'people_not_present'
    elif(userdata.phase_no == 2):
        pass
    elif(userdata.phase_no == 3):
        pass


people_percept_state = SimpleActionState('people_percept', PeoplePerceptionAction,
                                         goal=people_percept_goal,
                                         result_cb=people_percept_result_cb,
                                         output_keys=['no_of_people'],
                                         input_keys=['phase_no'])

# OBJECT PERCEPTION

# Setting up the goal message for Object Perception
object_percept_goal = ObjectPerceptionAction()
object_percept_goal.mode = 'detect'


def object_percept_result_cb(userdata, status, result):
    if (userdata.phase_no == 1):
        if status == GoalStatus.SUCCEEDED:
            userdata.no_of_object = result.no_of_object
            return 'object_detect_done'
    elif (userdata.phase_no == 2):
        pass 
    elif (userdata.phase_no == 3):
        pass 

object_percept_state = SimpleActionState('object_percept', ObjectPerceptionAction,
                                         goal=object_percept_goal,
                                         result_cb=object_percept_result_cb,
                                         output_keys=['no_of_object'],
                                         input_keys=['phase_no'])


# Setting up the request callback for the Save POI State service


@smach.cb_interface(input_keys=['table_object_state', 'table_people_state', 'current_poi', 'phase_no'])
def poi_state_request_cb(userdata, request):
    if (userdata.phase_no == 1):
        poi_state_request = POIState()
        poi_state_request.table_id = userdata.current_poi
        poi_state_request.no_of_people = userdata.table_people_state
        poi_state_request.no_of_object = userdata.table_object_state
        if(userdata.table_people_state > 0 and userdata.table_object_state == 0):
            poi_state_request.table_state = 'needs serving'
        if(userdata.table_people_state > 0 and userdata.table_object_state > 0):
            poi_state_request.table_state = 'already served'
        if(userdata.table_people_state == 0 and userdata.table_object_state > 0):
            poi_state_request.table_state = 'need cleaning'
        if(userdata.table_people_state == 0 and userdata.table_object_state == 0):
            # the table is ready to accept new customers
            poi_state_request.table_state = 'ready'
        return poi_state_request
    elif(userdata.phase_no == 2):
        pass 
    elif(userdata.phase_no == 3): 
        pass 

def poi_state_response_cb(userdata, response):
    if (response):
        return 'saved'


poi_state_state = ServiceState('poi_state',
                               POIState,
                               request_cb=poi_state_request_cb,
                               response_cb=poi_state_response_cb)


if __name__ == '__main__':
    rospy.init_node('sciroc_state_machine')

    poi = ['counter', 'table1', 'table2',
           'table3', 'table4', 'table5', 'table6']

    # Create a SMACH state machine
    Trial = smach.StateMachine(outcomes=['finished', 'failed'])

    with Trial:

        Phase1 = smach.StateMachine(
            outcomes=['finished', 'failed'], output_keys=['phase_value'])
        Phase1.userdata.phase_value = 1

        # Open the container
        with Phase1:

            # Add states to the container
            smach.StateMachine.add('NAVIGATION', Navigate(
                poi), {'shop_explore_done': 'HRI', 'at_POI': 'PEOPLE_PERCEPTION'},
                remapping={'phase_no': 'phase_value'})

            smach.StateMachine.add('PEOPLE_PERCEPTION',
                                   people_percept_state,
                                   transitions={
                                       'people_present': 'HRI', 'people_not_precent': 'OBJECT_PERCEPTION'},
                                   remapping={'no_of_people': 'table_people_state', 'phase_no': 'phase_value'})

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

            # take_order = smach.StateMachine(outcomes=['finished', 'failed'])

            # with take_order:
            #     smach.StateMachine.add('GREET', hri_state,
            #                            transitions={'greeted':'ASK_FOR_ORDER'}, remapping={'phase_no':'phase_value'})
            #     smach.StateMachine.add('ASK_FOR_ORDER', hri_state,
            #                            transitions={'success':'TAKE_ORDER'})
            #     smach.StateMachine.add('RECORD_ORDER', hri_state,
            #                            transitions={'success':'finished'})

            smach.StateMachine.add('TAKE_ORDER', hri_state,
                                   transitions={'finished': 'NAVIGATION'})

        Phase3 = smach.StateMachine(
            outcomes=['finished', 'failed'], output_keys=['phase_value'])
        Phase3.userdata.phase_value = 3

        # Open the container
        with Phase3:
            smach.StateMachine.add()
            smach.StateMachine.add()
            smach.StateMachine.add()

    # Execute SMACH plan
    outcome = Trial.execute()
