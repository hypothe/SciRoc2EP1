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
            outcomes=['shop_explore_done', 'at_POI'], output_keys=['current_poi', 'current_task'], input_keys=['phase_no', 'current_task'])
        self.poi = poi[1:]
        self.counter = poi[0]

    def _call_POI_service(self, poi):
        """To initialize and call the go_to_poi service
        """
        rospy.wait_for_service('go_to_poi_service')
        try:
            go_to_poi = rospy.ServiceProxy('go_to_poi_service', GoToPOI)
            result = go_to_poi(poi)

            if (result.result == 'goal reached'):
                print(result.result)
                return True
            else:
                print(
                    'Point of interest [{poi}] does not exist'.format(poi=poi))
                return False
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def _call_POI_state(self):
        rospy.wait_for_service('get_poi_state')
        try:
            poi_state = rospy.ServiceProxy('get_poi_state',  GetPOIState)
            state = poi_state('needs serving')
            return state
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def get_need_order(self):
        state = self._call_POI_state()
        if (state.need_order_no > 0):
            table_id = state.table_id
            return table_id
        else:
            return 'All order taken'

    def get_table_to_serve(self):
        state = self._call_POI_state()
        if (state.need_serving_no > 0):
            table_id = state.table_id
            return table_id
        else:
            return 'All Served'

    def execute(self, userdata):
        # Check what phase the robot is at
        if (userdata.phase_no == 1):
            # this is the code that is executed when in this state
            rospy.loginfo('Navigating to the next Point of Interest')
            if len(self.poi) == 0:
                next_poi = self.counter
                result = self._call_POI_service(next_poi)
                if result:
                    userdata.current_poi = next_poi
                    return 'shop_explore_done'
            else:
                next_poi = self.poi.pop()
                result = self._call_POI_service(next_poi)
                if result:
                    userdata.current_poi = next_poi
                    return 'at_POI'

        elif (userdata.phase_no == 2):
            table = self.get_need_order()
            if (table == 'All order taken'):
                return 'take_order_done'
            else:
                result = self._call_POI_service(table)
                if result:
                    userdata.current_poi = table
                    return 'at_POI'

        elif (userdata.phase_no == 3):
            if (userdata.task == 'report order'):
                result = self._call_POI_service(self.counter)
                if result:
                    userdata.current_poi = self.counter
                    return 'at_counter'
            if (userdata.task == 'deliver order'):
                table = self.get_table_to_serve()
                if (table == 'All Served'):
                    return 'all served'
                else:
                    result = self._call_POI_service(table)
                    if result:
                        userdata.current_poi = table
                        return 'at_POI'
            if (userdata.task == 'to default location'):
                result = self._call_POI_service(self.counter)
                if result:
                    userdata.current_poi = self.counter
                    return 'at_default_location'


class POI_State(smach.State):
    def __init__(self, poi):
        # from here we define the possible outcomes of the state.
        super().__init__(
            outcomes=['saved', 'updated'], output_keys=['current_poi'], input_keys=['table_object_state', 'table_people_state', 'current_poi', 'phase_no'])

    def _call_POI_State_service(self, task, set_state_request=SetPOIStateRequest(), update_state_request=UpdatePOIStateRequest()):
        """To initialize and call the go_to_poi service
        """
        if (task == 'set_state'):
            rospy.wait_for_service('set_poi_state')
            try:
                set_state = rospy.ServiceProxy('set_poi_state', SetPOIState)
                result = set_state(set_state_request)

                if (result.result == 'saved'):
                    return True
            except rospy.ServiceException as e:
                print('Service call failed: {e}'.format(e=e))
        elif(task == 'update_state'):
            rospy.wait_for_service('update_poi_state')
            try:
                update_state = rospy.ServiceProxy(
                    'update_poi_state', UpdatePOIState)
                result = update_state(update_state_request)

                if (result.result == 'updated'):
                    return True
            except rospy.ServiceException as e:
                print('Service call failed: {e}'.format(e=e))

    def execute(self, userdata):
        if (userdata.phase_no == 1):
            set_state_request = SetPOIStateRequest()
            set_state_request.table_id = userdata.current_poi
            set_state_request.no_of_people = userdata.table_people_state
            set_state_request.no_of_object = userdata.table_object_state
            if(userdata.table_people_state > 0 and userdata.table_object_state == 0):
                set_state_request.table_state = 'needs serving'
            if(userdata.table_people_state > 0 and userdata.table_object_state > 0):
                set_state_request.table_state = 'already served'
            if(userdata.table_people_state == 0 and userdata.table_object_state > 0):
                set_state_request.table_state = 'need cleaning'
            if(userdata.table_people_state == 0 and userdata.table_object_state == 0):
                # the table is ready to accept new customers
                set_state_request.table_state = 'ready'
            result = self._call_POI_State_service(
                'set_state', set_state_request=set_state_request)
            if (result):
                return 'saved'

        elif(userdata.phase_no == 2):
            update_state_request = UpdatePOIStateRequest()
            update_state_request.table_id = userdata.current_poi
            update_state_request.updated_states = ['needs serving']
            update_state_request.needs_serving = False

            result = self._call_POI_State_service(
                'update_state', update_state_request=update_state_request)
            if (result):
                return 'updated'

        elif(userdata.phase_no == 3):
            pass


# HUMAN ROBOT INTERACTION (HRI)

# Callback function for setting up the goal


def hri_goal_cb(userdata, goal):
    hri_goal = HRIAction()
    if (userdata.phase_no == 1):
        if userdata.current_poi == 'counter':
            hri_goal.task = 'announce'
        else:
            hri_goal.task = 'greet'
        return hri_goal
    elif(userdata.phase_no == 2):
        hri_goal.task = 'take_order'
        return hri_goal
    elif(userdata.phase_no == 3):
        if (userdata.task == 'report order'):
            hri_goal.task = 'report order'
        if (userdata.task == 'take item'):
            hri_goal.task = 'take item'
        if (userdata.task == 'deliver drink'):
            hri_goal.task = 'deliver drink'
        return hri_goal


def hri_result_cb(userdata, status, result):
    if (userdata.phase_no == 1):
        if status == GoalStatus.SUCCEEDED:
            if userdata.current_poi == 'counter':
                return 'announced'
            else:
                return 'greeted'
    elif (userdata.phase_no == 2):
        if status == GoalStatus.SUCCEEDED:
            return 'order_taken'
    elif (userdata.phase_no == 3):
        if status == GoalStatus.SUCCEEDED:
            if (result == 'order_reported'):
                userdata.current_task == 'check object'
                return 'order_reported'
            if (result == 'item taken'):
                userdata.current_task == 'deliver order'
                return 'object_taken'
            if (result == 'item delivered'):
                userdata.current_task == 'to default location'
                return 'object_delivered'


hri_state = SimpleActionState('hri', HRIAction,
                              goal_cb=hri_goal_cb,
                              result_cb=hri_result_cb,
                              input_keys=['current_poi',
                                          'phase_no', 'current_task'],
                              output_keys=['current_task'])

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
