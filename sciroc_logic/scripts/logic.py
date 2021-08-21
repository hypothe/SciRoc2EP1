#!/usr/bin/env python

import rospy
import smach
from smach_ros import ServiceState, SimpleActionState
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg._GoalStatus import GoalStatus
# Brings in the messages used by the go_to_poi service
from sciroc_navigation.srv import GoToPOI
from sciroc_poi_state.srv import SetPOIState, UpdatePOIState, GetTableByState
from sciroc_poi_state.srv import SetPOIStateRequest, UpdatePOIStateRequest


# Creating the States

# NAVIGATION


class Navigate(smach.State):
    def __init__(self, poi):
        # from here we define the possible outcomes of the state.
        super().__init__(
            outcomes=['shop_explore_done', 'at_POI'], output_keys=['current_poi', ], input_keys=['phase_no', 'current_task'])
        self.poi = poi[1:]
        self.counter = poi[0]

    def call_nav_service(self, next_poi):
        """To initialize and call the go_to_poi service
        """
        rospy.wait_for_service('go_to_poi_service')
        try:
            go_to_poi = rospy.ServiceProxy('go_to_poi_service', GoToPOI)
            result = go_to_poi(next_poi)

            if (result.result == 'goal reached'):
                return True
            else:
                print(
                    'Point of interest [{poi}] does not exist'.format(poi=poi))
                return False
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def get_table_by_state(self, state):
        rospy.wait_for_service('get_table_by_state')
        try:
            poi_state = rospy.ServiceProxy(
                'get_table_by_state',  GetTableByState)
            table = poi_state(state)
            return table
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def execute(self, userdata):
        # Check what phase the robot is at
        if (userdata.phase_no == 1):
            # this is the code that is executed when in this state
            if len(self.poi) == 0:
                next_poi = self.counter
                result = self.call_nav_service(next_poi)
                if result:
                    userdata.current_poi = next_poi
                    return 'shop_explore_done'
            else:
                next_poi = self.poi.pop()
                result = self.call_nav_service(next_poi)
                if result:
                    userdata.current_poi = next_poi
                    return 'at_POI'

        elif (userdata.phase_no == 2):
            table = self.get_table_by_state('require order')
            if (table.require_order_no > 0):
                result = self.call_nav_service(table.table_id)
                if result:
                    userdata.current_poi = table.table_id
                    return 'at_POI'
            else:
                return 'take_order_done'

        elif (userdata.phase_no == 3):
            if (userdata.task == 'report order'):
                result = self.call_nav_service(self.counter)
                if result:
                    userdata.current_poi = self.counter
                    return 'at_counter'
            if (userdata.task == 'deliver order'):
                table = self.get_table_by_state('current serving')
                if (table.current_serving_no > 0):
                    result = self.call_nav_service(table.table_id)
                    if result:
                        userdata.current_poi = table.table_id
                        return 'at_POI'
                else:
                    return 'served'

            if (userdata.task == 'to default location'):
                result = self.call_nav_service(self.counter)
                if result:
                    userdata.current_poi = self.counter
                    return 'at_default_location'

# POINT OF INTEREST STATE


class POI_State(smach.State):
    def __init__(self):
        # from here we define the possible outcomes of the state.
        super().__init__(
            outcomes=['saved', 'updated'], output_keys=['current_poi'], input_keys=['no_of_object', 'no_of_people', 'current_poi', 'phase_no'])

    def call_poi_state_service(self, task, set_state_request=SetPOIStateRequest(), update_state_request=UpdatePOIStateRequest()):
        """To initialize and call the poi state service
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
            set_state_request.no_of_people = userdata.no_of_people
            set_state_request.no_of_object = userdata.no_of_object

            result = self.call_poi_state_service(
                'set_state', set_state_request=set_state_request)
            if (result):
                return 'saved'

        elif(userdata.phase_no == 2):
            update_state_request = UpdatePOIStateRequest()
            update_state_request.table_id = userdata.current_poi
            update_state_request.updated_states = [
                'require order', 'required drinks']
            update_state_request.require_order = False
            update_state_request.required_drinks = userdata.order_list

            result = self.call_poi_state_service(
                'update_state', update_state_request=update_state_request)
            if (result):
                return 'updated'

        elif(userdata.phase_no == 3):
            pass


# HUMAN ROBOT INTERACTION (HRI)

class HRI(smach.State):
    def __init__(self):
        # from here we define the possible outcomes of the state.
        super().__init__(
            outcomes=[], output_keys=['current_task'], input_keys=['current_poi', 'phase_no', 'current_task'])

    def call_hri_action(self, goal_req):
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient('hri', HRIAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(goal_req)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # return the result of executing the action
        return client.get_result()

    def get_table_by_state(self, state):
        rospy.wait_for_service('get_table_by_state')
        try:
            poi_state = rospy.ServiceProxy(
                'get_table_by_state',  GetTableByState)
            table = poi_state(state)
            return table
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def execute(self, userdata):
        hri_goal = HRIAction()
        if (userdata.phase_no == 1):
            if userdata.current_poi == 'counter':
                hri_goal.task = 'announce'
                result = self.call_hri_action(hri_goal)
                if result.status == 'announced':
                    return 'announced'
            else:
                hri_goal.task = 'greet'
                result = self.call_hri_action(hri_goal)
                if result.status == 'greeted':
                    return 'greeted'

        elif (userdata.phase_no == 2):
            hri_goal.task = 'take_order'
            result = self.call_hri_action(hri_goal)
            if result.status == 'order taken':
                userdata.order_list = result.required_drinks
                return 'order_taken'

        elif (userdata.phase_no == 3):
            if (userdata.task == 'report order'):
                hri_goal.task = 'report order'
                result = self.call_hri_action(hri_goal)
                if (result.status == 'order_reported'):
                    userdata.current_task = 'check object'
                    return 'order_reported'
            if (userdata.task == 'take item'):
                hri_goal.task = 'take item'
                result = self.call_hri_action(hri_goal)
                if (result.status == 'item taken'):
                    userdata.current_task = 'deliver order'
                    return 'object_taken'
            if (userdata.task == 'report missing'):
                hri_goal.task = 'report missing'
                hri_goal.missing_drinks.extend(userdata.missing_drinks)
                result = self.call_hri_action(hri_goal)
                if (result.result == 'reported'):
                    userdata.task = 'check object'
                    return 'reported missing'

            if (userdata.task == 'report wrong'):
                hri_goal.task = 'report wrong'
                hri_goal.wrong_drinks.extend(userdata.wrong_drinks)
                result = self.call_hri_action(hri_goal)
                if (result.result == 'reported'):
                    userdata.task = 'check object'
                    return 'reported wrong'


# PEOPLE PERCEPTION


class PeoplePerception(smach.State):
    def __init__(self):
        # from here we define the possible outcomes of the state.
        super().__init__(
            outcomes=[], output_keys=['no_of_people'], input_keys=['phase_no'])

    def call_people_percept(self, goal_req):
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient(
            'people_percept', PeoplePerceptionAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(goal_req)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # return the result of executing the action
        return client.get_result()

    def get_table_by_state(self, state):
        rospy.wait_for_service('get_table_by_state')
        try:
            poi_state = rospy.ServiceProxy(
                'get_table_by_state',  GetTableByState)
            table = poi_state(state)
            return table
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def execute(self, userdata):
        people_percept_goal = PeoplePerceptionAction()
        if (userdata.phase_no == 1):
            people_percept_goal.mode = 'detect'
            result = self.call_people_percept(people_percept_goal)
            if result == 'detected':
                userdata.no_of_people = result.no_of_people
                if(result.no_of_people > 0):
                    return 'people_present'
                else:
                    return 'people_not_present'

        elif(userdata.phase_no == 2):
            pass
        elif(userdata.phase_no == 3):
            pass


# OBJECT PERCEPTION


class ObjectDetection(smach.State):
    def __init__(self):
        # from here we define the possible outcomes of the state.
        super().__init__(
            outcomes=[], output_keys=['no_of_object'], input_keys=['phase_no'])

    def call_object_detect(self, goal_req):
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient(
            'object_detect', ObjectPerceptionAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(goal_req)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # return the result of executing the action
        return client.get_result()

    def get_table_by_state(self, state):
        rospy.wait_for_service('get_table_by_state')
        try:
            poi_state = rospy.ServiceProxy(
                'get_table_by_state',  GetTableByState)
            table = poi_state(state)
            return table
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def execute(self, userdata):
        object_detect_goal = ObjectDetectionAction()
        if (userdata.phase_no == 1):
            object_detect_goal.mode = 'detect'
            result = self.call_object_detect(object_detect_goal)
            if result == 'detected':
                userdata.no_of_object = result.no_of_object
                return 'object_detect_done'

        elif(userdata.phase_no == 2):
            pass

        elif(userdata.phase_no == 3):
            if (userdata.task == 'check order'):
                table = self.get_table_by_state('current serving')
                object_detect_goal.mode = 'check'
                object_detect_goal.required_drinks = table.required_drinks
                result = self.call_object_detect(object_detect_goal)
                if (result.result == 'correct'):
                    userdata.task = 'take item'
                    return 'take item'
                if (result.result == 'wrong'):
                    userdata.task = 'report wrong'
                    return 'wrong order'
                if (result.result == 'missing'):
                    userdata.task = 'report missing'
                    return 'missing order'


# Setting up the request callback for the Save POI State service

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
