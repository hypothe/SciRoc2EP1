import rospy

# importing the labrary for the creation of the state machine
import smach

# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg._GoalStatus import GoalStatus

# navigation service message
from sciroc_navigation.srv import GoToPOI

# message for updating and the getting the state of the POI (Point of Interest)
from sciroc_poi_state.srv import UpdatePOIState, GetTableByState
from sciroc_poi_state.srv import UpdatePOIStateRequest


##########-------------- CREATING THE STATE ----------------------##########################


###+++++++++++++++++++ NAVIGATION +++++++++++++++++++++###


class Navigate(smach.State):
    """This is the class for the object that performs the required actions
    when the robot is in the navigation state, it helps the robot navigate to
    a point of interest based on the phase the robot operation is at

    Args:
        smach ([class]): This is the superclass the Navigation class inherits from,
        this gives it all the required attributes and methods to instantiate the state class

    Attributes
    ---
    poi: list
        this is a list that contains all the point of interest saved in the poi parameter server
    counter: str 
        this is the id of the counter pose 

    Methods
    ---
    call_nav_service(next_poi)
        for sending a request to the go to poi service to navigate the robot to the next_poi arg
    get_table_by_state(state)
        for getting the table object of the the table that has thesame state as the state arg
    execute(userdata)
        this is the function that would be called when the state is called
    """

    def __init__(self, poi):
        """
        Args:
            poi ([list]): list containing all the point of interest from the poi parameter server

        """
        super().__init__(
            outcomes=['at_POI', 'shop_explore_done', 'at_counter', 'at_require_order_table',
                      'at_current_serving_table', 'at_default_location', 'trail_finished'],
            output_keys=['current_poi', 'task'], input_keys=['phase_no', 'task'])
        # This would be changed later, only here for testing reasons
        self.poi = poi[1:]
        self.counter = poi[0]

    def call_nav_service(self, next_poi):
        """This method sends a request of the next_poi to the go to poi service to navigate the 
        robot to

        Args:
            next_poi (str): the id of the point of interest to navigate to

        Returns:
            [bool]: True if successful and False otherwise
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
        """This method gets the object of the point of interest that has thesame
        state as the requested state

        Args:
            state (str): The requested state

        Returns:
            [object]: an object of a point of interest with thesame state as the 
            requested state
        """
        rospy.wait_for_service('get_table_by_state')
        try:
            poi_state = rospy.ServiceProxy(
                'get_table_by_state',  GetTableByState)
            table = poi_state(state)
            return table
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def execute(self, userdata):
        """This is the function that is called when the state machine is at this state. 

        Args:
            userdata (struct): This is a struct containing input and/or output data from/to the state

        Returns:
            [string]: This is the outcome of this state
        """
        # Check what phase the robot is at
        if (userdata.phase_no == 1):
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
                    return 'at_require_order_table'
            else:
                result = self.call_nav_service(self.counter)
                return 'trial_finished'

        elif (userdata.phase_no == 3):
            if (userdata.task == 'report order'):
                result = self.call_nav_service(self.counter)
                if result:
                    userdata.current_poi = self.counter
                    return 'at_counter'
            if (userdata.task == 'deliver order'):
                table = self.get_table_by_state('current serving')
                result = self.call_nav_service(table.table_id)
                if result:
                    userdata.current_poi = table.table_id
                    userdata.task = 'announce order arrival'
                    return 'at_current_serving_table'

            # if (userdata.task == 'to default location'):
            #     result = self.call_nav_service(self.counter)
            #     if result:
            #         userdata.current_poi = self.counter
            #         return 'at_default_location'


###+++++++++++++++++++ POINT OF INTEREST STATE (POI STATE) +++++++++++++++++++++###

class POI_State(smach.State):
    """This is the class for the object that performs the required actions
    for saving and updating the state of the point of interest 

    Args:
        smach ([class]): This is the superclass the POI_State class inherits from,
        this gives it all the required attributes and methods to instantiate the state class

    Methods
    ---
    call_poi_state_service(update_state_request=UpdatePOIStateRequest())
        for sending the request to update the state of a point of interest
    execute(userdata)
        this is the function that would be called when the state is called
    """

    def __init__(self):
        super().__init__(
            outcomes=['saved', 'updated'],
            input_keys=['phase_no', 'current_poi', 'no_of_people', 'no_of_object', 'order_list'])

    def call_poi_state_service(self, update_state_request=UpdatePOIStateRequest()):
        """This is a method for calling the service for saving and update the state of 
        a point of interest, it send the updated state or the initial state as a request 
        to the update_poi_state service. 

        Args:
            update_state_request ([object], optional): This is the request message thats sent 
                to the update_poi_state_service. Defaults to UpdatePOIStateRequest().

        Returns:
            [bool]: True if successful False otherwise
        """
        rospy.wait_for_service('update_poi_state')
        try:
            update_state = rospy.ServiceProxy(
                'update_poi_state', UpdatePOIState)
            result = update_state(update_state_request)

            if (result.result == 'updated' or result.result == 'saved'):
                return True
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def execute(self, userdata):
        """This is the function that is called when the state machine is at this state

        Args:
            userdata (struct): This is a struct containing input and/or output data from/to the state

        Returns:
            [string]: This is the outcome of this state
        """
        # PHASE 1
        if (userdata.phase_no == 1):
            set_state_request = UpdatePOIStateRequest()
            set_state_request.task = 'set'
            set_state_request.table_id = userdata.current_poi
            set_state_request.no_of_people = userdata.no_of_people
            set_state_request.no_of_object = userdata.no_of_object

            result = self.call_poi_state_service(
                update_state_request=set_state_request)
            if (result):
                return 'saved'
        # PHASE 2
        elif(userdata.phase_no == 2):
            update_state_request = UpdatePOIStateRequest()
            update_state_request.task = 'update'
            update_state_request.table_id = userdata.current_poi
            update_state_request.updated_states = [
                'require order', 'required drinks', 'current serving']
            update_state_request.require_order = False
            update_state_request.current_serving = True
            update_state_request.required_drinks = userdata.order_list

            result = self.call_poi_state_service(
                update_state_request=update_state_request)
            if (result):
                return 'updated'
        # PHASE 3
        elif(userdata.phase_no == 3):
            update_state_request = UpdatePOIStateRequest()
            update_state_request.table_id = userdata.current_poi
            update_state_request.updated_states = [
                'need serving', 'already served', 'current serving']
            update_state_request.need_serving = False
            update_state_request.already_served = True
            update_state_request.current_serving = False
            result = self.call_poi_state_service(
                update_state_request=update_state_request)
            if (result):
                return 'updated'


###+++++++++++++++++++ HUMAN ROBOT INTERACTION (HRI) +++++++++++++++++++++###

class HRI(smach.State):
    """This is the class for the object that performs the required actions
    when the robot is in the human robot interaction state, it helps the robot interact
    with the customers and the barista

    Args:
        smach ([class]): This is the superclass the HRI class inherits from,
        this gives it all the required attributes and methods to instantiate the state class

    Methods
    ---
    call_hri_action(goal_req)
        for send a goal request to the hri action server 
    get_table_by_state(state)
        for getting the table object of the the table that has thesame state as the state arg
    execute(userdata)
        this is the function that would be called when the state is called
    """

    def __init__(self):
        super().__init__(
            outcomes=['announced', 'greeted', 'order_taken', 'order_reported',
                      'missing_reported', 'wrong_reported', 'object_taken', 'order_delivered'],
            output_keys=['task'],
            input_keys=['phase_no', 'current_poi', 'task', 'missing_drinks', 'wrong_drinks'])

    def call_hri_action(self, goal_req):
        """This method sends a goal request to the hri action server 

        Args:
            goal_req (str): the goal request message 

        Returns:
            [str]: the result returned from the hri action server
        """
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
        """This method gets the object of the point of interest that has thesame
        state as the requested state

        Args:
            state (str): The requested state

        Returns:
            [object]: an object of a point of interest with thesame state as the 
            requested state
        """
        rospy.wait_for_service('get_table_by_state')
        try:
            poi_state = rospy.ServiceProxy(
                'get_table_by_state',  GetTableByState)
            table = poi_state(state)
            return table
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def execute(self, userdata):
        """This is the function that is called when the state machine is at this state

        Args:
            userdata (struct): This is a struct containing input and/or output data from/to the state

        Returns:
            [string]: This is the outcome of this state
        """

        hri_goal = HRIAction()
        if (userdata.phase_no == 1):
            if userdata.current_poi == 'counter':
                hri_goal.task = 'announce state'
                result = self.call_hri_action(hri_goal)
                if result.result == 'announced':
                    return 'announced'
            else:
                hri_goal.task = 'greet'
                result = self.call_hri_action(hri_goal)
                if result.result == 'greeted':
                    return 'greeted'

        elif (userdata.phase_no == 2):
            hri_goal.task = 'take order'
            result = self.call_hri_action(hri_goal)
            if result.result == 'order taken':
                userdata.order_list = result.required_drinks
                return 'order_taken'

        elif (userdata.phase_no == 3):
            if (userdata.task == 'report order'):
                hri_goal.task = 'report order'
                result = self.call_hri_action(hri_goal)
                if (result.result == 'order reported'):
                    userdata.task = 'check object'
                    return 'order_reported'

            elif (userdata.task == 'report missing'):
                hri_goal.task = 'report missing'
                hri_goal.missing_drinks.extend(userdata.missing_drinks)
                result = self.call_hri_action(hri_goal)
                if (result.result == 'missing reported'):
                    userdata.task = 'check object'
                    return 'missing_reported'

            elif (userdata.task == 'report wrong'):
                hri_goal.task = 'report wrong'
                hri_goal.wrong_drinks.extend(userdata.wrong_drinks)
                result = self.call_hri_action(hri_goal)
                if (result.result == 'wrong reported'):
                    userdata.task = 'check object'
                    return 'wrong_reported'

            elif (userdata.task == 'take item'):
                hri_goal.task = 'take item'
                result = self.call_hri_action(hri_goal)
                if (result.result == 'item taken'):
                    userdata.task = 'deliver order'
                    return 'object_taken'
            elif (userdata.task == 'announce order arrival'):
                hri_goal.task = 'announce order arrival'
                result = self.call_hri_action(hri_goal)
                if (result.result == 'order arrival announced'):
                    return 'order_delivered'


###+++++++++++++++++++ PEOPLE PERCEPTION +++++++++++++++++++++###

class PeoplePerception(smach.State):
    """This is the class for the object that performs the required actions
    when the robot is in the people perception state, it helps the robot 
    detect the presence and absence of people at a point of interest (poi)

    Args:
        smach ([class]): This is the superclass the PeoplePerception class inherits from,
        this gives it all the required attributes and methods to instantiate the state class

    Methods
    ---
    call_people_percept(goal_req)
        for sending a goal request to the people perception action server 
    get_table_by_state(state)
        for getting the table object of the the table that has thesame state as the state arg
    execute(userdata)
        this is the function that would be called when the state is called
    """

    def __init__(self):
        # from here we define the possible outcomes of the state.
        super().__init__(
            outcomes=['people_present', 'people_not_present'], output_keys=['no_of_people'], input_keys=['phase_no'])

    def call_people_percept(self, goal_req):
        """This method sends a goal request to the people_percept action server 

        Args:
            goal_req (str): the goal request message

        Returns:
            [object]: the result returned from the people percept action server 
        """
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
        """This method gets the object of the point of interest that has thesame
        state as the requested state

        Args:
            state (str): The requested state

        Returns:
            [object]: an object of a point of interest with thesame state as the 
            requested state
        """
        rospy.wait_for_service('get_table_by_state')
        try:
            poi_state = rospy.ServiceProxy(
                'get_table_by_state',  GetTableByState)
            table = poi_state(state)
            return table
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def execute(self, userdata):
        """This is the function that is called when the state machine is at this state

        Args:
            userdata (struct): This is a struct containing input and/or output data from/to the state

        Returns:
            [string]: This is the outcome of this state
        """

        people_percept_goal = PeoplePerceptionAction()
        if (userdata.phase_no == 1):
            people_percept_goal.mode = 'detect'
            result = self.call_people_percept(people_percept_goal)
            if result.result == 'detected':
                userdata.no_of_people = result.no_of_people
                if(result.no_of_people > 0):
                    return 'people_present'
                else:
                    return 'people_not_present'
        # People perception is not required in both phase 1 & 2
        elif(userdata.phase_no == 2):
            pass
        elif(userdata.phase_no == 3):
            pass


###+++++++++++++++++++ OBJECT DETECTION  +++++++++++++++++++++###


class ObjectDetection(smach.State):
    """This is the class for the object that performs the required actions
    when the robot is in the navigation state, it helps the robot navigate to
    a point of interest based on the phase the robot operation is at

    Args:
        smach ([class]): This is the superclass the Navigation class inherits from,
        this gives it all the required attributes and methods to instantiate the state class

    Attributes
    ---
    poi: list
        this is a list that contains all the point of interest saved in the poi parameter server
    counter: str 
        this is the id of the counter pose 

    Methods
    ---
    call_nav_service(next_poi)
        for sending a request to the go to poi service to navigate the robot to the next_poi arg
    get_table_by_state(state)
        for getting the table object of the the table that has thesame state as the state arg
    execute(userdata)
        this is the function that would be called when the state is called
    """

    def __init__(self):
        # from here we define the possible outcomes of the state.
        super().__init__(
            outcomes=['object_detect_done', 'correct_order',
                      'wrong_order', 'missing_order'],
            output_keys=['no_of_object', 'task',
                         'wrong_drinks', 'missing_drinks'],
            input_keys=['phase_no', 'task'])

    def call_object_detect(self, goal_req):
        """This method sends a goal request to the object detection action server 

        Args:
            goal_req (str): the goal request message

        Returns:
            [object]: the result returned from the object detection action server 
        """
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
        """This method gets the object of the point of interest that has thesame
        state as the requested state

        Args:
            state (str): The requested state

        Returns:
            [object]: an object of a point of interest with thesame state as the 
            requested state
        """
        rospy.wait_for_service('get_table_by_state')
        try:
            poi_state = rospy.ServiceProxy(
                'get_table_by_state',  GetTableByState)
            table = poi_state(state)
            return table
        except rospy.ServiceException as e:
            print('Service call failed: {e}'.format(e=e))

    def execute(self, userdata):
        """This is the function that is called when the state machine is at this state

        Args:
            userdata (struct): This is a struct containing input and/or output data from/to the state

        Returns:
            [string]: This is the outcome of this state
        """
        object_detect_goal = ObjectDetectionAction()
        if (userdata.phase_no == 1):
            object_detect_goal.mode = 'detect'
            result = self.call_object_detect(object_detect_goal)
            if result.result == 'detected':
                userdata.no_of_object = result.no_of_object
                return 'object_detect_done'

        # Phase 2 does not require object detection
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
                    return 'correct_order'
                if (result.result == 'wrong'):
                    userdata.task = 'report wrong'
                    userdata.wrong_drinks = result.wrong_drinks
                    return 'wrong_order'
                if (result.result == 'missing'):
                    userdata.task = 'report missing'
                    userdata.missing_drinks = result.missing_drinks
                    return 'missing_order'
