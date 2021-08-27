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

# people perception package
from people_perception.msg import PeopleCounterAction, PeopleCounterGoal

# human robot interaction package
from sciroc_hri.msg import HRIAction, HRIGoal


def get_table_by_state(req):
    rospy.wait_for_service("get_table_object")
    try:
        poi_state = rospy.ServiceProxy("get_table_object", GetTableObject)
        req.mode = 0
        table = poi_state(req)
        return table
    except rospy.ServiceException as e:
        print("Service call failed: {e}".format(e=e))


def get_table_by_id(req):
    rospy.wait_for_service("get_table_object")
    try:
        poi_state = rospy.ServiceProxy("get_table_object", GetTableObject)
        req.mode = 1
        table = poi_state(req)
        return table
    except rospy.ServiceException as e:
        print("Service call failed: {e}".format(e=e))


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

    execute(userdata)
        this is the function that would be called when the state is called
    """

    def __init__(self, poi):
        """
        Args:
            poi ([list]): list containing all the point of interest from the poi parameter server

        """
        smach.State.__init__(
            outcomes=[
                "at_POI",
                "shop_explore_done",
                "at_counter",
                "at_require_order_table",
                "at_current_serving_table",
                "at_default_location",
                "trial_finished",
            ],
            output_keys=["current_poi", "task"],
            input_keys=["phase_no", "task"],
        )
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
        rospy.wait_for_service("go_to_poi_service")
        try:
            go_to_poi = rospy.ServiceProxy("go_to_poi_service", GoToPOI)
            result = go_to_poi(next_poi)

            if result.result == "goal reached":
                return True
            else:
                print("Point of interest [{poi}] does not exist".format(poi=poi))
                return False
        except rospy.ServiceException as e:
            print("Service call failed: {e}".format(e=e))

    def execute(self, userdata):
        """This is the function that is called when the state machine is at this state.

        Args:
            userdata (struct): This is a struct containing input and/or output data from/to the state

        Returns:
            [string]: This is the outcome of this state
        """
        # Check what phase the robot is at
        if userdata.phase_no == 1:
            if len(self.poi) == 0:
                next_poi = self.counter
                result = self.call_nav_service(next_poi)
                if result:
                    userdata.current_poi = next_poi
                    return "shop_explore_done"
            else:
                next_poi = self.poi.pop()
                result = self.call_nav_service(next_poi)
                if result:
                    userdata.current_poi = next_poi
                    return "at_POI"

        elif userdata.phase_no == 2:
            table_req = GetTableObjectRequest()
            table_req.table_state = "require order"
            table = get_table_by_state(table_req)
            if len(table.require_order_list) > 0:
                result = self.call_nav_service(table.table_id)
                if result:
                    userdata.current_poi = table.table_id
                    return "at_require_order_table"
            else:
                result = self.call_nav_service(self.counter)
                return "trial_finished"

        elif userdata.phase_no == 3:
            if userdata.task == "report order":
                result = self.call_nav_service(self.counter)
                if result:
                    userdata.current_poi = self.counter
                    return "at_counter"
            if userdata.task == "deliver order":
                table_req = GetTableObjectRequest()
                table_req.table_state = "current serving"
                table = get_table_by_state(table_req)
                result = self.call_nav_service(table.table_id)
                if result:
                    userdata.current_poi = table.table_id
                    userdata.task = "announce order arrival"
                    return "at_current_serving_table"

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
        smach.State.__init__(
            outcomes=["saved", "updated"],
            input_keys=[
                "phase_no",
                "current_poi",
                "no_of_people",
                "no_of_object",
                "order_list",
            ],
        )

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
        rospy.wait_for_service("update_poi_state")
        try:
            update_state = rospy.ServiceProxy("update_poi_state", UpdatePOIState)
            result = update_state(update_state_request)

            if result.result == "updated" or result.result == "saved":
                return True
        except rospy.ServiceException as e:
            print("Service call failed: {e}".format(e=e))

    def execute(self, userdata):
        """This is the function that is called when the state machine is at this state

        Args:
            userdata (struct): This is a struct containing input and/or output data from/to the state

        Returns:
            [string]: This is the outcome of this state
        """
        # PHASE 1
        if userdata.phase_no == 1:
            set_state_request = UpdatePOIStateRequest()
            set_state_request.task = "set"
            set_state_request.table_id = userdata.current_poi
            set_state_request.no_of_people = userdata.no_of_people
            set_state_request.no_of_object = userdata.no_of_object

            result = self.call_poi_state_service(update_state_request=set_state_request)
            if result:
                return "saved"
        # PHASE 2
        elif userdata.phase_no == 2:
            update_state_request = UpdatePOIStateRequest()
            update_state_request.task = "update"
            update_state_request.table_id = userdata.current_poi
            update_state_request.updated_states = [
                "require order",
                "required drinks",
                "current serving",
            ]
            update_state_request.require_order = False
            update_state_request.current_serving = True
            update_state_request.required_drinks = userdata.order_list

            result = self.call_poi_state_service(
                update_state_request=update_state_request
            )
            if result:
                return "updated"
        # PHASE 3
        elif userdata.phase_no == 3:
            update_state_request = UpdatePOIStateRequest()
            update_state_request.table_id = userdata.current_poi
            update_state_request.updated_states = [
                "need serving",
                "already served",
                "current serving",
            ]
            update_state_request.need_serving = False
            update_state_request.already_served = True
            update_state_request.current_serving = False
            result = self.call_poi_state_service(
                update_state_request=update_state_request
            )
            if result:
                return "updated"


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

    execute(userdata)
        this is the function that would be called when the state is called
    """

    def __init__(self):
        smach.State.__init__(
            outcomes=[
                "announced",
                "greeted",
                "order_taken",
                "order_reported",
                "missing_reported",
                "wrong_reported",
                "object_taken",
                "order_delivered",
                "wrong_and_missing_order_reported",
            ],
            output_keys=["task"],
            input_keys=[
                "phase_no",
                "current_poi",
                "task",
                "missing_drinks",
                "wrong_drinks",
            ],
        )

    def get_announce_text(self):
        pass

    def call_hri_action(self, goal_req):
        """This method sends a goal request to the hri action server

        Args:
            goal_req (str): the goal request message

        Returns:
            [str]: the result returned from the hri action server
        """
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient("hri", HRIAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(goal_req)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # return the result of executing the action
        return client.get_result()

    def execute(self, userdata):
        """This is the function that is called when the state machine is at this state

        Args:
            userdata (struct): This is a struct containing input and/or output data from/to the state

        Returns:
            [string]: This is the outcome of this state
        """

        hri_goal = HRIGoal()
        if userdata.phase_no == 1:
            if userdata.current_poi == "counter":
                hri_goal.mode = 0  # Announce text
                hri_goal.text = self.get_announce_text()
                result = self.call_hri_action(hri_goal)
                if result.result == "announced":
                    return "announced"
            else:
                hri_goal.mode = 2  # Greet Customer
                result = self.call_hri_action(hri_goal)
                if result.result:
                    return "greeted"

        elif userdata.phase_no == 2:
            hri_goal.mode = 1  # Take Order
            result = self.call_hri_action(hri_goal)
            if result.result:
                userdata.order_list = result.required_drinks
                return "order_taken"

        elif userdata.phase_no == 3:
            if userdata.task == "report order":
                hri_goal.mode = 0  # Announce Text
                hri_goal.text = self.get_announce_text()
                result = self.call_hri_action(hri_goal)
                if result.result:
                    userdata.task = "check object"
                    return "order_reported"

            elif userdata.task == "report missing":
                hri_goal.mode = 0  # Announce Text
                hri_goal.text = self.get_announce_text()
                # hri_goal.missing_drinks.extend(userdata.missing_drinks)
                result = self.call_hri_action(hri_goal)
                if result.result:
                    userdata.task = "check object"
                    return "missing_reported"

            elif userdata.task == "report wrong":
                hri_goal.mode = 0  # Announce Text
                hri_goal.text = self.get_announce_text()
                # hri_goal.wrong_drinks.extend(userdata.wrong_drinks)
                result = self.call_hri_action(hri_goal)
                if result.result:
                    userdata.task = "check object"
                    return "wrong_reported"

            elif userdata.task == "report wrong and missing":
                hri_goal.mode = 0  # Announce Text
                hri_goal.text = self.get_announce_text()
                # hri_goal.wrong_drinks.extend(userdata.wrong_drinks)
                # hri_goal.missing_drinks.extend(userdata.missing_drinks)
                result = self.call_hri_action(hri_goal)
                if result.result:
                    userdata.task = "check object"
                    return "wrong_and_missing_order_reported"

            elif userdata.task == "take item":
                hri_goal.mode = 3  # Take Item
                result = self.call_hri_action(hri_goal)
                if result.result:
                    userdata.task = "deliver order"
                    return "object_taken"
            elif userdata.task == "announce order arrival":
                hri_goal.mode = 4  # Drop Item
                result = self.call_hri_action(hri_goal)
                if result.result:
                    return "order_delivered"


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
    call_people_percept()
        for sending a goal request to the people perception action server

    execute(userdata)
        this is the function that would be called when the state is called
    """

    def __init__(self):
        # from here we define the possible outcomes of the state.
        smach.State.__init__(
            outcomes=["people_present", "people_not_present"],
            output_keys=["no_of_people"],
            input_keys=["phase_no"],
        )

    def call_people_percept(self):
        """This method sends a goal request to the people_percept action server

        Returns:
            [object]: the result returned from the people percept action server
        """
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient("people_detection", PeopleCounterAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Sends the goal to the action server.

        goal = PeopleCounterGoal()
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # return the result of executing the action
        return client.get_result()

    def execute(self, userdata):
        """This is the function that is called when the state machine is at this state
        Args:
            userdata (struct): This is a struct containing input and/or output data from/to the state
        Returns:
            [string]: This is the outcome of this state
        """

        if userdata.phase_no == 1:
            result = self.call_people_percept()
            userdata.no_of_people = result.n_people
            if result.n_people > 0:
                return "people_present"
            else:
                return "people_not_present"

        # People perception is not required in both phase 1 & 2
        elif userdata.phase_no == 2:
            pass
        elif userdata.phase_no == 3:
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

    execute(userdata)
        this is the function that would be called when the state is called
    """

    def __init__(self):
        # from here we define the possible outcomes of the state.
        smach.State.__init__(
            outcomes=[
                "object_detect_done",
                "correct_order",
                "wrong_order",
                "missing_order",
                "wrong_and_missing_order",
            ],
            output_keys=["no_of_object", "task", "wrong_drinks", "missing_drinks"],
            input_keys=["phase_no", "task"],
        )

    def call_object_detect(self, goal_req):
        """This method sends a goal request to the object detection action server

        Args:
            goal_req (str): the goal request message

        Returns:
            [object]: the result returned from the object detection action server
        """
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient("object_detect", ObjectPerceptionAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(goal_req)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # return the result of executing the action
        return client.get_result()

    def check_wrong_drinks(self, expected_tags, found_tags):
        wrong_drinks = []
        for drink in found_tags:
            if drink not in expected_tags:
                wrong_drinks.append(drink)
        return wrong_drinks

    def check_missing_drinks(self, expected_tags, found_tags):
        missing_drinks = []
        for drink in expected_tags:
            if drink not in found_tags:
                missing_drinks.append(drink)
        return missing_drinks

    def execute(self, userdata):
        """This is the function that is called when the state machine is at this state

        Args:
            userdata (struct): This is a struct containing input and/or output data from/to the state

        Returns:
            [string]: This is the outcome of this state
        """
        object_detect_goal = ObjectDetectionGoal()
        if userdata.phase_no == 1:
            object_detect_goal.mode = 0  # Enumeration
            result = self.call_object_detect(object_detect_goal)
            userdata.no_of_object = result.n_found_tags
            return "object_detect_done"

        # Phase 2 does not require object detection
        elif userdata.phase_no == 2:
            pass

        elif userdata.phase_no == 3:
            if userdata.task == "check order":
                table_req = GetTableObjectRequest()
                table_req.table_state = "current serving"
                table = get_table_by_state(table_req)
                object_detect_goal.mode = 2  # Comparison
                object_detect_goal.expected_tags = table.required_drinks
                result = self.call_object_detect(object_detect_goal)
                if result.match:
                    userdata.task = "take item"
                    return "correct_order"
                if result.match == False:
                    missing_drinks = self.check_missing_drinks(
                        expected_tags=table.required_drinks,
                        found_tags=result.found_tags,
                    )
                    wrong_drinks = self.check_wrong_drinks(
                        expected_tags=table.required_drinks,
                        found_tags=result.found_tags,
                    )
                    if len(missing_drinks) > 0:
                        userdata.task = "report missing"
                        userdata.missing_drinks = missing_drinks
                        return "missing_order"
                    elif len(wrong_drinks) > 0:
                        userdata.task = "report wrong"
                        userdata.wrong_drinks = wrong_drinks
                        return "wrong_order"
                    elif len(wrong_drinks) > 0 and len(missing_drinks) > 0:
                        userdata.task = "report wrong and missing"
                        userdata.missing_drinks = missing_drinks
                        userdata.wrong_drinks = wrong_drinks
                        return "wrong_and_missing_order"


if __name__ == "__main__":
    rospy.init_node("sciroc_state_machine")

    poi = ["counter", "table1", "table2", "table3", "table4", "table5", "table6"]

    # Create a SMACH state machine
    Trial = smach.StateMachine(outcomes=["trial_finished"])

    with Trial:

        Phase1 = smach.StateMachine(outcomes=["phase1_finished"])
        Phase1.userdata.phase_value = 1

        # Open the container
        with Phase1:

            # Add states to the container
            smach.StateMachine.add(
                "NAVIGATE",
                Navigate(poi),
                transitions={
                    "shop_explore_done": "HRI(Speak)",
                    "at_POI": "DETECT_PEOPLE",
                },
                remapping={"phase_no": "phase_value"},
            )

            smach.StateMachine.add(
                "DETECT_PEOPLE",
                PeoplePerception(),
                transitions={
                    "people_present": "HRI(Speak)",
                    "people_not_precent": "DETECT_OBJECT",
                },
                remapping={"phase_no": "phase_value"},
            )

            smach.StateMachine.add(
                "DETECT_OBJECT",
                ObjectDetection(),
                transitions={"object_detect_done": "SAVE_POI_STATE"},
                remapping={"phase_no": "phase_value"},
            )

            smach.StateMachine.add(
                "HRI(Speak)",
                HRI(),
                transitions={
                    "greeted": "DETECT_OBJECT",
                    "announced": "phase1_finished",
                },
                remapping={"phase_no": "phase_value", "current_poi": "current_poi"},
            )

            smach.StateMachine.add(
                "SAVE_POI_STATE",
                POI_State(),
                transitions={"saved": "NAVIGATE"},
                remapping={
                    "phase_no": "phase_value",
                    "current_poi": "current_poi",
                    "no_of_people": "no_of_people",
                    "no_of_object": "no_of_object",
                },
            )

        smach.StateMachine.add(
            "PHASE_1", Phase1, transitions={"phase1_finished": "PHASE_2"}
        )

        Phase2 = smach.StateMachine(outcomes=["phase2_finished", "at_default_location"])
        Phase2.userdata.phase_value = 2

        # Open the container
        with Phase2:
            smach.StateMachine.add(
                "NAVIGATE",
                Navigate(poi),
                transitions={
                    "at_require_order_table": "HRI(TakeOrder)",
                    "at_default_location": "at_default_location",
                },
                remapping={"phase_no": "phase_value"},
            )

            smach.StateMachine.add(
                "HRI(TakeOrder)",
                HRI(),
                transitions={"order_taken": "UPDATE_POI_STATE"},
                remapping={"phase_no": "phase_value"},
            )

            smach.StateMachine.add(
                "UPDATE_POI_STATE",
                POI_State(),
                transitions={"updated": "phase2_finished"},
                remapping={
                    "phase_no": "phase_value",
                    "current_poi": "current_poi",
                    "order_list": "order_list",
                },
            )

        smach.StateMachine.add(
            "PHASE_2",
            Phase2,
            transitions={
                "phase2_finished": "PHASE_3",
                "at_default_location": "trial_finished",
            },
        )

        Phase3 = smach.StateMachine(outcomes=["phase3_finished"])
        Phase3.userdata.phase_value = 3
        Phase3.userdata.task = "report order"

        # Open the container
        with Phase3:
            smach.StateMachine.add(
                "NAVIGATION",
                Navigate(poi),
                transitions={
                    "at_counter": "HRI(Speak)",
                    "at_current_serving_table": "HRI(Speak)",
                },
                remapping={"phase_no": "phase_value", "task": "task"},
            )

            smach.StateMachine.add(
                "HRI(Speak)",
                HRI(),
                transitions={
                    "order_reported": "CHECK_OBJECT",
                    "missing_reported": "CHECK_OBJECT",
                    "wrong_reported": "CHECK_OBJECT",
                    "wrong_and_missing_order_reported": "CHECK_OBJECT",
                    "object_taken": "NAVIGATE",
                    "order_delivered": "UPDATE_POI_STATE",
                },
                remapping={
                    "phase_no": "phase_value",
                    "task": "task",
                    "missing_drinks": "missing_drinks",
                    "wrong_drinks": "wrong_drinks",
                },
            )

            smach.StateMachine.add(
                "CHECK_OBJECT",
                ObjectDetection(),
                transitions={
                    "correct_order": "HRI(Speak)",
                    "wrong_order": "HRI(Speak)",
                    "missing_order": "HRI(Speak)",
                    "wrong_and_missing_order": "HRI(Speak)",
                },
                remapping={"phase_no": "phase_value", "task": "task"},
            )

            smach.StateMachine.add(
                "UPDATE_POI_STATE",
                POI_State(),
                transitions={"updated": "phase3_finished"},
                remapping={"phase_no": "phase_value", "current_poi": "current_poi"},
            )

        smach.StateMachine.add(
            "PHASE_3", Phase3, transitions={"phase3_finished": "PHASE_2"}
        )

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer("server_name", Trial, "/SM_ROOT")
    sis.start()

    # Execute SMACH plan
    outcome = Trial.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
