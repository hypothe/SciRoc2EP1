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

poi = ["counter", "t1", "t2", "t3", "t4", "t5", "t6"]


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


class Navigate(smach.State):
    def __init__(self):
        """
        Args:
            poi ([list]): list containing all the point of interest from the poi parameter server

        """

        smach.State.__init__(
            self,
            outcomes=[
                "at_POI",
                "shop_explore_done",
            ],
            output_keys=["current_poi", "task"],
            input_keys=["phase_no", "task"],
        )
        # This would be changed later, only here for testing reasons
        self.poi = poi[1:]
        self.counter = poi[0]

    def call_nav_service(self, next_poi):
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
        # Check what phase the robot is at
        if userdata.phase_no == 1:
            if len(self.poi) == 0:
                next_poi = self.counter
                # result = self.call_nav_service(next_poi)
                result = True
                if result:
                    userdata.current_poi = next_poi
                    return "shop_explore_done"
            else:
                next_poi = self.poi.pop()
                # result = self.call_nav_service(next_poi)
                result = True
                if result:
                    userdata.current_poi = next_poi
                    return "at_POI"


###+++++++++++++++++++ POINT OF INTEREST STATE (POI STATE) +++++++++++++++++++++###


class POI_State(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["saved"],
            input_keys=[
                "phase_no",
                "current_poi",
                "no_of_people",
                "no_of_object",
                "order_list",
            ],
        )

    def call_poi_state_service(self, update_state_request=UpdatePOIStateRequest()):

        rospy.wait_for_service("update_poi_state")
        try:
            update_state = rospy.ServiceProxy("update_poi_state", UpdatePOIState)
            result = update_state(update_state_request)

            if result.result == "updated" or result.result == "saved":
                return True
        except rospy.ServiceException as e:
            print("Service call failed: {e}".format(e=e))

    def execute(self, userdata):

        # PHASE 1
        if userdata.phase_no == 1:
            set_state_request = UpdatePOIStateRequest()
            set_state_request.task = "set"
            set_state_request.table_id = userdata.current_poi
            set_state_request.no_of_people = userdata.no_of_people
            set_state_request.no_of_object = userdata.no_of_object

            # result = self.call_poi_state_service(update_state_request=set_state_request)
            result = True
            if result:
                return "saved"


###+++++++++++++++++++ HUMAN ROBOT INTERACTION (HRI) +++++++++++++++++++++###


class HRI(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                "announced",
                "greeted"
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
        # client = actionlib.SimpleActionClient("hri", HRIAction)

        # Waits until the action server has started up and started
        # listening for goals.
        # client.wait_for_server()

        # Sends the goal to the action server.
        # client.send_goal(goal_req)

        # Waits for the server to finish performing the action.
        # client.wait_for_result()

        # return the result of executing the action
        # return client.get_result()
        result = HRIResult()
        result.result = True
        result.order_list = ["", "", ""]

    def execute(self, userdata):

        hri_goal = HRIGoal()
        if userdata.phase_no == 1:
            if userdata.current_poi == "counter":
                hri_goal.mode = 0  # Announce text
                # hri_goal.text = self.get_announce_text()
                # result = self.call_hri_action(hri_goal)
                result = True
                if result:
                    return "announced"
            else:
                hri_goal.mode = 2  # Greet Customer
                # result = self.call_hri_action(hri_goal)
                result = True
                if result:
                    return "greeted"


###+++++++++++++++++++ PEOPLE PERCEPTION +++++++++++++++++++++###


class PeoplePerception(smach.State):
    def __init__(self):
        # from here we define the possible outcomes of the state.
        smach.State.__init__(
            self,
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

        # client = actionlib.SimpleActionClient("people_detection", PeopleCounterAction)

        # Waits until the action server has started up and started
        # listening for goals.
        # client.wait_for_server()

        # Sends the goal to the action server.

        # goal = PeopleCounterGoal()
        # client.send_goal(goal)
        # Waits for the server to finish performing the action.
        # client.wait_for_result()

        # return the result of executing the action
        # return client.get_result()
        _res = PeopleCounterResult()
        _res.n_people = 4
        return _res

    def execute(self, userdata):
        """This is the function that is called when the state machine is at this state
        Args:
            userdata (struct): This is a struct containing input and/or output data from/to the state
        Returns:
            [string]: This is the outcome of this state
        """

        if userdata.phase_no == 1:
            # result = self.call_people_percept()
            # userdata.no_of_people = result.n_people
            n_people = 3
            userdata.no_of_people = n_people
            if n_people > 0:
                return "people_present"
            else:
                return "people_not_present"


###+++++++++++++++++++ OBJECT DETECTION  +++++++++++++++++++++###


class ObjectDetection(smach.State):
    def __init__(self):
        # from here we define the possible outcomes of the state.
        smach.State.__init__(
            self,
            outcomes=[
                "object_detect_done",
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
        # client = actionlib.SimpleActionClient("object_detect", ObjectPerceptionAction)

        # Waits until the action server has started up and started
        # listening for goals.
        # client.wait_for_server()

        # Sends the goal to the action server.
        # client.send_goal(goal_req)

        # Waits for the server to finish performing the action.
        # client.wait_for_result()

        # return the result of executing the action
        # return client.get_result()
        result = ObjDetInterfaceResult()
        result.n_found_tags = 2
        result.found_tags = ["bottle", "fanta"]
        result.match = False
        return result

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

        object_detect_goal = ObjDetInterfaceGoal()
        if userdata.phase_no == 1:
            object_detect_goal.mode = 0  # Enumeration
            # result = self.call_object_detect(object_detect_goal)
            n_found_tags = 3
            userdata.no_of_object = n_found_tags
            return "object_detect_done"


if __name__ == "__main__":
    rospy.init_node("sciroc_state_machine")

    # poi = ["counter", "t1", "t2", "t3", "t4", "t5", "t6"]

    # Create a SMACH state machine

    Phase1 = smach.StateMachine(outcomes=["phase1_finished"])
    Phase1.userdata.phase_value = 1

    # Open the container
    with Phase1:

        # Add states to the container
        smach.StateMachine.add(
            "NAVIGATE",
            Navigate(),
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
                "people_not_present": "DETECT_OBJECT",
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

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer("server_name", Phase1, "/SciRoc2EP1 Logic State Machine")
    sis.start()

    # Execute SMACH plan
    outcome = Phase1.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
