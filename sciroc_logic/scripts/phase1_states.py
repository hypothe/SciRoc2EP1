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

import time

counter = "counter"
poi = ["t1", "t2", "t3", "t4", "t5", "t6"]


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
        smach.State.__init__(
            self,
            outcomes=[
                "at_POI",
                "shop_explore_done",
            ],
            output_keys=["current_poi"],
        )

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
        if len(poi) == 0:
            next_poi = counter
            # result = self.call_nav_service(next_poi)
            result = True
            time.sleep(5)
            if result:
                userdata.current_poi = next_poi
                return "shop_explore_done"
        elif len(poi) > 0:
            next_poi = poi.pop(0)
            # result = self.call_nav_service(next_poi)
            result = True
            time.sleep(5)
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
                "current_poi",
                "no_of_people",
                "no_of_object",
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
        set_state_request = UpdatePOIStateRequest()
        set_state_request.task = "set"
        set_state_request.table_id = userdata.current_poi
        set_state_request.no_of_people = userdata.no_of_people
        set_state_request.no_of_object = userdata.no_of_object

        result = self.call_poi_state_service(update_state_request=set_state_request)
        time.sleep(5)
        if result:
            return "saved"


###+++++++++++++++++++ HUMAN ROBOT INTERACTION (HRI) +++++++++++++++++++++###


class HRI(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                "announced",
                "greeted",
            ],
            input_keys=["current_poi"],
        )

    def get_announce_text(self):
        pass

    def call_hri_action(self, goal_req):
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
        hri_goal = HRIGoal()
        if userdata.current_poi == "counter":
            hri_goal.mode = 0  # Announce text
            # hri_goal.text = self.get_announce_text()
            # result = self.call_hri_action(hri_goal)
            result = True
            time.sleep(5)
            if result:
                return "announced"
        else:
            hri_goal.mode = 2  # Greet Customer
            # result = self.call_hri_action(hri_goal)
            result = True
            time.sleep(5)
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
        )

    def call_people_percept(self):
        # Creates the SimpleActionClient, passing the type of the action

        client = actionlib.SimpleActionClient("people_detection", PeopleCounterAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Sends the goal to the action server.

        goal = PeopleCounterGoal()
        # client.send_goal(goal)
        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # return the result of executing the action
        return client.get_result()

    def execute(self, userdata):
        # result = self.call_people_percept()
        # userdata.no_of_people = result.n_people
        n_people = 3
        userdata.no_of_people = n_people
        time.sleep(5)

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
            output_keys=["no_of_object"],
        )

    def call_object_detect(self, goal_req):
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

    def execute(self, userdata):
        object_detect_goal = ObjDetInterfaceGoal()
        object_detect_goal.mode = 0  # Enumeration
        # result = self.call_object_detect(object_detect_goal)
        n_found_tags = 0
        userdata.no_of_object = n_found_tags
        time.sleep(5)
        return "object_detect_done"
