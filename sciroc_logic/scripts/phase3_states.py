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

# pal_head_manager control disable
from pal_common_msgs.msg import DisableAction, DisableGoal
import time

counter = "counter"


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


###+++++++++++++++++++ NAVIGATION +++++++++++++++++++++###


class Navigate(smach.State):
    def __init__(self):

        smach.State.__init__(
            self,
            outcomes=["at_counter", "at_current_serving_table", "at_default_location"],
            output_keys=["current_poi", "task"],
            input_keys=["task"],
        )

    def call_nav_service(self, next_poi):
        rospy.wait_for_service("go_to_poi_service")
        try:
            go_to_poi = rospy.ServiceProxy("go_to_poi_service", GoToPOI)
            result = go_to_poi(next_poi)

            if result.result == "goal reached":
                return True
            else:
                print("Point of interest [{poi}] does not exist".format(poi=next_poi))
                return False
        except rospy.ServiceException as e:
            print("Service call failed: {e}".format(e=e))

    def execute(self, userdata):
        if userdata.task == "report order":
            result = self.call_nav_service(counter)
            # result = True
            # time.sleep(2)
            if result:
                userdata.current_poi = counter
                return "at_counter"
        if userdata.task == "deliver order":
            table_req = GetTableObjectRequest()
            table_req.table_state = "current serving"
            table = get_table_by_state(table_req)
            result = self.call_nav_service(table.table_id)
            # time.sleep(2)
            # result = True
            if result:
                userdata.current_poi = table.table_id
                userdata.task = "announce order arrival"
                return "at_current_serving_table"
        if userdata.task == "go to default location":
            # time.sleep(2)
            result = self.call_nav_service(counter)
            # result = True
            if result:
                userdata.current_poi = counter
                return "at_default_location"


###+++++++++++++++++++ HUMAN ROBOT INTERACTION (HRI) +++++++++++++++++++++###


class HRI(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                "order_reported",
                "missing_reported",
                "wrong_reported",
                "object_taken",
                "order_delivered",
                "wrong_and_missing_order_reported",
            ],
            output_keys=["task"],
            input_keys=[
                "task",
                "missing_drinks",
                "wrong_drinks",
            ],
        )

    def get_announce_text(self, task, missing=[], wrong=[]):
        if task == "report order":
            table_req = GetTableObjectRequest()
            table_req.table_state = "current serving"
            table = get_table_by_state(table_req)
            text = (
                "The customers on   " + table.table_id + "   has made an Order for   "
            )
            for item in table.required_drinks:
                text = text + "   " + item + "   "

            res_text = (
                text
                + "   I will be here   waiting for you to place the order on the counter  "
            )
            return res_text
        elif task == "report missing":
            table_req = GetTableObjectRequest()
            table_req.table_state = "current serving"
            table = get_table_by_state(table_req)
            text = (
                "The customers on   " + table.table_id + "   has made an Order for   "
            )
            for item in table.required_drinks:
                text = text + "   " + item + "   "
            miss_text = " "
            for items in missing:
                miss_text = miss_text + "   " + items + "   "
            res_text = (
                "I'm sorry   but    "
                + miss_text
                + " is missing from the requested order   "
                + text
            )
            return res_text
        elif task == "report wrong":
            table_req = GetTableObjectRequest()
            table_req.table_state = "current serving"
            table = get_table_by_state(table_req)
            text = (
                "The customers on   " + table.table_id + "   has made an Order for   "
            )
            for item in table.required_drinks:
                text = text + "   " + item + "   "
            wrong_text = " "
            for items in wrong:
                wrong_text = wrong_text + "   " + items + "   "
            res_text = (
                "I'm sorry   but  i did not ask for  " + wrong_text + "   " + text
            )
            return res_text
        elif task == "report wrong and missing":
            table_req = GetTableObjectRequest()
            table_req.table_state = "current serving"
            table = get_table_by_state(table_req)
            text = (
                "The customers on   " + table.table_id + "   has made an Order for   "
            )
            for item in table.required_drinks:
                text = text + "   " + item + "   "
            wrong_text = " "
            for items in wrong:
                wrong_text = wrong_text + "   " + items + "   "
            miss_text = " "
            for items in missing:
                miss_text = miss_text + "   " + items + "   "

            res_text = (
                "I'm sorry    but   i did not ask for   "
                + wrong_text
                + "   and also   "
                + miss_text
                + "  is missing from the order"
            )
            return res_text

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

        if userdata.task == "report order":
            hri_goal.mode = 0  # Announce Text
            hri_goal.text = self.get_announce_text(task=userdata.task)
            result = self.call_hri_action(hri_goal)
            # time.sleep(2)
            # result = True
            if result:
                userdata.task = "check object"
                return "order_reported"

        elif userdata.task == "report missing":
            hri_goal.mode = 0  # Announce Text
            hri_goal.text = self.get_announce_text(
                task=userdata.task, missing=userdata.missing_drinks
            )
            result = self.call_hri_action(hri_goal)
            # result = True
            # time.sleep(2)
            if result:
                userdata.task = "check object"
                return "missing_reported"

        elif userdata.task == "report wrong":
            hri_goal.mode = 0  # Announce Text
            hri_goal.text = self.get_announce_text(
                task=userdata.task, wrong=userdata.wrong_drinks
            )
            result = self.call_hri_action(hri_goal)
            # result = True
            # time.sleep(2)
            if result:
                userdata.task = "check object"
                return "wrong_reported"

        elif userdata.task == "report wrong and missing":
            hri_goal.mode = 0  # Announce Text
            hri_goal.text = self.get_announce_text(
                task=userdata.task,
                wrong=userdata.wrong_drinks,
                missing=userdata.missing_drinks,
            )
            result = self.call_hri_action(hri_goal)
            # result = True
            # time.sleep(2)
            if result:
                userdata.task = "check object"
                return "wrong_and_missing_order_reported"

        elif userdata.task == "take item":
            hri_goal.mode = 3  # Take Item
            result = self.call_hri_action(hri_goal)
            # result = True
            # Sleep to allow for the barista to put the items on the tray
            # time.sleep(10)
            if result:
                userdata.task = "deliver order"
                return "object_taken"
        elif userdata.task == "announce order arrival":
            hri_goal.mode = 4  # Drop Item
            result = self.call_hri_action(hri_goal)
            # result = True
            # Sleep to allow for the customers to retrieve the orders
            # time.sleep(12)
            if result:
                return "order_delivered"


###+++++++++++++++++++ OBJECT DETECTION  +++++++++++++++++++++###


class ObjectDetection(smach.State):
    def __init__(self, head_ctrl_dsbl_client):
        # from here we define the possible outcomes of the state.
        smach.State.__init__(
            self,
            outcomes=[
                "correct_order",
                "wrong_order",
                "missing_order",
                "wrong_and_missing_order",
            ],
            output_keys=["task", "wrong_drinks", "missing_drinks"],
            input_keys=["task", "current_poi"],
        )
        self.head_ctrl_dsbl_client = head_ctrl_dsbl_client

    def call_object_detect(self, goal_req):
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient("objdet_interface", ObjDetInterfaceAction)

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
        if self.head_ctrl_dsbl_client:
            dsbl_head_ctrl_goal = DisableGoal()
            # TODO: get this duration from param server, not hardcoded
            dsbl_head_ctrl_goal.duration = 10
            self.head_ctrl_dsbl_client.send_goal(dsbl_head_ctrl_goal)
            # no need for waiting
        object_detect_goal = ObjDetInterfaceGoal()

        if userdata.task == "check object":
            table_req = GetTableObjectRequest()
            table_req.table_state = "current serving"
            table = get_table_by_state(table_req)
            object_detect_goal.mode = 2  # Comparison
            object_detect_goal.expected_tags = table.required_drinks

            rospy.loginfo("ObjDet expected tags %s " % object_detect_goal.expected_tags)

            result = self.call_object_detect(object_detect_goal)
            # time.sleep(2)
            # result = True
            if result.match:
                userdata.task = "take item"
                return "correct_order"
            elif not result.match:
                # missing_drinks = ["", ""]
                # wrong_drinks = []
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


###+++++++++++++++++++ POINT OF INTEREST STATE (POI STATE) +++++++++++++++++++++###


class POI_State(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["updated", "final_update_done"],
            input_keys=["current_poi"],
            output_keys=["task"],
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
        update_state_request = UpdatePOIStateRequest()
        update_state_request.task = "update"
        update_state_request.table_id = userdata.current_poi
        update_state_request.updated_states = [
            "need serving",
            "already served",
            "current serving",
        ]
        update_state_request.need_serving = False
        update_state_request.already_served = True
        update_state_request.current_serving = False
        result = self.call_poi_state_service(update_state_request=update_state_request)
        time.sleep(2)
        if result:
            table_req = GetTableObjectRequest()
            table_req.table_state = "require order"
            table = get_table_by_state(table_req)
            if len(table.require_order_list) > 0:
                # if there still exist a table that need serving
                userdata.task = "report order"
                return "updated"
            else:
                # if there is no more table that need serving
                userdata.task = "go to default location"
                return "final_update_done"
