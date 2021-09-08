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
from people_perception.msg import (
    PeopleCounter2Action,
    PeopleCounter2Goal,
    PeopleCounter2Result
)

# human robot interaction package
from sciroc_hri.msg import HRIAction, HRIGoal, HRIResult

from sciroc_objdet.msg import (
	ObjDetInterfaceAction,
	ObjDetInterfaceGoal,
	ObjDetInterfaceResult,
)

# pal_head_manager control disable
from pal_common_msgs.msg import (
	DisableAction,
	DisableGoal
)


import time

counter = "counter"
#poi = ["t1", "t2", "t3", "t4", "t5", "t6"]


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
			intput_keys=["poi"],
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
				print("Point of interest [{poi}] does not exist".format(poi=next_poi))
				return False
		except rospy.ServiceException as e:
			print("Service call failed: {e}".format(e=e))

	def execute(self, userdata):
		if len(poi) == 0:
			next_poi = counter
			result = self.call_nav_service(next_poi)
			#result = True
			#time.sleep(2)
			if result:
				userdata.current_poi = next_poi
				return "shop_explore_done"
		elif len(poi) > 0:
			next_poi = poi.pop(0)
			result = self.call_nav_service(next_poi)
			#result = True
			time.sleep(2)
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
		time.sleep(2)
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
				"announced_and_done"
			],
			input_keys=["current_poi"],
		)

	
	def get_announce_text():
		table_req = GetTableObjectRequest()
		table_req.table_state = "require order"
		table = get_table_by_state(table_req)

		if len(table.need_serving_list) == 0:
			need_serving_text = "no table"
			customer_text = ""
		elif len(table.need_serving_list) > 0:
			need_serving_text = " "
			customer_text = " "
			last_table = table.need_serving_list.pop()
			tabl_req = GetTableObjectRequest()
			tabl_req.table_id = last_table
			tabl = get_table_by_id(tabl_req)
			last_customer_text = (
				last_table + "  has " + str(tabl.no_of_people) + " customers waiting "
			)
			if len(table.need_serving_list) > 0:
				for item in table.need_serving_list:
					need_serving_text = need_serving_text + item + " " + "and "
					tab_req = GetTableObjectRequest()
					tab_req.table_id = last_table
					tab = get_table_by_id(tab_req)
					customer_text = ("  "+
						customer_text
						+ item
						+ " has "
						+ str(tab.no_of_people)
						+ " customers waiting and "
					)
			customer_text = customer_text + last_customer_text + ", "
			need_serving_text = need_serving_text + last_table + " needs serving, "

		if len(table.need_cleaning_list) == 0:
			need_cleaning_text = "no table"
		elif len(table.need_cleaning_list) > 0:
			need_cleaning_text = " "
			last_table = table.need_cleaning_list.pop()
			if len(table.need_cleaning_list) > 0:
				for item in table.need_cleaning_list:
					need_cleaning_text = "   " + need_cleaning_text + item + " " + "and  "
			need_cleaning_text = (
				need_cleaning_text + last_table + " needs to be cleaned,  "
			)

		if len(table.ready_list) == 0:
			ready_text = "no table"
		elif len(table.ready_list) > 0:
			ready_text = " "
			last_table = table.ready_list.pop()
			if len(table.ready_list) > 0:
				for item in table.ready_list:
					ready_text = ready_text + item + " " + "and "
			ready_text = ready_text + last_table + " is ready to take new customers, "

		if len(table.already_served_list) == 0:
			already_served_text = "no table is"
		elif len(table.already_served_list) > 0:
			already_served_text = " "
			last_table = table.already_served_list.pop()
			if len(table.already_served_list) > 0:
				for item in table.already_served_list:
					already_served_text = already_served_text + item + " " + "and "
			already_served_text = (
				already_served_text + last_table + " is already served  "
			)

		announce_text = (
			"Hello there Barrista,   how's it going?    After exploring the restaurant,   here are the states of each of the tables.  "
			+ need_serving_text
			+ customer_text
			+ need_cleaning_text
			+ ready_text
			+ already_served_text
		)
		return announce_text

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
			hri_goal.text = self.get_announce_text()
			# result = self.call_hri_action(hri_goal)
			result = True
			time.sleep(2)
			if result:
				table_req = GetTableObjectRequest()
				table_req.table_state = "require order"
				table = get_table_by_state(table_req)
				if len(table.require_order_list) > 0:
					# if there exist a table that need serving 
					return "announced"
				else: 
					# if there is no table that needs serving 
					return "announced_and_done"
		else:
			hri_goal.mode = 2  # Greet Customer
			# result = self.call_hri_action(hri_goal)
			result = True
			time.sleep(2)
			if result:
				return "greeted"


###+++++++++++++++++++ PEOPLE PERCEPTION +++++++++++++++++++++###


class PeoplePerception(smach.State):
	def __init__(self, head_ctrl_dsbl_client):
		# from here we define the possible outcomes of the state.
		smach.State.__init__(
			self,
			outcomes=["people_present", "people_not_present"],
            input_keys=["current_poi"],
			output_keys=["no_of_people"],
		)
		self.head_ctrl_dsbl_client = head_ctrl_dsbl_client

	def call_people_percept(self, userdata):
		# Creates the SimpleActionClient, passing the type of the action

		client = actionlib.SimpleActionClient("people_detection", PeopleCounter2Action)

		# Waits until the action server has started up and started
		# listening for goals.
		client.wait_for_server()

		# Sends the goal to the action server.

		goal = PeopleCounter2Goal()
		goal.table_poi = userdata.current_poi
		client.send_goal(goal)
		# Waits for the server to finish performing the action.
		client.wait_for_result()

		# return the result of executing the action
		return client.get_result()

	def execute(self, userdata):
		if self.head_ctrl_dsbl_client:
			dsbl_head_ctrl_goal = DisableGoal()
			# TODO: get this duration from param server, not hardcoded
			dsbl_head_ctrl_goal.duration = 10
			self.head_ctrl_dsbl_client.send_goal(dsbl_head_ctrl_goal)
			# no need for waiting

		result = self.call_people_percept(userdata)
		userdata.no_of_people = result.n_people
		
		#userdata.no_of_people = n_people

		if userdata.no_of_people > 0:
			return "people_present"
		else:
			return "people_not_present"


###+++++++++++++++++++ OBJECT DETECTION  +++++++++++++++++++++###


class ObjectDetection(smach.State):
	def __init__(self, head_ctrl_dsbl_client):
		# from here we define the possible outcomes of the state.
		smach.State.__init__(
			self,
			outcomes=[
				"object_detect_done",
			],
			input_keys=["current_poi"],
			output_keys=["no_of_object"],
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

	def execute(self, userdata):
		if self.head_ctrl_dsbl_client:
			dsbl_head_ctrl_goal = DisableGoal()
			# TODO: get this duration from param server, not hardcoded
			dsbl_head_ctrl_goal.duration = 10
			self.head_ctrl_dsbl_client.send_goal(dsbl_head_ctrl_goal)
			# no need for waiting
			
		object_detect_goal = ObjDetInterfaceGoal()
		object_detect_goal.mode = 0  # Enumeration

		## TODO: Here we should not actually pass the current poi,
		## but the poi of pass the POI of the table, not of
		## its "approaching point"
		object_detect_goal.table_id = userdata.current_poi
		result = self.call_object_detect(object_detect_goal)
		n_found_tags = result.n_found_tags
		userdata.no_of_object = n_found_tags
		#time.sleep(2)
		return "object_detect_done"