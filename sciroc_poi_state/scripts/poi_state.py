#!/usr/bin/env python

import rospy
from sciroc_poi_state.srv import UpdatePOIState, GetTableObject
from sciroc_poi_state.srv import UpdatePOIStateResponse, GetTableObjectResponse


POI = {}  # for storing all the table objects

table_states = {
    "require order": [],
    "need serving": [],
    "already served": [],
    "need cleaning": [],
    "ready": [],
    "current serving": [],
}


class Table:
    def __init__(self, table_id, no_of_people, no_of_object):
        self.table_id = table_id
        self.no_of_people = no_of_people
        self.no_of_object = no_of_object
        self.need_serving = False
        self.need_cleaning = False
        self.require_order = False  # the customer needs the robot to take their order
        self.already_served = False
        self.current_serving = False
        self.required_drinks = []

        if no_of_people == 0 and no_of_object == 0:
            self.table_state = "ready"  # the table is ready to accept new customers

        elif no_of_people > 0 and no_of_object == 0:
            self.table_state = "need serving"
            self.need_serving = True
            self.require_order = True

        elif no_of_people > 0 and no_of_object > 0:
            self.table_state = "already served"
            self.already_served = True

        elif no_of_people == 0 and no_of_object > 0:
            self.table_state = "need cleaning"
            self.need_cleaning = True

    def update_state(self, req):
        for state in req.updated_states:
            if state == "need serving":
                self.need_serving = req.need_serving
            elif state == "require order":
                self.require_order = req.require_order
            elif state == "required drinks":
                self.required_drinks = req.required_drinks
            elif state == "no of objects":
                self.no_of_object = req.no_of_object
            elif state == "no of people":
                self.no_of_people = req.no_of_people
            elif state == "need cleaning":
                self.need_cleaning = req.need_cleaning
            elif state == "already served":
                self.already_served = req.already_served
            elif state == "current serving":
                self.current_serving = req.current_serving

        if (
            self.need_serving == False
            and self.require_order == False
            and self.need_cleaning == False
            and self.already_served == False
        ):
            self.table_state = "ready"
        if (
            self.need_serving == True
            and self.require_order == True
            and self.need_cleaning == False
            and self.already_served == False
        ):
            self.table_state = "need serving"
        if (
            self.need_serving == True
            and self.require_order == False
            and self.need_cleaning == False
            and self.already_served == False
        ):
            self.table_state = "need serving"
        if (
            self.need_serving == False
            and self.require_order == False
            and self.need_cleaning == False
            and self.already_served == True
        ):
            self.table_state = "already served"
        if (
            self.need_serving == False
            and self.require_order == False
            and self.need_cleaning == True
            and self.already_served == False
        ):
            self.table_state = "need cleaning"

        return True


def update_poi_state(req):
    global POI
    if req.task == "set":
        POI[req.table_id] = Table(
            table_id=req.table_id,
            no_of_people=req.no_of_people,
            no_of_object=req.no_of_object,
        )
        return UpdatePOIStateResponse("saved")
    elif req.task == "update":
        result = POI[req.table_id].update_state(req)
        if result:
            response = "updated"
        return UpdatePOIStateResponse(response)


def update_table_state_data():
    global table_states, POI
    table_states = {
        "require order": [],
        "need serving": [],
        "already served": [],
        "need cleaning": [],
        "ready": [],
        "current serving": [],
    }
    for table_id in POI:
        table_states[POI[table_id].table_state].append(table_id)
        if POI[table_id].require_order:
            table_states["require order"].append(table_id)
        elif POI[table_id].current_serving:
            table_states["current serving"].append(table_id)


def generate_table_response(req_mes):
    global table_states, POI
    table_response = GetTableObjectResponse()

    if req_mes.mode == 0:
        table_ids = table_states[req_mes.table_state]
        table = POI[table_ids[0]]
    elif req_mes.mode == 1:
        table = POI[req_mes.table_id]

    if len(table_ids) > 0 or req_mes.mode == 1:
        table_response.table_id = table.table_id
        table_response.no_of_people = table.no_of_people
        table_response.no_of_object = table.no_of_object
        table_response.need_serving = table.need_serving
        table_response.need_cleaning = table.need_cleaning
        table_response.require_order = table.require_order
        table_response.current_serving = table.current_serving
        table_response.already_served = table.already_served
        table_response.required_drinks = table.required_drinks
        table_response.need_serving_list = table_states["need serving"]
        table_response.need_cleaning_list = table_states["need cleaning"]
        table_response.require_order_list = table_states["require order"]
        table_response.already_served_list = table_states["already served"]
        table_response.current_serving_list = table_states["current serving"]

    else:
        table_response.need_serving_list = table_states["need serving"]
        table_response.need_cleaning_list = table_states["need cleaning"]
        table_response.require_order_list = table_states["require order"]
        table_response.already_served_list = table_states["already served"]
        table_response.current_serving_list = table_states["current serving"]

    return table_response


def get_table_object(req):
    global table_states, POI

    update_table_state_data()
    table_response = generate_table_response(req)
    return table_response


def main():
    rospy.init_node("POI_state")
    s1 = rospy.Service("update_poi_state", UpdatePOIState, update_poi_state)
    s2 = rospy.Service("get_table_by_state", GetTableObject, get_table_object)

    rospy.spin()


if __name__ == "__main__":
    main()
