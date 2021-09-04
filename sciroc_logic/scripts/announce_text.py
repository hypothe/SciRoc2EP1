#!/usr/bin/env python

import rospy
# message for updating and the getting the state of the POI (Point of Interest)
from sciroc_poi_state.srv import UpdatePOIState, GetTableObject
from sciroc_poi_state.srv import UpdatePOIStateRequest, GetTableObjectRequest


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



if __name__ == "__main__":
    rospy.init_node("announce_text")
    print get_announce_text()