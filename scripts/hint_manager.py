#! /usr/bin/env python

## @package exprob_3
#
# \file hint_manager.py
# \brief This node is used for managing the hints from the markers.
#
# \author Maria Luisa Aiachini
# \version 1.0
# \date 14/10/2022
# 
# \details
#
#  Service: <BR>
#	/hint_list
#
#  Client: <BR>
#	/oracle_hint	
#
#  Subscriber: <BR>
#	/IDs
#
#  Description: <BR>
#	This node is used to manage the hints received. It receives the IDs from the subscriver /IDs. It then asks for the corresponding via the
# /oracle_hint service. It checks if the hint is valid and lput it in the corresponding list.
#

import rospy
import time
from std_msgs.msg import Int32
from exprob_3.srv import Marker, MarkerResponse, Hints, HintsResponse

found_id = []
hint_0 = []
hint_1 = []
hint_2 = []
hint_3 = []
hint_4 = []
hint_5 = []
hint_service = None

##
# Callback for the /IDs subscriber. It takes the ID, checks if it is valid and then sends is to the /oracle_hint to retreive the corresponding hint.
# It checks if there are no errors in the hint received to make sure it is valid and finally append it in the corresponding list.
#
def ID_list_callback(id_received):
	global found_id, hint_service, hint_0, hint_1, hint_2, hint_3, hint_4, hint_5
	
	req = Marker()
	
	if id_received.data not in found_id and id_received.data <= 40 and id_received.data > 5:
		found_id.append(id_received.data)
		req = id_received.data
		
		hint = hint_service(req)
		ID = hint.oracle_hint.ID
		key = hint.oracle_hint.key
		value = hint.oracle_hint.value
		print("\nNew hint found:", "\nID: ", ID, "\nkey :", key, "\nvalue: ", value, "\n")
		
		if hint.oracle_hint.key == "" or hint.oracle_hint.key == "when":
			print("Key error\n")
		elif hint.oracle_hint.value == "" or hint.oracle_hint.value == "-1":
			print("Value error\n")
		else:
			print("Valid hint\n")
			new_hint = ""
			new_hint += key
			new_hint += ":"
			new_hint += value
			if ID == 0:
				hint_0.append(new_hint)
			elif ID == 1:
				hint_1.append(new_hint)
			elif ID == 2:
				hint_2.append(new_hint)
			elif ID == 3:
				hint_3.append(new_hint)
			elif ID == 4:
				hint_4.append(new_hint)
			elif ID == 5:
				hint_5.append(new_hint)

##
# Function to send the hint list when the /hint_list service is called
#
def send_hint_list(req):
	global hint_0, hint_1, hint_2, hint_3, hint_4, hint_5, hint_list_service
	resp = HintsResponse()
	resp.hint_0 = hint_0
	resp.hint_1 = hint_1
	resp.hint_2 = hint_2
	resp.hint_3 = hint_3
	resp.hint_4 = hint_4
	resp.hint_5 = hint_5

	return resp

##
# Main function for the "hint_manager" node. It initialises the node, the subscriber, the client and the server.
#
def main():
	global id_sub, hint_service, hint_list_service

	rospy.init_node('hint_manager')
	print("\nHINT MANAGER STARTED\n")

	id_sub = rospy.Subscriber("/IDs", Int32, ID_list_callback)
	hint_service = rospy.ServiceProxy('/oracle_hint', Marker)
	hint_list_service = rospy.Service('/hint_list', Hints, send_hint_list)
	rospy.spin()
	

if __name__ == '__main__':
    main()
