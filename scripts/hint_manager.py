#! /usr/bin/env python

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

def ID_list_callback(id_received):
	global found_id, hint_service, hint_0, hint_1, hint_2, hint_3, hint_4, hint_5
	
	req = Marker()
	
	if id_received.data not in found_id:
		found_id.append(id_received.data)
		print("found id: ", found_id)
		req = id_received.data
		#print("req: ", req, "\n")
		
		hint = hint_service(req)
		ID = hint.oracle_hint.ID
		key = hint.oracle_hint.key
		value = hint.oracle_hint.value
		print("\nHINT:", "\nID: ", ID, "\nkey :", key, "\nvalue: ", value, "\n")
		
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
			print("New_hint: ", new_hint, "\n")
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
			#print("Hints by now:\n")
			#print("ID0: ", hint_0, "\n")
			#print("ID1: ", hint_1, "\n")
			#print("ID2: ", hint_2, "\n")
			#print("ID3: ", hint_3, "\n")
			#print("ID4: ", hint_4, "\n")
			#print("ID5: ", hint_5, "\n")

def send_hint_list(req):
	global hint_0, hint_1, hint_2, hint_3, hint_4, hint_5, hint_list_service
	print("Sending the hint list")
	resp = HintsResponse()
	resp.hint_0 = hint_0
	resp.hint_1 = hint_1
	resp.hint_2 = hint_2
	resp.hint_3 = hint_3
	resp.hint_4 = hint_4
	resp.hint_5 = hint_5

	return resp


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
