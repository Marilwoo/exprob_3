#! /usr/bin/env python

## @package exprob_3
#
# \file controller.py
# \brief This node is used for controlling the robot's behavior
#
# \author Maria Luisa Aiachini
# \version 1.0
# \date 14/10/2022
# 
# \details
#
#  Client: <BR>
#	/oracle_solution	
#
#	/hint_list
#
#  Action client: <BR>
#	/move_base
#
#  Publisher: <BR>
#	/cmd_vel
#
#  Description: <BR>
#	This node is used to control the behavior of the robot through a state machine. The state machine implements 6 states: 
#chosing randomly the next room to visit, moving towards the room, turning on itself to search for hints, moving towards home, check the hypotheses for completeness and consistency and checking hypotheses for the winning one.
#

import rospy
import time
import random
import actionlib

from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from std_msgs.msg import Int32
from exprob_3.srv import Hints, HintsRequest
from erl2.srv import Oracle, OracleRequest

pub = None
state = 0
visited = [0] * 6
actual_position = 100
targ = MoveBaseGoal()
finish = 0
found_id = []
hint_list_client = None
winning = None
ids_to_check = []

##
# Function for randomly choose the next room to visit. It also associate the room number to the x and y coordinates.
#
def next_room():
	global x, y, state, actual_position, visited, room
	if visited.count(1) == 6:
		visited = [0] * 6
		state = 0
	else: 
		print("Selecting which room to visit\n")
		room = random.randint(0,5)
		ready = 0
		while ready == 0:
			if visited[room] == 1 or actual_position == room:
				room = random.randint(0,5)
			else:
				ready = 1
			
		visited[room] = 1
		actual_position = room
		
		if room == 0:
			x = -4
			y = -3
		elif room == 1:
			x = -4
			y = 2	
		elif room == 2:
			x = -4
			y = 7
		elif room == 3:
			x = 5
			y = -7
		elif room == 4:
			x = 5
			y = -4
		elif room == 5:
			x = 5
			y = 1
		
		state = 1

##
# Function to make the robot move, it uses move_base action server to give the goal position. It also waits for the result before going
# to the next state.
#
def move():
	global pub2, targ, client, x, y, state, visited, room
	
	print("Going to position: (", x, ",", y, ")")
	targ.target_pose.pose.position.x = x
	targ.target_pose.pose.position.y = y
	targ.target_pose.header.frame_id = "map"
	targ.target_pose.pose.orientation.w = 1
	client.send_goal(targ)
	wait = client.wait_for_result() 
	if not wait:
		rospy.logerr("Action server not available!\n")
		rospy.signal_shutdown("Action server not available!\n")
	else:
		print("Target reached")
	state = 2

##
# Function to make the robot turn on itself to search for surrounding hints. It publishes on /cmd_vel topic.
#
def search():
	global state, pub
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	print("\nLooking around for hints\n")
	vel = Twist()	
	vel.angular.z = 0.35
	pub.publish(vel)
	time.sleep(45)
	state = 4
	
##
# Function to make the robot go towards home. It uses move_base action server and waits for the results before moving to the
# next state.
#
def go_home():
	global state, targ, client, ids_to_check
	print("Moving towards home position\n")
	targ.target_pose.pose.position.x = 0.0
	targ.target_pose.pose.position.y = -1.0
	targ.target_pose.header.frame_id = "map"
	targ.target_pose.pose.orientation.w = 1
	client.send_goal(targ)
	wait = client.wait_for_result() 
	if not wait:
		rospy.logerr("Action server not available!\n")
		rospy.signal_shutdown("Action server not available!\n")
	else:
		print("Home position reached\n")
	state = 5
	
##
# Function to check if one of the hypotheses received are complete and/or consistent.
# It uses the service /hint_list to ask for the hypotheses. It then implements a for cycle to check them all.
# If one hypotheses it is found complete and consistent it is appended to the ids_to_check list to check if it is the winning one.
# This function also uses the key associated to every hint to save the "who", "where", "what" parts of the hint to print it in natural language.
#
def check_hypo():
	global hint_list_client, ids_to_check, state
	print("\nChecking if any of the new hypotheses are complete and consistent\n")
	who = ""
	what = ""
	where = ""
	
	req = HintsRequest()
	hint_list = hint_list_client(req)
	
	hint_0 = hint_list.hint_0
	hint_1 = hint_list.hint_1
	hint_2 = hint_list.hint_2
	hint_3 = hint_list.hint_3
	hint_4 = hint_list.hint_4
	hint_5 = hint_list.hint_5
	
	to_check = []
	for i in range(0,6):
		if i == 0:
			to_check = hint_0
		elif i == 1:
			to_check = hint_1
		elif i == 2:
			to_check = hint_2
		elif i == 3:
			to_check = hint_3
		elif i == 4:
			to_check = hint_4
		elif i == 5:
			to_check = hint_5
		print("ID", i, ": hints received by now: ", to_check)
		if len(to_check) == 3:
			pos = to_check[0].find(":")
			key = to_check[0]
			key = key[0:pos]
			hint = to_check[0]
			hint = hint[pos+1:]
			if key == "who":
				who = hint
			elif key == "what":
				what = hint
			elif key == "where":
				where = hint
			
			pos2 = to_check[1].find(":")
			key2 = to_check[1]
			key2 = key2[0:pos2]
			hint2 = to_check[1]
			hint2 = hint2[pos2+1:]
			if key2 == "who":
				who = hint2
			elif key2 == "what":
				what = hint2
			elif key2 == "where":
				where = hint2
			
			pos3 = to_check[2].find(":")
			key3 = to_check[2]
			key3 = key3[0:pos3]
			hint3 = to_check[2]
			hint3 = hint3[pos3+1:]
			
			if key3 == "who":
				who = hint3
			elif key3 == "what":
				what = hint3
			elif key3 == "where":
				where = hint3
			if i not in ids_to_check:
				if key != key2 and key2 != key3 and key != key3:
					print("ID",i," has an hypothesis consistent: \nIt was ", who, " with the ", what, " in the " , where, "\n")
					time.sleep(1)
					ids_to_check.append(i)
					state = 3
	if ids_to_check == []:
		print("I haven't any new complete and consistent hypothesis yet\n")	
	if state != 3:
		state = 0
	
	
##
# Function to check if one of the complete and consistent hypotheses found is the winning one.
# It uses the /oracle_solution service to ask for the winning ID and compares it with the list of suitable hypotheses.
#
def check_winning():
	global ids_to_check, state, visited, winning, finish
	print("\nChecking if the winning hypothesis has been found\n")
	
	req = OracleRequest()
	win = winning(req)
	if win.ID in ids_to_check:
		winning_id = ids_to_check[ids_to_check.index(win.ID)]
		print("Winning hypothesis found!\nIt was hypothesis: ID", winning_id, "\nI WON!")
		finish = 1
		
	else:
		print("\nWinning hypothesis not found. Need to go back searching\n")
		state = 0
	
##
# Main function for the "controller" node. Here are initialized the clients and action client, as well as implementing the state machine.
#	
def main():
	global pub, client, state, hint_list_client, winning, finish #, id_sub #, pub2

	rospy.init_node('controller')
	print("\nCONTROLLER NODE STARTED\n")

	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	hint_list_client = rospy.ServiceProxy('/hint_list', Hints)
	winning = rospy.ServiceProxy('/oracle_solution', Oracle)
	
	print("state: ", state)
	
	while(finish == 0):
		if state == 0:
			next_room()
		elif state == 1:
			move()
		elif state == 2:
			search()
		elif state == 3:
			go_home()	
		elif state == 4:
			check_hypo()
		elif state == 5:
			check_winning()
		
	rospy.spin()
	

if __name__ == '__main__':
    main()
