#! /usr/bin/env python

import rospy
import time
import random
import actionlib


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from std_msgs.msg import Int32
from exprob_3.srv import Hints, HintsRequest
from erl2.srv import Oracle, OracleRequest

#pub = None
pub = None
pos_x = 0
pos_y = 0
state = 0
visited = [0] * 6
actual_position = 100
targ = MoveBaseGoal()
finish = 0
found_id = []
hint_list_client = None
ids_to_check = []


def odom_clbk(position):
	global pos_x, pos_y, state
	pos_x = position.pose.pose.position.x
	pos_y = position.pose.pose.position.y
	

def next_room():
	global x, y, state, actual_position, visited, room
	print("\nIn next room\n")
	print("\nVisited: ", visited, "\n")
	if visited.count(1) == 6:
		print("All room visited, going home to check the hypotheses\n")
		state = 3
	else: 
	
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
			y = -4 #-3 però è troppo vicino a un muro
		elif room == 5:
			x = 5
			y = 1
		
		state = 1

def move():
	global pub2, targ, client, x, y, state, visited, room
	
	print("\nIn move\n")
	#room = random.randint(0,5)
	#vel = Twist()
	#vel.linear.x = 0.6
	#vel.linear.y = 0.6
	print("X: ", x, " Y: ", y)
	targ.target_pose.pose.position.x = x
	targ.target_pose.pose.position.y = y
	targ.target_pose.header.frame_id = "map"
	targ.target_pose.pose.orientation.w = 1
	client.send_goal(targ)
	#pub.publish(vel)
	#pub2.publish(targ)
	wait = client.wait_for_result() 
	if not wait:
		rospy.logerr("Action server not available!\n")
		rospy.signal_shutdown("Action server not available!\n")
	else:
		print("Target reached")
	state = 2
	
def search():
	global state, pub
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	print("In search\n")
	vel = Twist()	
	vel.angular.z = 0.7
	pub.publish(vel)
	time.sleep(40)
	#vel.angular.z = 0.0
	#pub.publish(vel)
	#print("Robot stopped")
	check_hypo()
	
def go_home():
	global state, targ, client, ids_to_check
	print("In going home\n")
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
		print("Target reached\n")
	check_winning()
	
def check_hypo():
	global hint_list_client, ids_to_check, state
	#TODO Controllare ipotesi vincente.
	print("\nIn check hypo\n")
	
	req = HintsRequest()
	hint_list = hint_list_client(req)
	
	#print("Hint list: ", hint_list)
	hint_0 = hint_list.hint_0
	hint_1 = hint_list.hint_1
	hint_2 = hint_list.hint_2
	hint_3 = hint_list.hint_3
	hint_4 = hint_list.hint_4
	hint_5 = hint_list.hint_5
	print("Hint list received:\n")
	print("ID0: ", hint_0, "\n")
	print("ID1: ", hint_1, "\n")
	print("ID2: ", hint_2, "\n")
	print("ID3: ", hint_3, "\n")
	print("ID4: ", hint_4, "\n")
	print("ID5: ", hint_5, "\n")
	
	if len(hint_0) == 3:
		pos = hint_0[0].find(":")
		key = hint_0[0]
		key = key[0:pos]
		hint = hint_0[0]
		hint = hint[pos+1:]
		print("\nkey: ", key, " hint: ", hint, "\n")
		if key == "who":
			who = hint
		elif key == "what":
			what = hint
		elif key == "where":
			where = hint
		
		pos2 = hint_0[1].find(":")
		key2 = hint_0[1]
		key2 = key2[0:pos2]
		hint2 = hint_0[1]
		hint2 = hint2[pos2+1:]
		print("\nkey: ", key2, " hint: ", hint2, "\n")
		if key == "who":
			who = hint2
		elif key == "what":
			what = hint2
		elif key == "where":
			where = hint2
		
		pos3 = hint_0[2].find(":")
		key3 = hint_0[2]
		key3 = key3[0:pos3]
		hint3 = hint_0[2]
		hint3 = hint3[pos3+1:]
		print("\nkey: ", key3, " hint: ", hint3, "\n")
		if key == "who":
			who = hint3
		elif key == "what":
			what = hint3
		elif key == "where":
			where = hint3
		
		if key != key2 and key2 != key3 and key != key3:
			print("ID0 has an hypothesis consistent: \n")
			print("It was ", who, " with the ", what, " in the " , where)
			print("\nGoing home to check if it is the winning one\n")
			ids_to_check.append(0)
			state = 3
	
	if len(hint_1) == 3:
		pos = hint_1[0].find(":")
		key = hint_1[0]
		key = key[0:pos]
		hint = hint_1[0]
		hint = hint[pos+1:]
		print("\nkey: ", key, " hint: ", hint, "\n")
		if key == "who":
			who = hint
		elif key == "what":
			what = hint
		elif key == "where":
			where = hint
		
		pos2 = hint_1[1].find(":")
		key2 = hint_1[1]
		key2 = key2[0:pos2]
		hint2 = hint_1[1]
		hint2 = hint2[pos2+1:]
		print("\nkey: ", key2, " hint: ", hint2, "\n")
		if key == "who":
			who = hint2
		elif key == "what":
			what = hint2
		elif key == "where":
			where = hint2
	
		pos3 = hint_1[2].find(":")
		key3 = hint_1[2]
		key3 = key3[0:pos3]
		hint3 = hint_1[2]
		hint3 = hint3[pos3+1:]
		print("\nkey: ", key3, " hint: ", hint3, "\n")
		if key == "who":
			who = hint3
		elif key == "what":
			what = hint3
		elif key == "where":
			where = hint3
			
		if key != key2 and key2 != key3 and key != key3:
			print("ID1 has an hypothesis consistent: \n")
			print("It was ", who, " with the ", what, " in the " , where)
			print("\nGoing home to check if it is the winning one\n")
			ids_to_check.append(1)
			state = 3
			
	if len(hint_2) == 3:
		pos = hint_2[0].find(":")
		key = hint_2[0]
		key = key[0:pos]
		hint = hint_2[0]
		hint = hint[pos+1:]
		print("\nkey: ", key, " hint: ", hint, "\n")
		if key == "who":
			who = hint
		elif key == "what":
			what = hint
		elif key == "where":
			where = hint
			
		pos2 = hint_2[1].find(":")
		key2 = hint_2[1]
		key2 = key2[0:pos2]
		hint2 = hint_2[1]
		hint2 = hint2[pos2+1:]
		print("\nkey: ", key2, " hint: ", hint2, "\n")
		if key == "who":
			who = hint2
		elif key == "what":
			what = hint2
		elif key == "where":
			where = hint2
			
		pos3 = hint_2[2].find(":")
		key3 = hint_2[2]
		key3 = key3[0:pos3]
		hint3 = hint_2[2]
		hint3 = hint3[pos3+1:]
		print("\nkey: ", key3, " hint: ", hint3, "\n")
		if key == "who":
			who = hint3
		elif key == "what":
			what = hint3
		elif key == "where":
			where = hint3
			
		if key != key2 and key2 != key3 and key != key3:
			print("ID2 has an hypothesis consistent: \n")
			print("It was ", who, " with the ", what, " in the " , where)
			print("\nGoing home to check if it is the winning one\n")
			ids_to_check.append(2)
			state = 3
						
	if len(hint_3) == 3:
		pos = hint_3[0].find(":")
		key = hint_3[0]
		key = key[0:pos]
		hint = hint_3[0]
		hint = hint[pos+1:]
		print("\nkey: ", key, " hint: ", hint, "\n")
		if key == "who":
			who = hint
		elif key == "what":
			what = hint
		elif key == "where":
			where = hint
		
		pos2 = hint_3[1].find(":")
		key2 = hint_3[1]
		key2 = key2[0:pos2]
		hint2 = hint_3[1]
		hint2 = hint2[pos2+1:]
		print("\nkey: ", key2, " hint: ", hint2, "\n")
		if key == "who":
			who = hint2
		elif key == "what":
			what = hint2
		elif key == "where":
			where = hint2
		
		pos3 = hint_3[2].find(":")
		key3 = hint_3[2]
		key3 = key3[0:pos3]
		hint3 = hint_3[2]
		hint3 = hint3[pos3+1:]
		print("\nkey: ", key3, " hint: ", hint3, "\n")
		if key == "who":
			who = hint3
		elif key == "what":
			what = hint3
		elif key == "where":
			where = hint3
			
		if key != key2 and key2 != key3 and key != key3:
			print("ID3 has an hypothesis consistent: \n")
			print("It was ", who, " with the ", what, " in the " , where)
			print("\nGoing home to check if it is the winning one\n")
			ids_to_check.append(3)
			state = 3
			
	if len(hint_4) == 3:
		pos = hint_4[0].find(":")
		key = hint_4[0]
		key = key[0:pos]
		hint = hint_4[0]
		hint = hint[pos:]
		print("\nkey: ", key, " hint: ", hint, "\n")
		if key == "who":
			who = hint
		elif key == "what":
			what = hint
		elif key == "where":
			where = hint
		
		pos2 = hint_4[1].find(":")
		key2 = hint_4[1]
		key2 = key2[0:pos2]
		hint2 = hint_4[1]
		hint2 = hint2[pos2+1:]
		print("\nkey: ", key2, " hint: ", hint2, "\n")
		if key == "who":
			who = hint2
		elif key == "what":
			what = hint2
		elif key == "where":
			where = hint2
		
		pos3 = hint_4[2].find(":");
		key3 = hint_4[2]
		key3 = key3[0:pos3]
		hint3 = hint_4[2]
		hint3 = hint3[pos3+1:]
		print("\nkey: ", key3, " hint: ", hint3, "\n")
		if key == "who":
			who = hint3
		elif key == "what":
			what = hint3
		elif key == "where":
			where = hint3
			
		if key != key2 and key2 != key3 and key != key3:
			print("ID4 has an hypothesis consistent: \n")
			print("It was ", who, " with the ", what, " in the " , where)
			print("\nGoing home to check if it is the winning one\n")
			ids_to_check.append(4)
			state = 3
			
	if len(hint_5) == 3:
		pos = hint_5[0].find(":")
		key = hint_5[0]
		key = key[0:pos]
		hint = hint_5[0]
		hint = hint[pos+1:]
		print("\nkey: ", key, " hint: ", hint, "\n")
		if key == "who":
			who = hint
		elif key == "what":
			what = hint
		elif key == "where":
			where = hint
		
		pos2 = hint_5[1].find(":")
		key2 = hint_5[1]
		key2 = key2[0:pos2]
		hint2 = hint_5[1]
		hint2 = hint2[pos2+1:]
		print("\nkey: ", key2, " hint: ", hint2, "\n")
		if key == "who":
			who = hint2
		elif key == "what":
			what = hint2
		elif key == "where":
			where = hint2
		
		pos3 = hint_5[2].find(":")
		key3 = hint_5[2]
		key3 = key3[0:pos3]
		hint3 = hint_5[2]
		hint3 = hint3[pos3+1:]
		print("\nkey: ", key3, " hint: ", hint3, "\n")
		if key == "who":
			who = hint3
		elif key == "what":
			what = hint3
		elif key == "where":
			where = hint3
			
		if key != key2 and key2 != key3 and key != key3:
			print("ID5 has an hypothesis consistent: \n")
			print("It was ", who, " with the ", what, " in the " , where)
			print("\nGoing home to check if it is the winning one\n")
			ids_to_check.append(5)
			state = 3
	print("Ids with complete and consistent hypothesis: ", ids_to_check)
	state = 0
	
def check_winning():
	global ids_to_check, state, visited
	print("In check winning\n")
	
	req = OracleRequest()
	win = winning(req)
	
	if win in ids_to_check:
		winning_id = ids_to_check[ids_to_check.index(win)]
		finish = 1
		print("Winning hypothesis found!\nIt was hypothesis: ID", winning_id, "\nI WON!")
		
	else:
		print("\nWinning hypothesis not found. Need to go back searching\n")
		visited = [0] * 6
		state = 0
	
#def ID_list_callback(id_received):
#	global found_id
#	if id_received not in found_id:
#		print("found id")
#		found_id.append(id_received)
#	print("ID found till now: ", found_id, "\n")
	#else:
	#	found_id.append(id_received)
	#print("ID found till now: ", found_id, "\n")
			
def main():
	global pub, client, state, hint_list_client#, id_sub #, pub2

	rospy.init_node('controller')
	print("\nCONTROLLER NODE STARTED\n")

	#pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	#pub2 = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)	
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	sub = rospy.Subscriber('/odom', Odometry, odom_clbk)
#	id_sub = rospy.Subscriber("/IDs", Int32, ID_list_callback)
	hint_list_client = rospy.ServiceProxy('/hint_list', Hints)
	winning = rospy.ServiceProxy('/oracle_solution', Oracle)
	
	print("state: ", state)
	
	#TODO serve service per /oracle_solution per controllare la vincente. Se vincente trovato si vince, se no si ricomincia a cercare
	
	while(finish == 0):
		if state == 0:
			print("Changing state to 0\n")
			next_room()
		elif state == 1:
			print("Changing state to 1\n")
			move()
		elif state == 2:
			print("Changing state to 2\n")
			search()
		elif state == 3:
			print("Going home\n")
			go_home()	
		
	rospy.spin()
	

if __name__ == '__main__':
    main()
