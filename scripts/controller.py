#! /usr/bin/env python

import rospy
import time
import random
import actionlib


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

#pub = None
pub = None
pos_x = 0
pos_y = 0
state = 0
targ = MoveBaseGoal()
finish = 0

def odom_clbk(position):
	global pos_x, pos_y, state
	pos_x = position.pose.pose.position.x
	pos_y = position.pose.pose.position.y
	

def next_room():
	global x, y, state
	print("\nIn next room\n")
	room = random.randint(0,5)
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
		y = -3
	elif room == 5:
		x = 5
		y = 1
	
	state = 1

def move():
	global pub2, targ, client, x, y, state
	
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
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		print("Target reached")
	state = 2
	
def search():
	global state, pub
	print("In search\n")
	vel = Twist()	
	vel.angular.z=0.4
	pub.publish(vel)
	sleep(10)
	state = 0
	
def main():
	global pub, client, state #, pub2

	rospy.init_node('controller')
	print("\nCONTROLLER NODE STARTED\n")

	#pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	#pub2 = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)	
	client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	client.wait_for_server()
	sub = rospy.Subscriber('/odom', Odometry, odom_clbk)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	print("state: ", state)
	
	while(finish == 0):
		if state == 0:
			print("Changing state to 0")
			next_room()
		elif state == 1:
			print("Changing state to 1")
			move()
		elif state == 2:
			print("Changing state to 2")
			search()
	
		
	rospy.spin()
	

if __name__ == '__main__':
    main()
