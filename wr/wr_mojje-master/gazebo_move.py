#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg  import Pose
from sensor_msgs.msg import LaserScan
import math

INIT_x,INIT_y=0.5,0.5
global new_vel
global goal
global regions
goal=(5,1)


regions_ = {
	'right': 0,
	'fright': 0,
	'front': 0,
	'fleft': 0,
	'left': 0,
}
state_ = 0
state_dict_ = {
	0: 'straight to point',
	1: 'turn left',
	2: 'follow the wall',
}

#funkcja sprawdzajaca czy robot znajdyje sie na linii laczacej punkt startowy z docelowym
def on_line(x_pose, y_pose):
	tol = 0.1
	x_goal,y_goal=goal

	a = (y_goal-y_pose)/(x_goal-x_pose)
	b = INIT_y-a*INIT_x

	if (abs(y_pose-(a*x_pose+b))<=tol):
		return True
	else:
		return False

def change_state(state):
	global state_, state_dict_
	if state is not state_:
		print 'Wall followe - [%s] - %s' % (state, state_dict_[state])
		state_ = state

def take_action():
	global regions_
	regions = regions_
	
	new_vel.linear.x = 0
	new_vel.angular.z = 0

	state_description = ""

	d = 0.5

	if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
		state_description = 'case 1 - nothing'
		change_state(0)
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
		state_description = 'case 2 - front'
		change_state(1)
		state_=1
	elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
		state_description = 'case 3 - fright'
		change_state(2)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
		state_description = 'case 4 - fleft'
		change_state(0)
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
		state_description = 'case 5 - front and fright'
		change_state(1)
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
		state_description = 'case 6 - front and fleft'
		change_state(1)
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
		state_description = 'case 7 - front and fleft and fright'
		change_state(1)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
		state_description = 'case 8 - fleft and fright'
		change_state(0)
	else:
		state_description = 'unknown case'
		rospy.loginfo(regions)


#ruch prostoliniowy do punktu docelowego
def go_to_point(x_pose, y_pose, theta_pose):
	tol=0.1

	x_goal,y_goal=goal

	if(x_goal>10 or y_goal>10 or x_goal<0 or y_goal<0):
		print("Punkt nieosiagalny")
		new_vel.angular.z=0
		new_vel.linear.x=0

	#((angle-pose.theta+math.pi)%(math.pi*2)-math.pi)*factor_ang
	angle=math.atan2(y_goal-y_pose,x_goal-x_pose)
	print(angle-theta_pose)
	if(abs(angle-theta_pose)>tol and angle-theta_pose>0):
		new_vel.angular.z = -0.2
	else:
		new_vel.angular.z = 0.2

	print(goal)

	distance=abs(math.sqrt(((x_goal-x_pose)**2)+((y_goal-y_pose)**2)))
	print "Dystans: ", distance

	if(abs(angle-theta_pose)<=tol):
		new_vel.angular.z = 0.0
		new_vel.linear.x = 0.2
		
	if(abs(distance)<=tol):
		new_vel.linear.x = 0

	print(new_vel.angular.z,new_vel.linear.x)


def turn_left():
	new_vel.angular.z = 0.3

def follow_the_wall():
	global regions_

	new_vel.linear.x = 0.5


# funkcja wywolywana przy przyjsciu danych ze skanera laserowego
def scan_callback(scan):
	#print scan
	global regions_

	regions_ = {
		'left' : min(scan.ranges[89],10),
		'fleft' : min(min(scan.ranges[36:88]),10),
		'front' : min(min(scan.ranges[324:359]),min(scan.ranges[0:35]),10),
		'fright' : min(min(scan.ranges[270:323]),10),
		'right' : min(scan.ranges[269],10),
	}
	
	print regions_
	take_action()



# funkcja wywolywana przy przyjsciu danych o lokalizacji robota
def odom_callback(odom):
	global regions_
	regions = regions_

	pose = Pose()
	pose.x = odom.pose.pose.position.x
	pose.y = odom.pose.pose.position.y
	pose.theta = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])[2]
	print "Pozycja x: ",odom.pose.pose.position.x
	print "Pozycja y: ",odom.pose.pose.position.y
	print "Pozycja theta: ",pose.theta
	

	go_to_point(pose.x,pose.y,pose.theta)
	"""
	if state_ == 0:
		go_to_point(pose.x,pose.y,pose.theta)
		print("state0")
	elif state_ == 1:
		turn_left()
		print("state1")
	elif state_ == 2:
		follow_the_wall()
		print("state2")
	else:
		rospy.logerr('Unknown state!')"""
		


if __name__== "__main__":
	global new_vel
	global goal
	global regions
	new_vel = Twist()
	rospy.init_node('wr_zad', anonymous=True)
	print("ready")

	rospy.Subscriber( '/odom' , Odometry, odom_callback)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	rospy.Subscriber( '/scan' , LaserScan, scan_callback)
	
	rate=rospy.Rate(10) # 10Hz
	while not rospy.is_shutdown():
		pub.publish(new_vel)#wyslanie predkosci zadanej
		rate.sleep()

	print("END")
