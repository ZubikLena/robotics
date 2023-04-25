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
global HitWall
global LeavePoint

HitWall=False
goal=(-2,0)
LeavePoint=goal

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
	3: 'follow_the_wall_to_reach_the_line'
}

def distance_to_point(point1,point2):
	distance=math.sqrt(((point1[0]-point2[0])**2)+((point1[1]-point2[1])**2))
	return distance

def on_line(x_pose,y_pose):
	
	tol = 0.2
	x_goal,y_goal=goal

	if (INIT_x-x_goal != 0):
		a = (INIT_y-y_goal)/(INIT_x-x_goal)
		b = INIT_y-(a*INIT_x)
	else:
		a=0
		b=INIT_y

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

	global d
	d = 0.35

	if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['right'] > d:
		state_description = 'case 1 - nothing'
		change_state(0)
	elif regions['front'] > d and regions['fleft'] > d and regions['fright'] > d and regions['right'] < d:
		state_description = 'case 1 - nothing'
		change_state(3)
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
		state_description = 'case 2 - front'
		change_state(1)
	elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
		state_description = 'case 3 - fright'
		change_state(2)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d and regions['right'] > d:
		state_description = 'case 4 - fleft'
		change_state(0)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d and regions['right'] < d:
		state_description = 'case 4 - fleft'
		change_state(3)
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
		state_description = 'case 5 - front and fright'
		change_state(1)
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
		state_description = 'case 6 - front and fleft'
		change_state(1)
	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
		state_description = 'case 7 - front and fleft and fright'
		change_state(1)
	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d and regions['right'] > d:
		state_description = 'case 8 - fleft and fright'
		change_state(1)
	elif regions['back'] < d and regions['fright'] > d and regions['bright'] > d:
		state_description = 'case 2 - front'
		change_state(3)
	elif regions['back'] < d and regions['fright'] > d and regions['bright'] < d:
		state_description = 'case 5 - front and fright'
		change_state(3)
	elif regions['back'] > d and regions['fright'] > d and regions['bright'] < d and regions['right'] < d:
		state_description = 'case 8 - fleft and fright'
		change_state(3)
	elif regions['back'] < d and regions['fright'] > d and regions['bright'] < d and regions['right'] < d:
		state_description = 'case 8 - fleft and fright'
		change_state(3)
	else:
		state_description = 'unknown case'
		rospy.loginfo(regions)


#ruch prostoliniowy do punktu docelowego
def go_to_point(x_pose, y_pose, theta_pose):
	tol=0.2

	x_goal,y_goal=goal

	angle=math.atan2(y_goal-y_pose,x_goal-x_pose)
	#print(angle-theta_pose)
	if(abs(angle-theta_pose)>tol):
		new_vel.angular.z = -0.2


	print(goal)

	distance=abs(math.sqrt(((x_goal-x_pose)**2)+((y_goal-y_pose)**2)))
	#print "Dystans: ", distance

	if(abs(angle-theta_pose)<=tol):
		new_vel.angular.z = 0.0
		new_vel.linear.x = 0.2
		
	if(abs(distance)<=tol):
		new_vel.linear.x = 0

def turn_right():
	new_vel.angular.z = 0.5

def turn_left():
	new_vel.angular.z = -0.3

def follow_the_wall():
	new_vel.linear.x = 0.1
	new_vel.angular.z = 0.005

def follow():
	print "tu"
	if state_==1:
		turn_left()
	elif state_==2:
		follow_the_wall()
	elif state_==3:
		turn_right()
	else:
		follow_the_wall()
		

# funkcja wywolywana przy przyjsciu danych ze skanera laserowego
def scan_callback(scan):
	global regions_

	regions_ = {
		'left' : min(min(scan.ranges[80:109]),10),
		'fleft' : min(min(scan.ranges[26:79]),10),
		'front' : min(min(scan.ranges[335:359]),min(scan.ranges[0:25]),10),
		'fright' : min(min(scan.ranges[280:334]),10),
		'right' : min(min(scan.ranges[250:279]),10),
		'bright' : min(min(scan.ranges[206:249]),10),
		'back' : min(min(scan.ranges[180:205]),10),
	}
	
	take_action()



# funkcja wywolywana przy przyjsciu danych o lokalizacji robota
def odom_callback(odom):
	global regions_
	regions = regions_
	global HitWall
	global LeavePoint
	pose = Pose()
	pose.x = odom.pose.pose.position.x
	pose.y = odom.pose.pose.position.y
	pose.theta = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w])[2]
	#print "Pozycja x: ",odom.pose.pose.position.x
	#print "Pozycja y: ",odom.pose.pose.position.y
	#print "Pozycja theta: ",pose.theta
	CurrentPoint=(pose.x,pose.y)
	tol=0.1

	if on_line(pose.x,pose.y):
		if HitWall:
			follow()	
		else:
			if state_==1:
				turn_left()
				HitWall=True
			elif state_==0 and distance_to_point(CurrentPoint,LeavePoint)>tol:
				go_to_point(pose.x,pose.y,pose.theta)
				HitWall=False
			elif state_==0 and distance_to_point(CurrentPoint,LeavePoint)<tol:
				follow_the_wall()
				HitWall=False
			else:
				turn_left()
				HitWall=False
				LeavePoint=CurrentPoint
	else:
		follow()
		HitWall=False

	print new_vel.linear.x, new_vel.angular.z
	print "state", state_
	print "ONLINE", on_line(pose.x,pose.y)
	print "HIT", HitWall
	print "LeavePoint", LeavePoint


if __name__== "__main__":
	global new_vel
	global goal
	global regions
	global d
	global HitWall
	global LeavePoint
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
