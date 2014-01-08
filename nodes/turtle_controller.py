#!/usr/bin/env python  
import roslib
roslib.load_manifest('turtle_kinect')
import rospy
import math
import tf
import turtlesim.msg
import turtlesim.srv
import std_srvs.srv
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point

if __name__ == '__main__':
	#Define frame need to transform
	fixed_frame = '/openni_depth_frame'
	right_hand = '/left_hand'
	left_hand = '/right_hand'

	#Initialize first position of right and left hand
	firstRX = 100
	firstRY = 100
	firstRZ = 100
	firstLX = 100

	#Constants range
	depth_range_follow = 0.1
	#Clear should be less sentitive
	depth_range_clear = 0.2
	move_range = 0.1
	rotation_range = 0.1


	rospy.init_node('turtle_controller')

	listener = tf.TransformListener()

	clear_screen = rospy.ServiceProxy('clear', std_srvs.srv.Empty)

	#Spawn the following turtle
	rospy.wait_for_service('spawn')
	spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
	spawner(4, 2, 0, 'turtle2')

	turtle_vel = rospy.Publisher('turtle1/cmd_vel', Twist)
	spawn_turtle_vel = rospy.Publisher('turtle2/cmd_vel', Twist)

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		try:
			(right_hand_trans,rot) = listener.lookupTransform(fixed_frame, '/left_hand_1', rospy.Time(0))
			(left_hand_trans,rot) = listener.lookupTransform(fixed_frame, '/right_hand_1' , rospy.Time(0))

			if (firstRX == 100):
				firstRX = right_hand_trans[0]
				firstRY = right_hand_trans[1]
				firstRZ = right_hand_trans[2]
				firstLX = left_hand_trans[0]

			if right_hand_trans[0] < firstRX - depth_range_clear:
				#Clean
				clear_screen()
			elif right_hand_trans[2] > firstRZ + move_range:
				#Up
				turtle_vel.publish(Twist(Vector3(2.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
			elif right_hand_trans[2] < firstRZ - move_range:
				#Down
				turtle_vel.publish(Twist(Vector3(-2.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
			elif right_hand_trans[1] > firstRY + rotation_range:
				#Rotation Left
				turtle_vel.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 2.0)))
			elif right_hand_trans[1] < firstRY - rotation_range:
				#Rotation Right
				turtle_vel.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, -2.0)))
			else:
				#Stop
				turtle_vel.publish(Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))

			#Get transform from turtle1
			(trans,rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))

			#Determine whether turtle2 follows turtle1
			if left_hand_trans[0] > firstLX - depth_range_follow:
				angular = 4 * math.atan2(trans[1], trans[0])
				linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
				#Make turtle2 follow turtle1
				spawn_turtle_vel.publish(Twist(Vector3(linear, 0.0, 0.0), Vector3(0.0, 0.0, angular)))

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass

		rate.sleep()