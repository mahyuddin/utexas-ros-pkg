#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from traxxas_node.msg import AckermannDriveMsg
import sys, select, termios, tty

speed = .5
turn = 20

def moveForward(seconds):
	start = time.time()
	print start
	time.clock()
	elapsed = 0
	while elapsed < seconds:
		print"Moving Forward"
		elapsed = time.time() - start
		ackermannDriveMsg = AckermannDriveMsg()
		ackermannDriveMsg.speed = 1*speed
		ackermannDriveMsg.steering_angle = 0*turn 
		rospy.loginfo("Publishing speed = %f, steering = %f", ackermannDriveMsg.speed, ackermannDriveMsg.steering_angle)		
		pub.publish(ackermannDriveMsg)

def turnRight(seconds):
	start = time.time()
	time.clock()
	elapsed = 0
	while elapsed < seconds:
		print"Turning Right"
		elapsed = time.time() - start
		ackermannDriveMsg = AckermannDriveMsg()
		ackermannDriveMsg.speed = 1*speed
		ackermannDriveMsg.steering_angle = 1*turn 
		rospy.loginfo("Publishing speed = %f, steering = %f", ackermannDriveMsg.speed, ackermannDriveMsg.steering_angle)		
		pub.publish(ackermannDriveMsg)

def turnLeft(seconds):
	start = time.time()
	time.clock()
	elapsed = 0
	while elapsed < seconds:
		print"Turning Left"
		elapsed = time.time() - start
		ackermannDriveMsg = AckermannDriveMsg()
		ackermannDriveMsg.speed = 1*speed
		ackermannDriveMsg.steering_angle = -1*turn 
		rospy.loginfo("Publishing speed = %f, steering = %f", ackermannDriveMsg.speed, ackermannDriveMsg.steering_angle)		
		pub.publish(ackermannDriveMsg)



if __name__=="__main__":

   	settings = termios.tcgetattr(sys.stdin)

	
	pub = rospy.Publisher('/traxxas_node/ackermann_drive', AckermannDriveMsg)
	rospy.init_node('route_deployment')

	
	status = 0

	try:
			print"Begin Moving"
			moveForward(10)
			r = rospy.Rate(10)
			r.sleep()
			turnRight(3)
			moveForward(10)
			r.sleep()
			
	except:
		print e

	finally:
		ackermannDriveMsg = AckermannDriveMsg()
		#twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		#twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(AckermannDriveMsg)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


