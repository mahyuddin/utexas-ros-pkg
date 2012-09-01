#!/usr/bin/env python

import roslib; roslib.load_manifest('bwi_apps')
import rospy

from geometry_msgs.msg import Vector3

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speed by 10%
anything else : stop

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0),
		'o':(1,-1),
		'j':(0,1),
		'l':(0,-1),
		'u':(1,1),
		',':(-1,0),
		'.':(-1,-1),
		'm':(-1,1),
	       }

speedBindings={
		'q':(1.1),
		'z':(.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = .75

def vels(speed):
	return "currently:\tspeed %s " % (speed)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Vector3)
	rospy.init_node('object_controller')

	x = 0
	y = 0
	status = 0

	try:
		print msg
		print vels(speed)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key]

				print vels(speed)
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				x = 0
				y = 0
				if (key == '\x03'):
					break

			vector = Vector3()
			vector.x = x*speed; 
			vector.y = y*speed;
			vector.z = 0;
			pub.publish(vector)

	except:
		print "Unexpected Error"

	finally:
		vector = Vector3()
		vector.x = 0; 
		vector.y = 0;
		vector.z = 0;
		pub.publish(vector)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


