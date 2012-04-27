#!/usr/bin/env python

'''
Implements a ROS node that tests the overhead of rospy.loginfo(...)

Author: Chien-Liang Fok
Date: 02/12/2012
'''

import roslib; roslib.load_manifest('pharos_benchmarks')
import rospy, time
from threading import Thread

class LoopThread(Thread):
	def __init__(self, n):
		Thread.__init__(self, name=n) # Call superclass constructor
			
	def run(self):
		rospy.loginfo(rospy.get_name() + " " + self.getName() + ": Thread starting.")
		while not rospy.is_shutdown():
			rospy.loginfo(self.getName() + ": blah")
			time.sleep(0.1)  # cycle at 10Hz

if __name__ == '__main__':
	rospy.init_node('benchmark_loginfo')

	# Create and start a LoopThread thread
	lt1 = LoopThread("LoopThread 1")
	lt1.start()

	lt2 = LoopThread("LoopThread 2")
	lt2.start()

	rospy.spin()
