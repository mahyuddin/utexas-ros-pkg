#!/usr/bin/env python
import roslib; roslib.load_manifest('beginner_tutorials')
import rospy
from std_msgs.msg import String

def driveCmdHandler(data):
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

def cmdRcvr():
    rospy.init_node('driver')
    rospy.Subscriber("ackermann_drive", AckermannDrive, driveCmdHandler)
    rospy.spin()

if __name__ == '__main__':
    cmdRcvr()
