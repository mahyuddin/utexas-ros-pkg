#!/usr/bin/env python
import roslib; roslib.load_manifest('lightswitch_node')
import rospy
from lightswitch_node.msg import LightSwitchCmd
import serial, time, struct

def callback(cmd):
    rospy.loginfo(rospy.get_name() + " - Received command %i", cmd.cmd)
    data = [cmd.cmd]
    # Format chars: http://docs.python.org/library/struct.html#format-characters
    bytes = struct.pack("<b", *data)
    numTX = ser.write(bytes)
    rospy.loginfo(rospy.get_name() + " - Sent command (%i bytes)", numTX)
			
if __name__ == '__main__':
    rospy.init_node('lightswitch_node')

    port = rospy.get_param('/lightswitch_node/port', "/dev/ttyACM0")
    baud = rospy.get_param('/lightswitch_node/baud', 115200)    
    rospy.loginfo("Serial port = %s", port)
    rospy.loginfo("Serial baud = %i", baud)

    ser = serial.Serial(port, baud)

    if (ser):
        rospy.loginfo("Serial port " + ser.portstr + " opened.")
    else:
        rospy.logerr("Unable to open serial port")
        sys.exit()

    rospy.loginfo("Waiting 2s for Arduino to initialize...")
    time.sleep(2)

    ser.flushInput()
    rospy.Subscriber("lightswitch_cmd", LightSwitchCmd, callback)
    rospy.loginfo("Subscribed to \"lightswitch_cmd\" topic...")
    rospy.spin()
