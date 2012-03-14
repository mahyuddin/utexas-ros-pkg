#!/usr/bin/env python
import roslib; roslib.load_manifest('lightswitch_node')
import rospy
from lightswitch_node.msg import LightSwitchCmd
from lightswitch_node.msg import AmbientLight
import serial, time, struct
from threading import Thread

'''
SerialMonitor operates as a separate thread that 
receives incoming light level data from the Arduino Mega.  Since
light level messages are transmitted at 10Hz, this
thread loops at 20Hz.
'''
class SerialMonitor(Thread): # SerialMonitor extends Thread
  ''' 
  The constructor of SerialMonitor.

  Parameters:
   - ser: The serial port object
   - pub: The object through which LightLevel messages may be published
  '''
  def __init__(self, ser, pub):
    Thread.__init__(self, name="SerialMonitor") # Call superclass constructor
    self.ser = ser; 
    self.seqno = 0;

  def run(self):
    rospy.loginfo(rospy.get_name() + " SerialMonitor: Thread starting.")
    while not rospy.is_shutdown():
      if (ser.inWaiting() > 0):
        lightLevel = ser.read(1)
	lightLevel = struct.unpack("B", lightLevel) # convert from string to tuple (an immutable sequence type)
	rospy.logdebug(rospy.get_name() + " SerialMonitor: light level = %i", lightLevel[0])
        
        # Create an AmbientLight message
        msg = AmbientLight()
        msg.header.seq = self.seqno
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "light_sensor"
        msg.lightLevel = lightLevel[0]

        self.seqno += 1
        
        # Publish the AmbientLight message
        pub.publish(msg)
      time.sleep(0.05)  # cycle at 20Hz

def callback(cmd):
    rospy.loginfo(rospy.get_name() + " - Received light switch command %i", cmd.cmd)
    data = [cmd.cmd]
    # Format chars: http://docs.python.org/library/struct.html#format-characters
    bytes = struct.pack("<b", *data)
    numTX = ser.write(bytes)
    #rospy.loginfo(rospy.get_name() + " - Sent command (%i bytes)", numTX)
			
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

    # Create and start a serial monitor thread.
    # This is for receiving light level information.
    pub = rospy.Publisher('lightLevel', AmbientLight)
    smThread = SerialMonitor(ser, pub)
    smThread.start()

    rospy.Subscriber("lightswitch_cmd", LightSwitchCmd, callback)
    rospy.loginfo("Subscribed to \"lightswitch_cmd\" topic...")
    rospy.spin()
