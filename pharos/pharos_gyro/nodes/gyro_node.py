#!/usr/bin/env python
import roslib; roslib.load_manifest('lightswitch_node')
import rospy
from threading import Thread
from std_msgs.msg import Float32
import serial, time, sys, numpy

'''
SerialMonitor operates as a separate thread that receives incoming light level
data from the Arduino Mega. This thread continuously polls for new serial
messages and transmits them whenever available
'''
class SerialMonitor(Thread): # SerialMonitor extends Thread
  ''' 
  The constructor of SerialMonitor.

  Parameters:
   - ser: The serial port object
   - pub: The object through which LightLevel messages may be published
  '''
  def __init__(self, ser, pub, gyro_scale_factor):
    Thread.__init__(self, name="SerialMonitor") # Call superclass constructor
    self.ser = ser;
    self.gyro_scale_factor = gyro_scale_factor

  def run(self):
    rospy.loginfo(rospy.get_name() + " SerialMonitor: Thread starting.")

    count = 0
    initial_values = []
    gyro_stand_value = 0;

    while not rospy.is_shutdown():
      gyro_value = ser.readline()
      gyro_value = float(gyro_value)
      
      count += 1
      if count < 5:
        initial_values.append(gyro_value)
      elif count == 5:
        gyro_stand_value = numpy.mean(initial_values)
        rospy.loginfo(rospy.get_name() + " SerialMonitor: Setting default gyro value as " + str(gyro_stand_value))
      
      if count >= 5:
        msg = Float32()
        msg.data = self.gyro_scale_factor * (gyro_value - gyro_stand_value)
        pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('gyro_node')

    port = rospy.get_param('~port', "/dev/ttyUSB0")
    baud = rospy.get_param('~baud', 9600)
    gyro_scale_factor = rospy.get_param('~gyro_scale_factor', 1.0)
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
    pub = rospy.Publisher('gyro', Float32)
    smThread = SerialMonitor(ser, pub, gyro_scale_factor)
    smThread.start()

    rospy.loginfo("Starting ROS spinner...")
    rospy.spin()
