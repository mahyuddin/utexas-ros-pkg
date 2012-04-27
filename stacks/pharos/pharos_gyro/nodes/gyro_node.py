#!/usr/bin/env python
import roslib; roslib.load_manifest('pharos_gyro')
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
  def __init__(self, ser, pub, gyro_scale_correction):
    Thread.__init__(self, name="SerialMonitor") # Call superclass constructor
    self.ser = ser;
    self.gyro_scale_correction = gyro_scale_correction

  def run(self):
    rospy.loginfo("Starting SerialMonitor thread.")

    count = 0
    initial_values = []
    stationary_gyro_value = 0;

    while not rospy.is_shutdown():
      gyro_value = ser.readline()
      gyro_value = float(gyro_value)
      
      count += 1
      if count < 5:
        initial_values.append(gyro_value)
      elif count == 5:
        stationary_gyro_value = numpy.mean(initial_values)
        rospy.loginfo("Stationary gyro value selected as %f", stationary_gyro_value)
      
      if count >= 5:
        msg = Float32()
        msg.data = self.gyro_scale_correction * (gyro_value - stationary_gyro_value)
        pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('gyro_node')

    port = rospy.get_param('~serial_port', "/dev/ttyUSB0")
    baud = rospy.get_param('~serial_baud', 9600)
    gyro_scale_correction = rospy.get_param('~gyro_scale_correction', 1.0)
    rospy.loginfo("Serial port = %s", port)
    rospy.loginfo("Serial baud = %i", baud)
    rospy.loginfo("Gyro Scale Correction = %f", gyro_scale_correction)

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
    smThread = SerialMonitor(ser, pub, gyro_scale_correction)
    smThread.start()

    rospy.loginfo("Starting ROS spinner...")
    rospy.spin()
