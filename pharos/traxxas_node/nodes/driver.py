#!/usr/bin/env python

'''
Implements a ROS node that interfaces with the Traxxas
Stampede mobility plane used by the Pharos Lab at UT
Austin.

Author: Chien-Liang Fok
Date: 02/08/2012
'''

import roslib; roslib.load_manifest('traxxas_node')
import rospy, serial, sys, binascii, time, struct
from std_msgs.msg import String
from threading import Thread
from traxxas_node.msg import AckermannDrive

''' Constant Defintions '''
PROTEUS_START = 0x24

def computeChecksum(data):
	#print "Computing checksum of " + str(data)
	checksum = 0
	for b in data:
		checksum ^= b
		#print "  after XORing " + str(b) + " checksum = " + str(checksum) + \
		#	" (0x" + binascii.b2a_hex(chr(checksum)) + ")"
	#rospy.loginfo(rospy.get_name() + " computeChecksum: Checksum of %s is %i", str(data), checksum)
	return checksum

'''
SerialMonitor operates as a separate thread that 
receives incoming data from the Traxxas.
'''
class SerialMonitor(Thread): # StatusReceiver extends Thread
	''' 
	The constructor of SerialMonitor.

	Parameters:
	 - ser: The serial port object
         - watchdog: A list object that when non-empty allows the
           SerialMonitor's loop to exit.
	'''
	def __init__(self, ser):
		Thread.__init__(self) # Call superclass constructor
		self.ser = ser;
		#self.running = True
		#self.watchdog = watchdog
	
	#def stop():
		#running = False
			
	def run(self):
		rospy.loginfo(rospy.get_name() + " Thread starting.")
		while not rospy.is_shutdown():
			while (ser.inWaiting() >= 12):
				startByte = ser.read(1)
				if (startByte == chr(PROTEUS_START)):
					#sys.stdout.write("Receiving 11 bytes...\n")
					rxdata = ser.read(11)
					
					# Format chars: http://docs.python.org/library/struct.html#format-characters
					respBytes = struct.unpack("<" + "B"*11, rxdata)
					#print "Received " + str(respBytes)
					checksum = PROTEUS_START
					for b in respBytes[:-1]:
						checksum ^= b
						#print "  after XORing " + str(b) + " checksum = " + str(checksum) + \	
						#	" (0x" + binascii.b2a_hex(chr(checksum)) + ")"

					resp = struct.unpack("<hhHhhB", rxdata)
					if (chr(checksum) == chr(resp[5])):
						rospy.loginfo(rospy.get_name() + " target speed = %i, current speed = %i, motor cmd = %i, prev err = %i, total err = %i",
							resp[0], resp[1], resp[2], resp[3], resp[4])

						#sys.stdout.write("target speed = " + str(resp[0]) \
						#	+ ", current speed = " + str(resp[1]) \
						#	+ ", motor cmd = " + str(resp[2]) \
						#	+ ", prev err = " + str(resp[3]) \
						#	+ ", total err = " + str(resp[4]) \
						#	+ "\n")
						#sys.stdout.write("Received " + str(struct.unpack("<hhHhh", rxdata)) + "\n\n")
						#sys.stdout.write("Received " + str(struct.unpack("b"*10, rxdata)) + "\n\n")
						#sys.stdout.flush()
					else:
						rospy.loginfo(rospy.get_name() + " Checksum mismatch 0x%x != 0x%x", \
							binascii.b2a_hex(chr(checksum)), binascii.b2a_hex(chr(resp[5])))
						#print "Checksum mismatch 0x" + binascii.b2a_hex(chr(checksum)) + " != 0x" \
						#	+ binascii.b2a_hex(chr(resp[5]))
				else:
					rospy.loginfo(rospy.get_name() + " Invalid start byte 0x%x != 0x%x", \
						binascii.b2a_hex(startByte), binascii.b2a_hex(chr(PROTEUS_START)))
					#print "Invalid start byte 0x" + binascii.b2a_hex(startByte) + " != 0x" + \
					#	binascii.b2a_hex(chr(PROTEUS_START)) + ", discarding"

'''
CommandSender periodically sends a move command to the Traxxas.
This is to prevent its safety stop from being triggered.
'''
class CommandSender(Thread): # CommandSender extends Thread
	
	''' 
	The constructor of CommandSender.

	Parameters:
	 - ser: The serial port object
	 - watchdog: A list object that when non-empty allows the
       SerialMonitor's loop to exit.
	'''
	def __init__(self, ser):
		Thread.__init__(self) # Call superclass constructor
		self.ser = ser;
		self.steering = 0
		self.speed = 0
		#self.running = True
		#self.watchdog = watchdog
	
	def driveCmdHandler(self, driveMsg):
		self.steering = int(driveMsg.steering_angle * 10)  # Convert to 1/10 degrees
		self.speed = int(driveMsg.speed * 100) # Convert to cm/s
		rospy.loginfo(rospy.get_name() + " New command: Steering=%i, speed=%i", \
			self.steering, self.speed)
	
	#def stop():
		#running = False

	def run(self):
		rospy.loginfo(rospy.get_name() + " Thread starting.")
		while not rospy.is_shutdown():
			data = [PROTEUS_START, self.steering, self.speed]
			data.append(computeChecksum(data))
			bytes = struct.pack("<BhhB", *data)
			numTX = ser.write(bytes)
			rospy.loginfo(rospy.get_name() + " CommandSender: Sent %s, num bytes: %i", str(data), numTX)
			time.sleep(0.1)  # sleep for 0.1 seconds to cycle at 10Hz



def getSteering(self):
	rospy.loginfo(rospy.get_name() + " getSteering(): %i", __steering)
	return self.__steering

def getSpeed(self):
	rospy.loginfo(rospy.get_name() + " getSpeed(): %i", __speed)
	return self.__speed

if __name__ == '__main__':
	port = rospy.get_param('/traxxas_node/port', "/dev/ttyUSB0")
	baud = rospy.get_param('/traxxas_node/baud', 115200)
	ser = serial.Serial(port, baud)

	if (ser):
        	rospy.loginfo("Serial port " + ser.portstr + " opened.")
	else:
		rospy.logerr("Unable to open serial port")
		sys.exit()

	rospy.loginfo("Waiting two seconds for Arduino to start...")
	time.sleep(2)

	ser.flushInput()

	#watchdog = []

	# Create and start a SerialMonitor thread
	smThread = SerialMonitor(ser)
	smThread.start()

	# Create and start the CommandSender thread
	cmdThread = CommandSender(ser)
	cmdThread.start()

	rospy.init_node('traxxas_driver')
	rospy.Subscriber("traxxas_node/ackermann_drive", AckermannDrive, cmdThread.driveCmdHandler)

	rospy.spin()

	# allow SerialMonitor and CommandSender threads to exit
	#smThread.stop() 
	#cmdThread.stop()
