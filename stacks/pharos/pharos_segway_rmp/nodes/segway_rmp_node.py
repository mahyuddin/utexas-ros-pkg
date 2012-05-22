#!/usr/bin/env python

import roslib; roslib.load_manifest('turtlebot_node')

"""
This node is based on a combination of William Woodall's C++
segway_rmp stack and the ROS Turtlebot code to produce a 
serial driver for the Segway RMP in python using pySerial.

The goal of this script is to provide a simple single threaded
driver for the Segway RMP

This driver is initially targetted at the RMP 50. 
"""

import os
import time
import serial
import struct

import rospy

from math import radians
from geometry_msgs.msg import Pose2D

from pharos_segway_rmp.msg import SegwayRMPSensorState

_struct_I = roslib.message.struct_I
_struct_BI = struct.Struct(">BI")
_struct_12B2hBHhb7HBH5B4h = struct.Struct(">12B2hBHhb7HBH5B4h")

def deserialize(msg, buff, timestamp):
    """
    unpack serialized message in str into this message instance
    @param buff: byte array of serialized message
    @type  buff: str
    """
    try:
        _x = msg
        (_x.bumps_wheeldrops, _x.wall, _x.cliff_left, _x.cliff_front_left, _x.cliff_front_right, _x.cliff_right, _x.virtual_wall, _x.motor_overcurrents, _x.dirt_detector_left, _x.dirt_detector_right, _x.remote_opcode, _x.buttons, _x.distance, _x.angle, _x.charging_state, _x.voltage, _x.current, _x.temperature, _x.charge, _x.capacity, _x.wall_signal, _x.cliff_left_signal, _x.cliff_front_left_signal, _x.cliff_front_right_signal, _x.cliff_right_signal, _x.user_digital_inputs, _x.user_analog_input, _x.charging_sources_available, _x.oi_mode, _x.song_number, _x.song_playing, _x.number_of_stream_packets, _x.requested_velocity, _x.requested_radius, _x.requested_right_velocity, _x.requested_left_velocity,) = _struct_12B2hBHhb7HBH5B4h.unpack(buff[0:52])

        msg.wall = bool(msg.wall)
        msg.cliff_left = bool(msg.cliff_left)
        msg.cliff_front_left = bool(msg.cliff_front_left)
        msg.cliff_front_right = bool(msg.cliff_front_right)
        msg.cliff_right = bool(msg.cliff_right)
        msg.virtual_wall = bool(msg.virtual_wall)
        msg.song_playing = bool(msg.song_playing)

        # do unit conversions
        msg.angle = radians(msg.angle)
        msg.header.stamp = rospy.Time.from_seconds(timestamp)        
        msg.distance     = float(msg.distance) / 1000.
        
        msg.requested_velocity       = float(msg.requested_velocity) / 1000.
        msg.requested_radius         = float(msg.requested_radius) / 1000.
        msg.requested_right_velocity = float(msg.requested_right_velocity) / 1000.
        msg.requested_left_velocity  = float(msg.requested_left_velocity) / 1000.

        return msg
    except struct.error, e:
      raise roslib.message.DeserializationError(e)

class SegwayRMPSensorHandler(object):
    
    def __init__(self, robot):
        self.robot = robot    

    def request_packet(self, packet_id):
        """Reqeust a sensor packet."""
        with self.robot.si.lock:
            self.robot.si.flush_input()
            self.robot.si.sensors(packet_id)
            #kwc: there appears to be a 10-20ms latency between sending the
            #sensor request and fully reading the packet.  Based on
            #observation, we are preferring the 'before' stamp rather than
            #after.
            stamp = time.time()
            length = SENSOR_GROUP_PACKET_LENGTHS[packet_id]
            return self.robot.si.read(length), stamp

    def get_all(self, sensor_state):
        buff, timestamp = self.request_packet(6)
        if buff is not None:
            deserialize(sensor_state, buff, timestamp)

class SegwayRMP(object):

  def __init__(self, segway_rmp_type='50/100'):

    if segway_rmp_type == '50/100':
      self.dps_to_counts = 7.8
      self.mps_to_counts = 401.0
      self.meters_to_counts = 40181.0
      self.rev_to_counts = 117031.0
      self.torque_to_counts = 1463.0
    else:
      self.dps_to_counts = 7.8;
      self.mps_to_counts = 332.0;
      self.meters_to_counts = 33215.0;
      self.rev_to_counts = 112644.0;
      self.torque_to_counts = 1094.0;

class SegwayRMPNode(object):

  def __init__(self, default_port='/dev/ttyUSB0', default_update_rate=30.0):

    """
    @param default_port: default tty port to use for establishing
        connection to Turtlebot.  This will be overriden by ~port ROS
        param if available.
    """
    self.default_port = default_port
    self.default_update_rate = default_update_rate

    rospy.init_node('segway_rmp_node')
    self._init_params()
    self._init_pubsub()

    self.robot = SegwayRMP()
    self.sensor_state = SegwayRMPSensorState()
    self.req_cmd_vel = None

    self._pos2d = Pose2D()

  def start(self):
    
    # Start serial connection (Retry every 5 seconds if unable to connect)
    log_once = True
    while not rospy.is_shutdown():
      try:
        self.robot.start(self.port, self.baudrate)
      except serial.serialutil.SerialException as ex:
        msg = "Failed to open port %s.  Please make sure the Create cable is plugged into the computer. \n"%(self.port)
        if log_once:
          log_once = False
          rospy.logerr(msg)
        time.sleep(5.0)



def connected_file():
  return os.path.join(roslib.rosenv.get_ros_home(), 'segway-rmp-connected')

def segway_rmp_main(argv):
  c = SegwayRMPNode()
  while not rospy.is_shutdown():
    try:
      # This sleep throttles reconnecting of the driver.  It
      # appears that pyserial does not properly release the file
      # descriptor for the USB port in the event that the Create is
      # unplugged from the laptop.  This file desecriptor prevents
      # the create from reassociating with the same USB port when it
      # is plugged back in.  The solution, for now, is to quickly
      # exit the driver and let roslaunch respawn the driver until
      # reconnection occurs.  However, it order to not do bad things
      # to the Create bootloader, and also to keep relaunching at a
      # minimum, we have a 5-second sleep.
      time.sleep(5.0)

      c.start()
      c.spin()

    except Exception as ex:
      msg = "Failed to contact device with error: [%s]. Please check that the Segway is powered on and that the connector is plugged into."%(ex)
      rospy.logerr(msg)

    finally:
      # Driver no longer connected, delete flag from disk
          try:
            os.remove(connected_file())
          except Exception: pass

if __name__ == '__main__':
  segway_rmp_main(sys.argv)
