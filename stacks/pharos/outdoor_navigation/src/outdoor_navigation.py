#!/usr/bin/env python

import sys
import math

import rospy
from std_msgs.msg import String
from traxxas_node.msg import AckermannDriveMsg
from proteus3_gps_hydro.msg import GPSMsg
from proteus3_compass_hydro.msg import CompassMsg



# variables
# distance in meters considered close enough to destination to stop
close_enough = 4
# specifies the navigation component's cycle time
nav_cycle_period = 200

dest_lat = 0
dest_lon = 0

# Initialize GPS and compass readings


cur_lat = 0
cur_lon = 0
heading = 0
heading_gps = 0
pitch = 0
roll = 0

 

# gets robot's current longitude and latitude from gps msg
def get_gps(data):
	global cur_lat
	global cur_lon
	global heading_gps
	cur_lat = data.latitude
	cur_lon = data.longitude
	heading_gps = data.heading
	
 	print rospy.get_caller_id(),'Received Coordinates - Latitude: ',data.latitude,' Longitude: ',data.longitude


FILTER_SIZE = 10
filter_buffer = [0] * FILTER_SIZE 
filter_index = 0
filter_sum = 0
# gets robot's current heading, pitch, and roll, from compass msg
def get_compass(data):
	global heading
	global pitch
	global roll
	global filter_buffer
	global filter_index
	global filter_sum
	
	filter_buffer[filter_index] = data.heading
	filter_sum = 0
	for datapoint in filter_buffer:
		filter_sum += get_angle_difference(filter_buffer[filter_index], datapoint)
	heading = filter_buffer[filter_index] + filter_sum/FILTER_SIZE
	if heading > 180:
		heading -= 360
	elif heading <= -180:
		heading += 360

	filter_index += 1
	if filter_index >= FILTER_SIZE:
		filter_index = 0
		
	pitch = data.pitch
	roll = data.roll
	update_navigation()	


	#print rospy.get_caller_id(),'Received Compass Info - Heading: ',data.heading



def check():
	print 'Current Location:', cur_lat, cur_lon
	print 'Destination:', final_lat, final_lon

	distance = get_distance(cur_lat, cur_lon, final_lat, final_lon)

	print "Distance to destination:", distance

# calculates distance from location A (lon_A, lat_A) to B (lon_B, lat_B)
def get_distance(lat_A, lon_A, lat_B, lon_B):
	# uses "Haversine Formula"

	# approx radius of earth in kilometers
	R = 6371 

	# calculations
	lon_D = math.radians(lon_B - lon_A)
	lat_D = math.radians(lat_B - lat_A)

	lat_A = math.radians(lat_A)
	lat_B = math.radians(lat_B)

	a = ( math.sin(lat_D / 2) * math.sin(lat_D / 2)
		 + math.sin(lon_D / 2) * math.sin(lon_D / 2)
		 * math.cos(lat_A) * math.cos(lat_B) )

	c = 2 * math.atan2( math.sqrt(a) , math.sqrt(1-a) )

	d = R * c;

	d = d *1000  # Return meters

	return d



def update_navigation():
	distance = get_distance(cur_lat, cur_lon, dest_lat, dest_lon)

	if distance < close_enough:
		return True

	print 'Distance to Destination', distance

	angle_to_target = get_angle(cur_lat, cur_lon, dest_lat, dest_lon)

	print 'Angle to target = ', angle_to_target

	heading_error = get_heading_error(heading, angle_to_target)

	print 'Heading error = ', heading_error

	print 'Received Compass Info - Heading: ', heading
	print 'Received GPS     Info - Heading: ', heading_gps
	# set steering angle
	# max_steering for traxxas is 0.35 radians (20 degrees)
	max_steering = 20

	if heading_error > max_steering:
		steering = max_steering
	elif heading_error < - max_steering:
		steering = - max_steering
	else:
		steering = heading_error

	speed = 1

	print 'Steering = ', steering, 'Speed = ', speed
	move(speed, steering)

	return False



def update_navigation_PID():
	distance = get_distance(cur_lat, cur_lon, dest_lat, dest_lon)

	if distance < close_enough:
		return True

	print 'Distance to Destination', distance

	angle_to_target = get_angle(cur_lat, cur_lon, dest_lat, dest_lon)

	print 'Angle to target = ', angle_to_target

	heading_error = get_heading_error(heading, angle_to_target)

	total_error += heading_error

	delta_error = heading_error - previous_error

	previous_error = heading_error

	steering = Kp * heading_error + Ki * total_error + Kd * delta_error

	print 'Heading Error = ', heading_error

	# set steering angle
	# max_steering for traxxas is 0.35 radians (20 degrees)
	max_steering = 20

	if steering > max_steering:
		steering = max_steering
	elif steering < - max_steering:
		steering = - max_steering


	speed = calculate_speed(distance, 2, steering)

	print 'Steering = ', steering, 'Speed = ', speed
	#move(speed, steering)

	return False

def calculate_speed(distance, desired_speed, desired_steering):
	
	if (math.fabs(desired_steering) > 18):
		return 0.6

	if (distance > 6):
		return desired_speed
	elif (distance > 5):
		return 1.5
	elif (distance > 4):
		return 1
	elif (distance >3):
		return 0.7
	else:
		return 0.5




def get_angle(cur_lat, cur_lon, dest_lat, dest_lon):
	#TODO Handle vertical, horizontal

	x_err = dest_lat - cur_lat
	y_err = -1 * (dest_lon - cur_lon)

	angle_radians = math.atan2(y_err, x_err)

	angle_deg = math.degrees(angle_radians)

	return angle_deg


def get_angle_difference (angle1, angle2):
	difference = angle2 - angle1
	if difference > 180:
		difference -= 360
	elif difference <= -180:
		difference += 360
	return difference


def get_heading_error(cur_heading, angle_to_target):
	if cur_heading < 0:
		cur_heading = 360 + cur_heading

	if angle_to_target < 0:
		angle_to_target = 360 + angle_to_target

	heading_error = angle_to_target - cur_heading

	if heading_error > 180:
		heading_error = heading_error - 360
	elif heading_error < -180:
		heading_error = 360 + heading_error

	return heading_error



def move(speed, steering):
	# publish speed and steering to traxxas_node AckermannDriveMsg
	ackermann_drive_msg = AckermannDriveMsg()
	ackermann_drive_msg.speed = speed
	ackermann_drive_msg.steering_angle = steering
	pub.publish(ackermann_drive_msg)




if __name__ == '__main__':
	rospy.init_node('outdoor_navigation', anonymous=True)

	dest_lat = float(sys.argv[1])
	dest_lon = float(sys.argv[2])
	
	rospy.Subscriber("gps/measurement", GPSMsg, get_gps)

	rospy.Subscriber("compass/measurement", CompassMsg, get_compass) 
	pub = rospy.Publisher('/traxxas_node/ackermann_drive', AckermannDriveMsg)




	
	

	rospy.spin()

	
