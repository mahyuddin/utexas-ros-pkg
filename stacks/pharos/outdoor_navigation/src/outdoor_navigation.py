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
close_enough = 2.5
# specifies the navigation component's cycle time
nav_cycle_period = 200

final_lat = 0
final_lon = 0

# Initialize GPS and compass readings
cur_lon = 0
cur_lat = 0

heading = 0
pitch = 0
roll = 0




# gets robot's current longitude and latitude from gps msg
def get_gps(data):
	a = data.latitude
	b = data.longitude
	
 	print rospy.get_caller_id(),'Received Coordinates - Latitude: ',data.latitude,' Longitude: ',data.longitude

	check(a, b)


# gets robot's current heading, pitch, and roll, from compass msg
def get_compass(data):
	heading = data.heading
	pitch = data.pitch
	roll = data.roll

	# print rospy.get_caller_id(),'Received Compass Info - Heading: ',data.heading



def check(a, b):
	print 'Current Location:', a, b
	print 'Destination:', final_lat, final_lon

	distance = get_distance(a, b, final_lat, final_lon)

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

	return d


# checks if robot's current gps location is close enough to destination
def there_yet(longitude, latitude):
	
	distance = get_distance(cur_lon, cur_lat, longitude, latitude)


	if distance < close_enough:
		return True
	else:
		return False




if __name__ == '__main__':
	rospy.init_node('outdoor_navigation', anonymous=True)

	final_lat = float(sys.argv[1])
	final_lon = float(sys.argv[2])
	
	rospy.Subscriber("gps/measurement", GPSMsg, get_gps)

	rospy.Subscriber("compass/measurement", CompassMsg, get_compass)



	
	

	rospy.spin()

	