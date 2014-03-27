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

# Initialize GPS and compass readings
cur_lon = 0
cur_lat = 0

heading = 0
pitch = 0
roll = 0




# gets robot's current longitude and latitude from gps msg
def get_gps(data):

	print rospy.get_caller_id(),'Received Coordinates - Longitude: ',data.longitude,' Latitude: ',data.latitude



# gets robot's current heading, pitch, and roll, from compass msg
def get_compass(data):

	print rospy.get_caller_id(),'Received Compass Info - Heading: ',data.heading



class Navigate(Thread):
	"""Thread to run navigation computations and publish to the traxxas_node AckermannDriveMsg for movement"""
	
	# constructor
	def __init__(self, dest_longitude, dest_latitude):
		# call Thread superclass constructor
		Thread.__init__(self, name="Navigate")
		
		# set class variables
		self.dest_longitude = dest_longitude
		self.dest_latitude = dest_latitude

	# fuction called by Thread.start() to start navigation
	def run(self):
		print "Thread Navigate Starting."
		print 'Navigating to (', dest_longitude, ',', dest_latitude, ')'


# calculates distance from location A (lon_A, lat_A) to B (lon_B, lat_B)
def get_distance(lon_A, lat_A, lon_B, lat_B):
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
	
	(cur_lon, cur_lat) = get_gps()

	distance = get_distance(cur_lon, cur_lat, longitude, latitude)

	return distance

	# if distance < close_enough:
	# 	return True
	# else:
	# 	return False

	# pass



# def do_next_move():
# 	# TODO
# 	pass



# def navigate(longitude, latitude, speed):
# 	done = False
# 	error = False

# 	while(not done and not error):
# 		(cur_lon, cur_lat) = get_gps()
# 		# check if current coordinates are valid

# 		cur_heading = get_compass()[0]
# 		# check if current heading is valid

# 		done = do_next_move(cur_lon, cur_lat, longitude, latitude, speed)

# 		if not done:
# 			time.sleep(nav_cycle_period / 1000)

# 	pass



if __name__ == '__main__':
	rospy.init_node('outdoor_navigation', anonymous=True)


	# parse arguments
	# TODO better arugments with argparse library
	# if len(sys.argv) < 3:
	# 	print 'No arguments provided.\n', 'Usage: navigate.py [longitude] [latitude]'
	# 	sys.exit(1)

	rospy.Subscriber("gps/measurement", GPSMsg, get_gps)

	rospy.Subscriber("compass/measurement", CompassMsg, get_compass)


	longitude = 0
	latitude = 0
	# set location to navigate
	# longitude = sys.argv[0]
	# latitude = sys.argv[1]
	# speed = 1;

	# final_lon = float(30)
	# final_lat = float(-98)

	# (cur_lon, cur_lat) = get_gps()
	# (heading, pitch, roll) = get_compass()

	# distance = there_yet(final_lon, final_lat)

	# print 'Distance to desired location: ', distance

	# print 'Heading = ', heading

	navigate_thread = Navigate(longitude, latitude)
	navigate_thread.start()

	rospy.spin()

	# subscribe to GPS message data
	# subscribe to Compass message data
	# TODO
	'''need to setup get_gps and get_compass function -- ask pedro for help''' 



	# # call navigate function
	# if navigate(latitude, longitude, speed) == True:
	# 	print 'Success, now at location:', get_location()
	# else:
	# 	print 'Failure, now at location:', get_locatoin()