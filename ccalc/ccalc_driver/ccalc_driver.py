#! /usr/bin/env python

import roslib; roslib.load_manifest('ccalc_driver')
import rospy
import pexpect
import subprocess
import sys
import os

if len(sys.argv)<2:
	print "No input file is given"
	sys.exit()
else:
	swipl_param=rospy.get_param("ccalc_driver/swi_prolog_location")
	ccalc_param=rospy.get_param("ccalc_driver/ccalc_location")
	navigate_param=rospy.get_param("ccalc_navigate/data")
	inputFile=navigate_param+"/"+sys.argv[1]
	if (os.path.isfile(inputFile)):
		print "Launch CCalc with file "+inputFile
		outputFile = open(navigate_param+"/"+sys.argv[1]+"_plan","w")
		planner = "./planner.py "+inputFile
		retcode = subprocess.Popen(planner, shell = True, stdout=outputFile)
		retcode.wait()
		outputFile.close()

	else:
		print "File "+sys.argv[1]+" not found in directory "+navigate_param




