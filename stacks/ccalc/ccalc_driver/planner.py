#!/usr/bin/env python
import roslib; roslib.load_manifest('ccalc_driver')
import rospy
import pexpect
import subprocess
import sys
import os

swipl_param=rospy.get_param("ccalc_driver/swi_prolog_location")
ccalc_param=rospy.get_param("ccalc_driver/ccalc_location")
launch= swipl_param+" -f "+ccalc_param
ccalc = pexpect.spawn(launch)
ccalc.expect('\?\-')
ccalc.sendline('set(solver,grasp).')
ccalc.expect('\?\-')
loadfile="loadf \'"+sys.argv[1]+"\'.\n"
ccalc.sendline(loadfile)
ccalc.expect('\?\-')
ccalc.sendline('query 1.\n')
ccalc.expect('\?\-')
ccalc.sendline('halt.')
ccalc.interact()
#ccalc.expect(pexpect.EOF)





