#!/usr/bin/env python

import urllib, urllib2, getpass, subprocess

url = 'http://horatio.cs.utexas.edu/login.cgi'
username = raw_input('username: ')
password = getpass.getpass('password: ')

values = {'username': username, 'password':password}

try:
	data = urllib.urlencode(values)
	req = urllib2.Request(url,data)
	response = urllib2.urlopen(req)
except Exception, detail:
	print 'Err ', detail

#subprocess.check_call(['ifconfig'])
