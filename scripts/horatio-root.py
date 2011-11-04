#!/usr/bin/env python

import urllib, urllib2, getpass, subprocess, socket

url = 'http://horatio.cs.utexas.edu/login.cgi'
username = raw_input('username: ')
password = getpass.getpass('password: ')

values = {'username': username, 'password':password}

#See if waiting for horatio to come up will help
counter = 0;
while counter < 5:
	try:
		socket.getaddrinfo('horatio.cs.utexas.edu', 80)
		break
	except Exception, detail:
                counter = counter + 1
                time.sleep(1.0)

try:
	data = urllib.urlencode(values)
	req = urllib2.Request(url,data)
	response = urllib2.urlopen(req)
except Exception, detail:
	print 'Err ', detail

#subprocess.check_call(['ifconfig'])
