#!/usr/bin/env python

# Based on the turtlebot_addr.py script in the turtlebot_bringup package
# Original Authors: Tully Foote
# License: BSD
# Source: https://kforge.ros.org/turtlebot/turtlebot/raw-file/fb8354a98ac5/turtlebot_bringup/scripts/turtlebot_addr.py

import sys
import netifaces

if len(sys.argv) != 2:
    preferred_interfaces = ['eth','wlan']
else:
    preferred_interfaces = [sys.argv[1]]

sorted_interfaces = sorted(netifaces.interfaces())

for pref in preferred_interfaces:
    for inf in netifaces.interfaces():
        if inf.startswith(pref):
            addrs = netifaces.ifaddresses(inf)
            if not netifaces.AF_INET in addrs:
                continue
            else:
                print addrs[netifaces.AF_INET][0]['addr']
                sys.exit(0)

print >> sys.stderr, "ROS: failed to determine IP address for interfaces %s. You won't be able to connect to this machine externally."%str(preferred_interfaces)
print "127.0.0.1" #bind to loopback for now
sys.exit(1)
