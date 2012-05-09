#!/usr/bin/env python

import sys
import netifaces

if len(sys.argv) > 1:
  interface = sys.argv[1]
else:
  interface = 'wlan0'

for inf in netifaces.interfaces():
  if inf.startswith(interface):
    addrs = netifaces.ifaddresses(inf)
    if not netifaces.AF_INET in addrs:
      continue
    else:
      print addrs[netifaces.AF_INET][0]['addr']
      sys.exit(0)

print >> sys.stderr, "Failed to determine IP address for interface: " + interface
print "localhost" #bind to loopback for now
sys.exit(1)
