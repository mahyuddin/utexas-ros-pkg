#!/usr/bin/env python

import sys
import netifaces

for inf in netifaces.interfaces():
  if inf.startswith('ath0'):
    addrs = netifaces.ifaddresses(inf)
    if not netifaces.AF_INET in addrs:
      continue
    else:
      print addrs[netifaces.AF_INET][0]['addr']
      break
else:
  print >> sys.stderr, "failed to determine IP address"
  print "127.0.0.1" #bind to loopback for now
  sys.exit(1)
