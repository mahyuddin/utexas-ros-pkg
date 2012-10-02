#!/usr/bin/python
import yaml, sys, re

udevFile = sys.argv[1]
camList = sys.argv[2]
cameras = []
f = open(udevFile, "r")
for line in f:
  m = re.search('SYMLINK\+="([\w+\-]+)', line)
  camName = m.groups(0)[0]
  cameras += [camName]
f.close()

c = open(camList, "w")
output = yaml.dump({'cameras' : cameras}, indent=True)
c.write(output)
c.close()
