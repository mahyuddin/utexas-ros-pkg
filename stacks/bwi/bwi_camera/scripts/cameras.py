#!/usr/bin/python
import sys, yaml

action = sys.argv[1]
camera = sys.argv[2]
camfile = sys.argv[3]
try:
  f = open(camfile, 'r')
  cameras = yaml.load(f.read())['cameras']
  f.close()
except IOError:
  cameras = []

if action == "add" and camera not in cameras:
  cameras += [camera]
elif action == "remove" and camera in cameras:
  cameras.remove(camera)
f = open(camfile, 'w')
f.write(yaml.dump({'cameras': cameras}))
f.close()
  


