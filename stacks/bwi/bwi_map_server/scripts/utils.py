#!/usr/bin/env python

def construct(name, suffix):
  assert '/' not in name, "level name cannot contain forward slash"
  return '/' + name + '/' + suffix

def deconstruct(name, suffix):

def frameIdFromLevelName(name):
  return construct(name,'map')

def mapTopicFromLevelName(name):
  return construct(name,'map')

def metadataTopicFromLevelName(name):
  return construct(name,'map_metadata')

def nameFromLevelFrameId(frame_id):
  components = frame_id.split('/')
  assert len(components) == 3 and components[0] == '' and components[2] == 'map', "malformed frame id provided for obtaining name"
  return components[1] 

def nameFromLevelMapTopic(map_topic):
  return nameFromLevelFrameId(map_topic)
