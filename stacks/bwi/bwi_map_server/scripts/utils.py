#!/usr/bin/env python

def frameIdFromLevelName(name):
    assert '/' not in name, "level name cannot contain forward slash"
    return '/' + name + '/map'

def nameFromLevelFrameId(frame_id):
    components = frame_id.split('/')
    assert len(components) == 3 and components[0] == '' and components[2] == 'map', "malformed frame id provided for obtaining name"
    return components[1] 
