#!/usr/bin/env python

import roslib; roslib.load_manifest('bwi_apps')
import rospy
import thread

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovariance
from bwi_msgs.msg import ObjectArray
from bwi_msgs.msg import Object

class LocationAggregator:
  def __init__(self):
    rospy.init_node('listener', anonymous=True)

    try: 
      self.in_topics = rospy.get_param('~in_topics')
      print self.in_topics
    except KeyError:
      rospy.logfatal("Did you forget to supply the input topics (~in_topics)")
      return

    self.out_msg = ObjectArray()
    self.publish_rate = rospy.get_param('~publish_rate', 10)
    self.out_topic = rospy.get_param('~out_topic', 'detections')
    self.global_frame = rospy.get_param('~global_frame', '/map')
    self.publisher = rospy.Publisher(self.out_topic, ObjectArray)

    for topic, id in self.in_topics.iteritems():
      rospy.loginfo("Subscribing to %s for object id %s" %(topic, id))
      rospy.Subscriber(topic, Vector3, self.callback, callback_args=id)

    self.mutex = thread.allocate_lock()
    self.spin()
    
  def callback(self, data, id):

    self.mutex.acquire()

    try:
      object = Object()
      object.id = id
      object.pose = self.vector3ToPose(data)
      self.addObjectSafely(self.out_msg.objects, object)
      
    finally:
      self.mutex.release()
   
  def vector3ToPose(self, data):
    pose = PoseWithCovariance()
    pose.pose.position.x = data.x
    pose.pose.position.y = data.y
    pose.pose.position.z = data.z
    pose.covariance[0] = 0.001
    pose.covariance[7] = 0.001
    pose.covariance[14] = 0.001
    pose.covariance[21] = 0.001
    pose.covariance[28] = 0.001
    pose.covariance[35] = 0.001
    return pose

  def addObjectSafely(self, list, object):
    object_id = -1
    for i in range(len(list)):
      if list[i].id == object.id:
        object_id = i
    if object_id == -1:
      list.append(object)
    else:
      del list[object_id]
      list.append(object)

  def spin(self):
    rate = rospy.Rate(self.publish_rate)
    while not rospy.is_shutdown():
      self.mutex.acquire()
      try:
        self.out_msg.header.frame_id = self.global_frame
        self.out_msg.header.stamp = rospy.get_rostime()
        self.publisher.publish(self.out_msg)
        del self.out_msg.objects[:]
      finally:
        self.mutex.release()
      rate.sleep()

if __name__ == '__main__':
  location_aggregator = LocationAggregator()
