#!/usr/bin/env python
import roslib; roslib.load_manifest('bwi_dispatcher')
import rospy
import bwi_dispatcher
import bwi_dispatcher.srv
import bwi_dispatcher.msg
from bwi_dispatcher.msg import MachineStatus

import gateway_comms.msg
import gateway_comms.srv

# when dispatcher comes online, it needs to advertise /machines to the gateway, and pull /machines from the gateway
# publish the presence of its machine to gateway with no services
# when a device on the machine comes online, make a service call (from launch file) to the dispatcher advertising that device and its public topics. the dispatcher then tracks this device and advertises those topics to the gateway
# when an app/device needs another device, it requests it from the dispatcher. the dispatcher will allow requests by:
#  type, machine
#  machine
#  specifiic topic (nice for debug tools, not good for general purpose)

# dispatcher functions:
# publishStatus
# pullStatuses
# setupStatus -- this is this initial setup, just publishes machine with no topics/services
# listTopics(machine = None, type = None)
# listServices(machine = None, type = None))
# pullTopics(machine = None, type = None)
# 

class Dispatcher():
  def __init__(self):
    self.status = MachineStatus()
    self.statuses = {}
    
    statusTopic = rospy.get_param('~status_topic', '/machine')
    self.publisher = rospy.Publisher(statusTopic, MachineStatus)
    self.publishRate = rospy.get_param('~publish_rate', 10)
    self.maxDuration = rospy.get_param('~max_status_duration', 20)
    # advertise statusTopic
    # pull statusTopic from the gateway
    self.dservices, self.gservices = self.__setupRosServices()

  def __setupRosServices(self):
    dispatcher_services = {}
    dispatcher_services['list_topics'] = rospy.Service('~list_topics', bwi_dispatcher.srv.ListTopics, self.rosServiceListTopics)
    # ... snip ... #

    gateway_services = {}
    gateway_services['advertise'] = rospy.ServiceProxy('advertise', gateway_comms.srv.Advertise)
  
    return dispatcher_services, gateway_services

  def __publishStatus(self):
    rospy.loginfo("publishing status")

  def handleStatus(self, status):
    self.statuses[status.machine] = status
    status.timestamp = rospy.Time.now()

  def updateStatuses(self):
    for machine in self.statuses:
      s = self.statuses[machine]
      duration = rospy.Time.now() - s.timestamp
      if duration.secs > self.maxDuration:
        del self.statuses[machine]

  def spin(self):
    rospy.Timer(rospy.Duration(self.publishRate), self.__publishStatus)
    rospy.Timer(rospy.Duration(10), self.updateStatuses)
    rospy.spin()    

  ################################################################
  # Ros Service Callbacks
  ###############################################################

  '''
    The idea here is, we list out whatever data we might need,
    pick the topics we want, and then pull them individually before
    subscribing. If we know our topics ahead of time, we can just
    pull them (for debugging)
  '''

  def rosServiceListTopics(self, msg):
    response = bwi_dispatcher.srv.ListTopicsResponse()
    response.topics = [t for t in d.topics for n in self.devices + self.applications]
    return response

  def rosServiceListServices(self, msg):
    response = bwi_dispatcher.srv.ListServicesResponse()
    services = []
    for d in self.status.devices:
      for s in d.services:
        services += [s]
    for a in self.status.applications:
      for s in a.services:
        services += [s]
    response.services = services
    return response

  def rosServiceListDevices(self, msg):
    response = bwi_dispatcher.srv.ListDevicesResponse()
    response.devices = self.status.devices
    return response

  def rosServiceListApplications(self, msg):
    response = bwi_dispatcher.srv.ListApplicationsResponse()
    response.applications = self.status.applications
    return response

  def rosServiceListMachines(self, msg):
    response = bwi_dispatcher.srv.ListMachinesResponse()
    response.machines = self.statuses.keys()
    return response

  def rosServicePullAll(self, pull):
    response = bwi_dispatcher.srv.PullAllResponse()
    request = gateway_comms.srv.RemoteAllRequest()
    request.cancel = pull.cancel
    request.gateway = pull.machine
    self.gservices['pull_all'](request)
    return response

  def rosServicePullTopic(self, pull):
    response = bwi_dispatcher.srv.PullTopicResponse()
    request = gateway_comms.srv.RemoteRequest()
    request.cancel = pull.cancel
    r = gateway_comms.msg.RemoteRule()
    r.gateway = pull.machine
    r.rule.type = pull.type
    r.rule.name = pull.name
    r.rule.node = pull.node
    request.remote = r
    self.gservices['pull'](request)
    return response

  def __rosServiceAdvertiseNode(self, msg):
    response = bwi_dispatcher.srv.AdvertiseNodeResponse()
    request = gateway_comms.srv.AdvertiseRequest()
    request.cancel = msg.cancel
    for ad in msg.advertisements:
      r = gateway_comms.msg.Rule()
      r.type = ad.type
      r.name = ad.name
      r.node = ad.node
      request.rules.add(r)
    self.gservices['advertise'](request)
    return response

  def rosServiceAdvertiseAll(self, msg):
    response = bwi_dispatcher.srv.AdvertiseAllResponse()
    request = gateway_comms.srv.AdvertiseAllRequest()
    request.cancel = msg.cancel
    self.gservices['advertise_all'](request)
    return response

  def rosServiceAdvertiseApplication(self, msg):
    msg.node_type = bwi_dispatcher.msg.NodeType.APPLICATION
    filter(lambda a: a.name != msg.name, self.status.applications)
    if not msg.cancel:
      a = self.getNodeStatus(msg, bwi_dispatcher.msg.Application())
      self.status.applications += [a]
    response = self.__rosServiceAdvertiseNode(self, msg)
    return response

  def rosServiceAdvertiseDevice(self, msg):
    msg.node_type = bwi_dispatcher.msg.NodeType.DEVICE
    if not msg.cancel:
      d = self.getNodeStatus(msg, bwi_dispatcher.msg.Device())
      self.status.applications += [a]
    response = self.__rosServiceAdvertiseNode(self, msg)
    return response

  def getNodeStatus(self, msg, status):
    for ad in msg.advertisements:
      if ad.type == bwi_dispatcher.msg.AdvertisementTypes.PUBLISHER:
        t = bwi_dispatcher.msg.Topic()
        t.name = ad.name
        status.topics += [t]
      if ad.type == bwi_dispatcher.msg.AdvertisementTypes.SERVICE:
        s = bwi_dispatcher.msg.Service()
        s.name = ad.name
        status.services += [s]
    status.name = msg.name
    status.type = msg.type


if __name__ == '__main__':
  rospy.init_node('dispatcher')
  dispatcher = Dispatcher()
  dispatcher.spin()
  rospy.loginfo('Dispatcher shutting down.')
