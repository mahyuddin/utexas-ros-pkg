import os
from threading import Lock
import roslib
roslib.load_manifest('bwi_visualization')
import rospy

from QtCore import Qt, Signal, Slot
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
import bwi_msgs.msg
import bwi_msgs.srv
import std_srvs.srv

class DemoPlugin(Plugin):
    new_person = Signal(int)
    def __init__(self, context):
      super(DemoPlugin, self).__init__(context)
      # give QObjects reasonable names
      self.setObjectName('DemoPlugin')

      # create QWidget
      self._widget = QWidget()
      # get path to UI file which is a sibling of this file
      # in this example the .ui file is in the same folder as this Python file
      ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'DemoPlugin.ui')
      # extend the widget with all attributes and children from UI file
      loadUi(ui_file, self._widget)
      # give QObjects reasonable names
      self._widget.setObjectName('DemoPluginUi')
      # add widget to the user interface
      context.add_widget(self._widget)

      self.register_machine = 'machineX'
      
      self.mutex = Lock()
      self.new_person.connect(self.update_persons)
      self.connect_buttons()
      self.persons = dict()
      self.connect_topics()

    def shutdown_plugin(self):
      # TODO unregister all publishers here
      pass

    def save_settings(self, plugin_settings, instance_settings):
      # TODO save intrinsic configuration, usually using:
      # instance_settings.set_value(k, v)
      pass

    def restore_settings(self, plugin_settings, instance_settings):
      # TODO restore intrinsic configuration, usually using:
      # v = instance_settings.value(k)
      pass

    def connect_buttons(self):
      self._widget.btnRegStart.pressed.connect(self.register_start)
      self._widget.btnRegStop.pressed.connect(self.register_stop)
      self._widget.btnNavEntrance.pressed.connect(self.navigate_entrance)
      self._widget.btnNavDana.pressed.connect(self.navigate_dana)

    def register_start(self):
      self.regStartClient()
      print "registering detections"

    def register_stop(self):
      self.regStopClient()
      print "registration complete"

    def connect_topics(self):
      self.personSub = rospy.Subscriber('/global/person_detections', bwi_msgs.msg.PersonDetectionArray, self.person_callback)
      self.regStartClient = rospy.ServiceProxy('/' + self.register_machine + '/axis9/register_start', std_srvs.srv.Empty)
      self.regStopClient = rospy.ServiceProxy('/' + self.register_machine + '/axis9/register_stop', std_srvs.srv.Empty)
      self.navClient = rospy.ServiceProxy('/navigate', bwi_msgs.srv.NavigatePerson)

    @Slot(long)
    def update_persons(self, id):
      self.mutex.acquire()
      print "adding person %u" % id
      self._widget.cbxPersons.addItem("%u" % id)
      self.mutex.release()
    
    def navigate_entrance(self):
      id = int(self._widget.cbxPersons.currentText())
      self.navigate(id,1,1,"demo1")
      print "navigating %i to entrance" % id

    def navigate_dana(self):
      id = long(self._widget.cbxPersons.currentText())
      print "navigating %i to dana's lab" % id

    def navigate(self, id, x, y, level):
      request = bwi_msgs.srv.NavigatePersonRequest()
      request.person_id = id
      request.goal.level_id = level
      request.goal.point.x = x
      request.goal.point.y = y
      request.goal.point.z = 0
      self.navClient(request)
      

    def person_callback(self, data):
      for person in data.detections:
        if person.id:
          if person.id not in self.persons:
            self.new_person.emit(person.id)
          self.persons[person.id] = person

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a dialog to offer the user a set of configuration
