#!/usr/bin/env python
# ros
import roslib; roslib.load_manifest('teleop_twist_gui')
import rospy
from geometry_msgs.msg import Twist
# qt
from PyQt4 import QtGui, QtCore
from teleop_mainWindow import Ui_MainWindow

class Publisher(object):
  def __init__(self,gui,commandFrequency):
    self.gui = gui
    self.pub = rospy.Publisher('cmd_vel',Twist)
    rospy.init_node('teleop')
    self.running = True
    self.commandFrequency = commandFrequency
    self.linearScale = rospy.get_param('~linearScale')
    self.angularScale = rospy.get_param('~angularScale')
    self.gui.mainWindow.setScale([self.linearScale,self.angularScale])

  def run(self):
    r = rospy.Rate(self.commandFrequency)
    msg = Twist()
    time = rospy.get_time()
    while not(rospy.is_shutdown()) and self.running:
      self.gui.mainWindow.updateVels(rospy.get_time()-time)
      time = rospy.get_time()
      fwd,turn = self.gui.mainWindow.getCurrentVels()
      msg.linear.x = fwd * self.linearScale
      msg.angular.z = turn * self.angularScale
      self.pub.publish(msg)
      r.sleep()

  def shutdown(self):
    self.running = False

class MainWindow(QtGui.QMainWindow,Ui_MainWindow):
  def __init__(self,parent=None):
    super(MainWindow,self).__init__(parent)
    self.setupUi(self)
    self.stopButton.clicked.connect(self.stop)
    self.velocity = [0,0]
    self.velocityBounds = [[-1,1],[-1,1]]
    self.scale = [0,0]
    self.displayCurrentVels()
    self.directionsActivated = [False for i in range(4)]
    self.LEFT = 0
    self.RIGHT = 1
    self.UP = 2
    self.DOWN = 3
    self.keys = [[] for i in range(4)]
    self.keys[self.LEFT] =  [QtCore.Qt.Key_Left, QtCore.Qt.Key_A]
    self.keys[self.RIGHT] = [QtCore.Qt.Key_Right,QtCore.Qt.Key_D]
    self.keys[self.UP] =    [QtCore.Qt.Key_Up,   QtCore.Qt.Key_W]
    self.keys[self.DOWN] =  [QtCore.Qt.Key_Down, QtCore.Qt.Key_S]
    self.fwdInc.setValue(1)
    self.turnInc.setValue(1)
 
  def setScale(self,scale):
   self.scale = scale

  def displayCurrentVels(self):
    self.fwdVelFrac.setText('% 1.2f' % self.velocity[0])
    self.turnVelFrac.setText('% 1.2f' % self.velocity[1])
    self.fwdVelAbs.setText('% 2.2f' % (self.velocity[0] * self.scale[0]))
    self.turnVelAbs.setText('% 2.2f' % (self.velocity[1] * self.scale[1]))

  def stop(self,_):
    self.velocity = [0,0]
    self.displayCurrentVels()

  def getCurrentVels(self):
    return self.velocity

  def bound(self,val,minVal,maxVal):
    return max(minVal,min(maxVal,val))

  def setVel(self,dim,inc,dec):
    if inc and dec:
      # cancel each other out
      return
    if not(inc) and not(dec):
      # slow down
      if self.velocity[dim] < 0:
        self.velocity[dim] += self.velocityInc[dim]
        self.velocity[dim] = self.bound(self.velocity[dim],self.velocityBounds[dim][0],0)
      else:
        self.velocity[dim] -= self.velocityInc[dim]
        self.velocity[dim] = self.bound(self.velocity[dim],0,self.velocityBounds[dim][1])
      return

    if inc:
      dir = 1
    if dec:
      dir = -1
    self.velocity[dim] += self.velocityInc[dim] * dir
    self.velocity[dim] = self.bound(self.velocity[dim],self.velocityBounds[dim][0],self.velocityBounds[dim][1])

  def updateVels(self,timePassed):
    self.velocityInc = [self.fwdInc.value()*timePassed,self.turnInc.value()*timePassed]
    self.setVel(1,self.directionsActivated[self.LEFT],self.directionsActivated[self.RIGHT])
    self.setVel(0,self.directionsActivated[self.UP],self.directionsActivated[self.DOWN])
    self.displayCurrentVels()
    
  def keyPressEvent(self,event):
    self.processKey(event.key(),True)
 
  def keyReleaseEvent(self,event):
    self.processKey(event.key(),False)

  def processKey(self,key,press):
    for ind,keys in enumerate(self.keys):
      if key in keys:
        self.directionsActivated[ind] = press


class Gui(object):
  def __init__(self,args):
    self.app = QtGui.QApplication(args)
    self.mainWindow = MainWindow()
    self.mainWindow.show()
    self.mainWindow.setFocus()

  def run(self):
    self.app.exec_()

def main(args):
  import threading
  commandFrequency = 10
  gui = Gui(args)
  pub = Publisher(gui,commandFrequency)
  t = threading.Thread(target=pub.run)
  t.start()
  gui.run()
  pub.shutdown()
  t.join()

if __name__ == '__main__':
  import sys
  main(sys.argv)
