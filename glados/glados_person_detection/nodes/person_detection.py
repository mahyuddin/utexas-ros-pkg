#!/usr/bin/python

import roslib
roslib.load_manifest('glados_person_detection')
import rospy

import sys, os

import sensor_msgs.msg
import geometry_msgs.msg
from cv_bridge import CvBridge
import cv

class Detector(object):
  def __init__(self,pub,doDisplay):
    # Parameters for haar detection
    # From the API:
    # The default parameters (scale_factor=2, min_neighbors=3, flags=0) are tuned 
    # for accurate yet slow object detection. For a faster operation on real video 
    # images the settings are: 
    # scale_factor=1.2, min_neighbors=2, flags=CV_HAAR_DO_CANNY_PRUNING, 
    # min_size=<minimum possible face size

    self.min_size = (10, 10)
    self.image_scale = 2
    self.haar_scale = 1.2
    self.min_neighbors = 2
    self.haar_flags = 0

    self.bridge = CvBridge()
    self.storage = cv.CreateMemStorage(0)
    self.doDisplay = doDisplay
    self.pub = pub

    haarfiles = ['frontalface_alt','profileface','mcs_upperbody']
    base = os.path.join(roslib.packages.get_pkg_dir('opencv2'),'opencv/share/opencv/haarcascades/haarcascade_%s.xml')
    self.haarfiles = [cv.Load(base % x) for x in haarfiles]
    self.distanceParams = [1,1,1]
    self.gray = None
    self.small_img = None

  def createBuffers(self,img):
    # allocate temporary images
    if (self.gray is None) or (self.gray.width != img.width) or (self.gray.height != img.height):
      self.gray = cv.CreateImage((img.width,img.height), 8, 1)
    w = cv.Round(img.width / self.image_scale)
    h = cv.Round(img.height / self.image_scale)
    if (self.small_img is None) or (self.small_img.width != w) or (self.small_img.height != h):
      self.small_img = cv.CreateImage((cv.Round(img.width / self.image_scale), cv.Round (img.height / self.image_scale)), 8, 1)

  def publishDetection(self,x,y,w,h,n):
    # TODO
    msg = geometry_msgs.msg.Point()
    msg.x = x
    msg.y = y
    msg.z = w * h
    self.pub.publish(msg)

  def detect(self,imgmsg):
    img = self.bridge.imgmsg_to_cv(imgmsg, 'bgr8')
    self.createBuffers(img)

    # convert color input image to grayscale
    cv.CvtColor(img, self.gray, cv.CV_BGR2GRAY)

    # scale input image for faster processing
    cv.Resize(self.gray, self.small_img, cv.CV_INTER_LINEAR)

    cv.EqualizeHist(self.small_img, self.small_img)
    
    for haarfile,distanceParams in zip(self.haarfiles,self.distanceParams):
      res = cv.HaarDetectObjects(self.small_img, haarfile, self.storage, self.haar_scale, self.min_neighbors, self.haar_flags, self.min_size)
      for ((x,y,w,h),n) in res:
        self.publishDetection(x,y,w,h,n)
        if self.doDisplay:
          self.displayDetection(img,x,y,w,h,n)
    if self.doDisplay:
      cv.ShowImage("result", img)
      cv.WaitKey(6)

  def displayDetection(self,img,x,y,w,h,n):
    pt1 = (int(x * self.image_scale), int(y * self.image_scale))
    pt2 = (int((x + w) * self.image_scale), int((y + h) * self.image_scale))
    cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)

if __name__ == '__main__':
    rospy.init_node('person_detection')
    image_topic = rospy.resolve_name("/usb_cam/image_raw")
    pub = rospy.Publisher("/person_detection/faces",geometry_msgs.msg.Point)
    detector = Detector(pub,True)
    rospy.Subscriber(image_topic, sensor_msgs.msg.Image, detector.detect)
    rospy.spin()
