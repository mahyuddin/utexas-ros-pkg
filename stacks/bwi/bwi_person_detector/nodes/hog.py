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


    self.fpsCount = 0
    self.fpsStart = rospy.get_rostime()
    self.fpsFreq = 5

    self.bridge = CvBridge()
    self.storage = cv.CreateMemStorage(0)
    self.doDisplay = doDisplay
    self.pub = pub

    self.gray = None
    self.small_img = None

  def inside(self,r,q):
    (rx,ry),(rw,rh) = r
    (qx,qy),(qw,qh) = q
    return rx > qx and ry > qy and rx + rw < qx + qw and ry + rh < qy + qh

  def createBuffers(self,img):
    # allocate temporary images
    if (self.gray is None) or (self.gray.width != img.width) or (self.gray.height != img.height):
      self.gray = cv.CreateImage((img.width,img.height), 8, 1)
    w = 320#cv.Round(img.width / self.image_scale)
    h = 240#cv.Round(img.height / self.image_scale)
    if (self.small_img is None) or (self.small_img.width != w) or (self.small_img.height != h):
      self.small_img = cv.CreateImage((w,h), 8, 1)
      self.image_scale = img.width / w

  def publishDetection(self,x,y,w,h):
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
    
    found = list(cv.HOGDetectMultiScale(self.small_img, self.storage, win_stride=(8,8), padding=(32,32), scale=1.05, group_threshold=2))
    found_filtered = []

    for r in found:
      insidef = False
      for q in found:
        if self.inside(r,q):
          insidef = True
          break
      if not insidef:
        found_filtered.append(r)
    
    for (rx,ry),(rw,rh) in found_filtered:
      self.publishDetection(rx,ry,rw,rh)
      if self.doDisplay:
        self.displayDetection(img,rx,ry,rw,rh)
    if self.doDisplay:
      cv.ShowImage("result", img)
      cv.WaitKey(6)
    
    self.fpsCount += 1
    now = rospy.get_rostime()
    timePassed = (now - self.fpsStart).to_sec()
    if timePassed > self.fpsFreq:
      print >>sys.stderr,"FPS: %2.1f"%(self.fpsCount / timePassed)
      self.fpsCount = 0
      self.fpsStart = now

  def displayDetection(self,img,rx,ry,rw,rh):
    tl = tuple([int(a * self.image_scale) for a in [rx + int(rw*0.1),ry + int(rh*0.07)]])
    br = tuple([int(a * self.image_scale) for a in [rx + int(rw*0.9),ry + int(rh*0.87)]])
    cv.Rectangle(img,tl,br,(0,255,0),3)

if __name__ == '__main__':
    rospy.init_node('glados_person_detection')
    image_topic = rospy.resolve_name("/glados_person_detection/image_raw")
    pub = rospy.Publisher("/glados_person_detection/person",geometry_msgs.msg.Point)
    detector = Detector(pub,True)
    rospy.Subscriber(image_topic, sensor_msgs.msg.Image, detector.detect)
    rospy.spin()


