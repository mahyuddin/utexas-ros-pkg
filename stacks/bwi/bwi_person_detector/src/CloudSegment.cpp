#include "CloudSegment.h"

CloudSegment::CloudSegment() : _parent(0) {
}

int CloudSegment::size() {
  return _points.size();
}

void CloudSegment::addPoint(const CPoint& point) {
  _points.push_back(point);
}

float CloudSegment::distanceTo(CloudSegment& other) {
  return CPOINT_DISTANCE(_centroid, other._centroid);
}

void CloudSegment::setParent(CloudBlob* parent) {
  _parent = parent;
}

CloudBlob* CloudSegment::getParent() {
  return _parent;
}

int CloudSegment::getLeft() {
  return _left;
}

int CloudSegment::getRight() {
  return _right;
}

void CloudSegment::setLeft(int value) {
  _left = value;
}

void CloudSegment::setRight(int value) {
  _right = value;
}

CPoint CloudSegment::getCentroid() {
  return _centroid;
}

void CloudSegment::computeStatistics() {
  float x = 0, y = 0, z = 0;
  _maxX = _maxY = _maxZ = -100;
  _minX = _minY = _minZ = 100;
  BOOST_FOREACH(CPoint point, _points) {
    _minX = std::min(_minX, point.x); _maxX = std::max(_maxX, point.x);
    _minY = std::min(_minY, point.y); _maxY = std::max(_maxY, point.y);
    _minZ = std::min(_minZ, point.z); _maxZ = std::max(_maxZ, point.z);
    x += point.x;
    y += point.y;
    z += point.z;
  }
  int n = _points.size();
  _centroid = CPoint(x/n,y/n,z/n);
}

float CloudSegment::minX() {
  return _minX;
}

float CloudSegment::maxX() {
  return _maxX;
}

float CloudSegment::minY() {
  return _minY;
}

float CloudSegment::maxY() {
  return _maxY;
}

float CloudSegment::minZ() {
  return _minZ;
}

float CloudSegment::maxZ() {
  return _maxZ;
}
