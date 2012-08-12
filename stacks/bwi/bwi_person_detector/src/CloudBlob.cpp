#include "CloudBlob.h"

CloudBlob::CloudBlob() : _isAdded(false) {
}

CloudBlob::~CloudBlob() {
}

void CloudBlob::addSegment(CloudSegment* segment) {
  segment->setParent(this);
  _segments.push_back(segment);
}

void CloudBlob::merge(CloudBlob* other) {
  BOOST_FOREACH(CloudSegment* segment, other->_segments)
    addSegment(segment);

  other->_segments.clear();
}

float CloudBlob::distanceTo(CloudBlob* other) {
  CPoint c = getCentroid(), o = other->getCentroid();
  return CPOINT_DISTANCE(c,o);
}

int CloudBlob::size() { 
  return _segments.size();
}

CPoint CloudBlob::getCentroid() {
  float x = 0, y = 0, z = 0;
  int n = 0;
  BOOST_FOREACH(CloudSegment* s, _segments) {
    x += s->getCentroid().x;
    y += s->getCentroid().y;
    z += s->getCentroid().z;
    n++;
  }
  if(!n) return CPoint(0,0,0);
  return CPoint(x/n,y/n,z/n);
}
float CloudBlob::getWidth() {
  float min = 100, max = -100;
  BOOST_FOREACH(CloudSegment* s, _segments) {
    min = std::min(min, s->minX());
    max = std::max(max, s->maxX());
  }
  return max - min;
}

float CloudBlob::getHeight() {
  float min = 100, max = -100;
  BOOST_FOREACH(CloudSegment* s, _segments) {
    min = std::min(min, s->minY());
    max = std::max(max, s->maxY());
  }
  return max - min;
}

float CloudBlob::getDepth() {
  float min = 100, max = -100;
  BOOST_FOREACH(CloudSegment* s, _segments) {
    min = std::min(min, s->minZ());
    max = std::max(max, s->maxZ());
  }
  return max - min;
}

float CloudBlob::getArea() {
  return getWidth() * getHeight();
}

void CloudBlob::isAdded(bool value) {
  _isAdded = value;
}

bool CloudBlob::isAdded() {
  return _isAdded;
}

void CloudBlob::output() {
  CPoint centroid = getCentroid();
  ROS_INFO("blob (%2.2f,%2.2f,%2.2f) with hwd: (%2.2f, %2.2f, %2.2f), segs: %i",
      centroid.x,
      centroid.y,
      centroid.z,
      getHeight(),
      getWidth(),
      getDepth(),
      size()
      );
}


