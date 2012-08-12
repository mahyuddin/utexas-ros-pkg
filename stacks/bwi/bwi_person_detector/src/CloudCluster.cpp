#include "CloudCluster.h"

void CloudCluster::computeStatistics() {
  float x = 0.0, y = 0.0, z = 0.0;
  unsigned int n = 0;
  float minX = 100, maxX = -100, minY = 100, maxY = -100, minZ = 100, maxZ = -100;
  BOOST_FOREACH (const CPoint& pt, *this) {
    if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
      x += pt.x;
      y += pt.y;
      z += pt.z;
      n++;
      minX = std::min(x,minX); maxX = std::max(x,maxX);
      minY = std::min(y,minY); maxY = std::max(y,maxY);
      minZ = std::min(z,minZ); maxZ = std::max(z,maxZ);
    }
  }
  if (n) {
    x /= n;
    y /= n;
    z /= n;
  }
  _centroid = CPoint(x / n,y / n,z / n);
  _width = fabs(maxX - minX);
  _height = fabs(maxY - minY);
  _depth = fabs(maxZ - minZ);
}

CPoint CloudCluster::getCentroid() {
  return _centroid;
}

float CloudCluster::getWidth() {
  return _width;
}

float CloudCluster::getHeight() { 
  return _height;
}

float CloudCluster::getDepth() {
  return _depth;
}

