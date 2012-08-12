#ifndef CLOUD_SEGMENT_H
#define CLOUD_SEGMENT_H

#include <pcl/point_types.h>
#include "TypeDefs.h"
#include "CloudMacros.h"

class CloudBlob;

class CloudSegment {
  private:
    CloudBlob* _parent;
    int _left, _right;
    float _minX, _maxX, _minY, _maxY, _minZ, _maxZ;
    CPoint _centroid;
    std::vector<CPoint> _points;
  public:
    CloudSegment();
    void addPoint(const CPoint&);
    float distanceTo(CloudSegment&);
    void setParent(CloudBlob*);
    CloudBlob* getParent();
    int size();
    int getLeft();
    int getRight();
    void setLeft(int);
    void setRight(int);
    void computeStatistics();
    float minX();
    float maxX();
    float minY();
    float maxY();
    float minZ();
    float maxZ();
    CPoint getCentroid();
};
#endif
