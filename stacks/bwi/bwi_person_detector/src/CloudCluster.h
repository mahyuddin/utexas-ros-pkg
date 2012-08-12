#ifndef CLOUD_CLUSTER_H
#define CLOUD_CLUSTER_H

#include <pcl/point_types.h>
#include <vector>
#include "TypeDefs.h"
#include <boost/foreach.hpp>


class CloudCluster : public std::vector<CPoint> {
  private:
    CPoint _centroid;
    float _width, _height, _depth;

  public:
    CPoint getCentroid();
    float getWidth();  
    float getHeight();
    float getDepth();
    void computeStatistics();
};

#endif

