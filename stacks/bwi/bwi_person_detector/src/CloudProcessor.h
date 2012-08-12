#ifndef CLOUD_PROCESSOR_H
#define CLOUD_PROCESSOR_H

#include "CloudSegment.h"
#include "CloudBlob.h"
#include "CloudCluster.h"

#include <boost/foreach.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

#include "TypeDefs.h"
#include "CloudMacros.h"

class CloudProcessor {
  private:
    float _minX, _maxX, _minY, _maxY, _minZ, _maxZ;
    CloudPtr _cloud;
    Cloud::Ptr _cloudFiltered;
    CloudSegment** _scanMap;
  public:
    CloudProcessor();
    enum CDirection { Horizontal, Vertical };
    
    void setCloud(const CloudPtr&);
    const CloudPtr& getCloud();
    void setBoundingBox(float,float,float,float,float,float);  
    void processSegments();
    void pruneScanMap(CloudSegment**);
    std::vector<CloudSegment*> constructSegments();
    std::vector<CloudBlob*> constructBlobs(CloudSegment**);
    std::vector<CloudBlob*> mergeBlobs(std::vector<CloudBlob*> blobs);
    bool getLegs(CPoint&,CPoint&);
    bool getCentroid(CPoint&);
};
#endif
