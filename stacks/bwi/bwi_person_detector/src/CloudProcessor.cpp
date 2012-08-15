#include "CloudProcessor.h"

#define POINT_DISTANCE_THRESHOLD .05
#define SEGMENT_DISTANCE_THRESHOLD .1
#define BLOB_DISTANCE_THRESHOLD .2
#define MINIMUM_SEGMENT_POINTS 3
#define MINIMUM_BLOB_SEGMENTS 100
#define MINIMUM_BLOB_HEIGHT .4
#define MINIMUM_BLOB_WIDTH .08
#define MAXIMUM_BLOB_HEIGHT 1
#define MAXIMUM_BLOB_WIDTH .55
#define SCAN_WIDTH 640
#define SCAN_HEIGHT 480
#define SCAN_SIZE (SCAN_WIDTH * SCAN_HEIGHT)

CloudProcessor::CloudProcessor() : _cloudFiltered(new Cloud()) {
}

void CloudProcessor::setCloud(const CloudPtr& cloud) {
  _cloud = cloud;
}

const CloudPtr& CloudProcessor::getCloud() {
  return _cloud;
}

void CloudProcessor::setBoundingBox(float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {
  _minX = minX; _maxX = maxX;
  _minY = minY; _maxY = maxY;
  _minZ = minZ; _maxZ = maxZ;
}

// Here we construct horizontal segments of consecutive points. The distance
// between each consecutive pair is at most DISTANCE_THRESHOLD. 
std::vector<CloudSegment*> CloudProcessor::constructSegments() {
  _scanMap = new CloudSegment*[SCAN_SIZE];
  memset(_scanMap, 0, SCAN_SIZE * sizeof(CloudSegment*));
  std::vector<CloudSegment*> segments;
  for(int j = 0; j <  SCAN_HEIGHT; j++) {
    CloudSegment* segment = new CloudSegment();
    segment->setLeft(0);
    for(int i = 0; i < SCAN_WIDTH - 1; i++) {
      CPoint pt = _cloud->points[j * SCAN_WIDTH + i], next;
      if(!KEEP_POINT(pt)) continue;
      while(true) {
        segment->addPoint(pt);
        _scanMap[j  * SCAN_WIDTH + i] = segment;
        if(i == SCAN_WIDTH - 1) break;
        next = _cloud->points[j * SCAN_WIDTH + i + 1];
        if(!KEEP_POINT(next)) break;
        pt = next;
        if(CPOINT_DISTANCE(pt,next) > POINT_DISTANCE_THRESHOLD) break;
        i++;
      }
      segment->computeStatistics();
      segment->setRight(i);
      segments.push_back(segment);
      segment = new CloudSegment();
      segment->setLeft(i + 1);
    }
    segment->computeStatistics();
    segment->setRight(SCAN_WIDTH - 1);
    segments.push_back(segment);
  }
  return segments;
}

void CloudProcessor::pruneScanMap(CloudSegment** scanMap) {
  for(int i = 0; i < SCAN_SIZE; i++) 
    if(scanMap[i] && scanMap[i]->size() < MINIMUM_SEGMENT_POINTS)
      scanMap[i] = 0;
}

// Here we merge the horizontal segments into blobs by moving down the _scanMap vertically. 
// Each pair of consecutive segments is merged together if their centroids are
// at most DISTANCE_THRESHOLD apart.
std::vector <CloudBlob*> CloudProcessor::constructBlobs(CloudSegment** scanMap) {
  std::vector<CloudBlob*> blobs;
  CloudBlob* blob;
  int id = 0;
  std::set<int> theset;
  for(int i = 0; i < SCAN_WIDTH; i++) {
    blob = new CloudBlob();
    for(int j = 0; j < SCAN_HEIGHT - 1;j++) {
      CloudSegment 
        *top = scanMap[j * SCAN_WIDTH + i],
        *bottom = scanMap[(j + 1) * SCAN_WIDTH + i];          
      if(!top) continue;
      if(!top->getParent()) {
        blob->addSegment(top);
      }
      if(!bottom || top->distanceTo(*bottom) > SEGMENT_DISTANCE_THRESHOLD) {
        if(blob->size() == 0)
          delete blob;
        else if (!blob->isAdded()) {
          blobs.push_back(blob);
          blob->isAdded(true);
        }
        blob = new CloudBlob();
        continue;
      }
      if(bottom->getParent() && bottom->getParent() != blob) {
        bottom->getParent()->merge(blob);
        if(!blob->isAdded()) {
          delete blob;
        }
        blob = bottom->getParent();
      } else if (!bottom->getParent()) {
        blob->addSegment(bottom);
      }
    }
    if(blob->size() == 0)
      delete blob;
    else if(!blob->isAdded()) {
      blobs.push_back(blob);
      blob->isAdded(true);
    }
  }
  return blobs;
}

std::vector<CloudBlob*> CloudProcessor::mergeBlobs(std::vector<CloudBlob*> blobs) {
  std::vector<CloudBlob*> filtered;
  std::set<CloudBlob*> merged, removed;
  BOOST_FOREACH(CloudBlob* o, blobs) {
    BOOST_FOREACH(CloudBlob* i, blobs) {
      if(o == i) continue;
      if(removed.find(o) != removed.end()) continue; // outer was already merged in
      if(removed.find(i) != removed.end()) continue; // inner was already merged in
      if(merged.find(i) != merged.end()) continue; // inner is already a merge blob
      if(o->distanceTo(i) > BLOB_DISTANCE_THRESHOLD) continue;
        o->merge(i);
        merged.insert(o);
        removed.insert(i);
    }
  }
  BOOST_FOREACH(CloudBlob* blob, merged)
    if(
        blob->size() > MINIMUM_BLOB_SEGMENTS &&
        blob->getHeight() >= MINIMUM_BLOB_HEIGHT &&
        blob->getHeight() <= MAXIMUM_BLOB_HEIGHT &&
        blob->getWidth() >= MINIMUM_BLOB_WIDTH &&
        blob->getWidth() <= MAXIMUM_BLOB_WIDTH
      )
      filtered.push_back(blob);
  //else if (blob->size() > MINIMUM_BLOB_SEGMENTS) { 
    //if(blob->getHeight() < MINIMUM_BLOB_HEIGHT)
      //{ ROS_INFO("threw out blob for height: "); blob->output(); }
    //if(blob->getWidth() < MINIMUM_BLOB_WIDTH)
      //{ ROS_INFO("threw out blob for width: "); blob->output(); }
  //}

  return filtered;
}

std::vector<CloudBlob*> CloudProcessor::processSegments() {
  BOOST_FOREACH(CloudSegment* segment, _segments) {
    delete segment;
  }
  BOOST_FOREACH(CloudBlob* blob, _blobs) {
    //ROS_INFO("deleting %i", blob->id);
    delete blob;
  }
  //ROS_INFO("processing segments");
  _segments = constructSegments();
  //ROS_INFO("%i segments, pruning scan map", segments.size());
  pruneScanMap(_scanMap);
  //ROS_INFO("constructing blobs");
  _blobs = constructBlobs(_scanMap);
  //ROS_INFO("%i blobs, merging", blobs.size());
  std::vector<CloudBlob*> merged = mergeBlobs(_blobs);
  //ROS_INFO("%i merged", merged.size());
  //ROS_INFO("clearing memory");
  delete [] _scanMap;
  //ROS_INFO("complete");
  //BOOST_FOREACH(CloudBlob* blob, merged)
    //ROS_INFO("returning %i", blob->id);
  return merged;
}

bool CloudProcessor::getLegs(CPoint& left, CPoint& right) {
  return false;
}

bool CloudProcessor::getCentroid(CPoint& centroid) {
  float x = 0.0, y = 0.0, z = 0.0;
  unsigned int n = 0;
  BOOST_FOREACH (const CPoint& pt, _cloud->points) {
    if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
      if (
          pt.x > _minX && pt.x < _maxX &&
          pt.y > _minY && pt.y < _maxY &&
          pt.z > _minZ && pt.z < _maxZ
        ) {
        x += pt.x;
        y += pt.y;
        z += pt.z;
        n++;
      }
    }
  }
  if (n) {
    x /= n;
    y /= n;
    z /= n;
  }
  centroid = CPoint(x,y,z);
  return n;
}
