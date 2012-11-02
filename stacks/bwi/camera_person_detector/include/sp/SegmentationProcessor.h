#ifndef SEGMENTATION_PROCESSOR_H
#define SEGMENTATION_PROCESSOR_H
#include <list>
#include <map>
#include <opencv/cv.h>
#include "boost/foreach.hpp"

#include "Blob.h"
#include "Run.h"
#include "Constants.h"

namespace sp {

  class SegmentationProcessor {
    private:
      Run*** rleMap;
      int frameNumber;
      cv::Mat _image;
      int _rows, _cols;
    protected:
      Run* generateRun(int,int);
      void completeRun(Run*,int);
      void constructRleMap();
      void mergeOverlaps();
      void initializeRleMap(int,int);
      void resetRleMap();
      void printSegmentationArray();
      std::vector<Blob*> mergeOverlappedBlobs(std::vector<Blob*>&);
    public:
      std::vector<Blob*> constructBlobs(cv::Mat&);
      SegmentationProcessor();
      ~SegmentationProcessor();
  };

}
#endif
