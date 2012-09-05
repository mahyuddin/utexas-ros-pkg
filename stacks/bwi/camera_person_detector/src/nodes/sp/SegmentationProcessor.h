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
      Run* rleMap[SEG_IMAGE_WIDTH][SEG_IMAGE_HEIGHT];
      int frameNumber;
      cv::Mat _image;
    protected:
      Run* generateRun(int,int);
      void completeRun(Run*,int);
      void constructRleMap();
      void mergeOverlaps();
      void resetRleMap();
      void initializeRleMap();
      void printSegmentationArray();
      std::vector<Blob*> mergeOverlappedBlobs(std::vector<Blob*>&);
    public:
      std::vector<Blob*> constructBlobs(cv::Mat&);
      SegmentationProcessor();
      ~SegmentationProcessor();
  };

}
#endif
