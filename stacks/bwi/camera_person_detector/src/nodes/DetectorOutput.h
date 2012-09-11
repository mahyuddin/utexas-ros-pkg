#ifndef DETECTOR_OUTPUT_H
#define DETECTOR_OUTPUT_H

#include <opencv/cv.h>
#include "ekf/PersonReading.h"
#include "ColorSignature.h"

struct DetectorOutput {
  cv::Rect boundingBox;
  cv::Point feetImage;
  PersonReading reading;
  std::vector<ColorSignature> signatures;
};

#endif
