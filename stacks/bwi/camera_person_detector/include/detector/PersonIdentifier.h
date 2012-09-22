#ifndef PERSON_IDENTIFIER_H
#define PERSON_IDENTIFIER_H

#include <opencv/cv.h>
#include <boost/foreach.hpp>
#include <ColorSignature.h>
#include <bwi_msgs/ColorSignature.h>
#include <bwi_msgs/PersonDetection.h>

#include "DetectorTypeDefs.h"

#define SIGNATURE_LIFETIME 60.0 // in seconds
typedef boost::mt19937 Random;
#define SEED static_cast<GUID>(std::time(0))


class PersonIdentifier {
  private:
    std::vector<ColorSignature> _signatures;
    void trimOldSignatures();
    ColorSignature getMatchingSignature(cv::Mat&, cv::Mat&, cv::Rect);
    ColorSignature generateSignature(cv::Mat&, cv::Mat&, cv::Rect, GUID);
  public:
    PersonIdentifier();
    int getPersonId(cv::Mat&, cv::Mat&, cv::Rect);
    std::vector<bwi_msgs::ColorSignature> getSignaturesById(int);
    GUID generateGuid();
    ColorSignature generateSignature();
    void registerSignatures(const bwi_msgs::PersonDetection&);
};

#endif
    
