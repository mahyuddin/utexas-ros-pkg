#ifndef PERSON_IDENTIFIER_H
#define PERSON_IDENTIFIER_H

#include <opencv/cv.h>
#include <boost/foreach.hpp>
#include "ColorSignature.h"

#define SIGNATURE_LIFETIME 60.0 // in seconds

class PersonIdentifier {
  private:
    static int _id;
    std::vector<ColorSignature> _signatures;
    void trimOldSignatures();
    /*ColorSignature getMatchingSignature(cv::Mat&, cv::Rect);*/
    ColorSignature getMatchingSignature(cv::Mat&, cv::Mat&, cv::Rect);
  public:
    PersonIdentifier();
    /*int getPersonId(cv::Mat&, cv::Rect);*/
    int getPersonId(cv::Mat&, cv::Mat&, cv::Rect);
    /*int getBestPersonId(cv::Mat&, cv::Rect,std::map<int,bool>&);*/
    int getBestPersonId(cv::Mat&, cv::Mat&, cv::Rect,std::map<int,bool>&);
};

#endif
    
