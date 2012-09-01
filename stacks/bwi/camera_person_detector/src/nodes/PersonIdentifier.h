#ifndef PERSON_IDENTIFIER_H
#define PERSON_IDENTIFIER_H

#include <opencv/cv.h>
#include <boost/foreach.hpp>
#include "ColorSignature.h"

#define SIGNATURE_LIFETIME 10.0 // 1 minute

class PersonIdentifier {
  private:
    static int _id;
    std::vector<ColorSignature> _signatures;
    void trimOldSignatures();
    ColorSignature getMatchingSignature(cv::Mat&, cv::Rect);
  public:
    PersonIdentifier();
    int getPersonId(cv::Mat&,cv::Rect);
};

#endif
    
