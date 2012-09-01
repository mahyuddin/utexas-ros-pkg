#include "PersonIdentifier.h"

PersonIdentifier::PersonIdentifier() {
}

int PersonIdentifier::getPersonId(cv::Mat& image, cv::Rect detection) {
  ColorSignature signature = getMatchingSignature(image,detection);
  return signature.getId();
}

ColorSignature PersonIdentifier::getMatchingSignature(cv::Mat& image, cv::Rect detection) {
  ColorSignature test(image,detection);
  BOOST_FOREACH(ColorSignature& signature, _signatures) {
    if(signature == test)
      return signature;
  }
  _signatures.push_back(test);
  return test;
}

void PersonIdentifier::trimOldSignatures() {
  std::vector<ColorSignature> trimmed;
  BOOST_FOREACH(ColorSignature& signature, _signatures) {
    if(signature.getStamp().toSec() > (ros::Time::now() - ros::Time(SIGNATURE_LIFETIME)).toSec())
      trimmed.push_back(signature);
  }
  _signatures = trimmed;
}

// see a person
// trim old signatures
// get matching signature
// search filters for signature id
// if exists, update filter
// otherwise, new filter w/ new id
// trim filters with old timestamps or high covariance
