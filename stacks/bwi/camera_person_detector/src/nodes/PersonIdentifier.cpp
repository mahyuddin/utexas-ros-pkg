#include "PersonIdentifier.h"

PersonIdentifier::PersonIdentifier() {
}

int PersonIdentifier::getPersonId(cv::Mat& image, cv::Rect detection) {
  trimOldSignatures();
  ColorSignature signature = getMatchingSignature(image,detection);
  return signature.getId();
}

ColorSignature PersonIdentifier::getMatchingSignature(cv::Mat& image, cv::Rect detection) {
  ColorSignature test(image,detection);
  BOOST_FOREACH(ColorSignature& signature, _signatures) {
    if(signature == test) {
      signature.update(test);
      return signature;
    }
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
