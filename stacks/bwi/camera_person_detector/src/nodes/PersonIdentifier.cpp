#include "PersonIdentifier.h"

PersonIdentifier::PersonIdentifier() {
}

int PersonIdentifier::getPersonId(cv::Mat& image, cv::Rect detection) {
  trimOldSignatures();
  ColorSignature signature = getMatchingSignature(image,detection);
  return signature.getId();
}

int PersonIdentifier::getBestPersonId(cv::Mat& image, cv::Rect detection, std::map<int,bool>& found) {
  ColorSignature test(image,detection);
  ColorSignature* best = 0;
  double distance = 100000;
  BOOST_FOREACH(ColorSignature& signature, _signatures) {
    if(found.find(signature.getId()) != found.end()) continue;
    double d = signature.distance(test);
    if(d < distance) {
      distance = d;
      best = &signature;
    }
  }
  if(best) return best->getId();
  return 0;
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
