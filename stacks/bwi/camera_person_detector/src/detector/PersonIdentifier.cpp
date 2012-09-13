#include "PersonIdentifier.h"

PersonIdentifier::PersonIdentifier() {
}

int PersonIdentifier::getPersonId(cv::Mat& image, cv::Mat& mask, cv::Rect detection) {
  trimOldSignatures();
  ColorSignature signature = getMatchingSignature(image, mask, detection);
  return signature.getId();
}

int PersonIdentifier::getBestPersonId(cv::Mat& image, cv::Mat& mask, cv::Rect detection, std::map<int,bool>& found) {
  ColorSignature test(image,mask,detection);
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

ColorSignature PersonIdentifier::getMatchingSignature(cv::Mat& image, cv::Mat& mask, cv::Rect detection) {
  ColorSignature test(image,mask,detection);
  BOOST_FOREACH(ColorSignature& signature, _signatures) {
    if(signature == test) {
      signature.update(test);
      return signature;
    }
  }
  _signatures.push_back(test);
  return test;
}

std::vector<ColorSignature> PersonIdentifier::getSignaturesById(int id) {
  std::vector<ColorSignature> signatures;
  BOOST_FOREACH(ColorSignature& signature, _signatures) {
    if(signature.getId() == id)
      signatures.push_back(signature);
  }
  return signatures;
}

void PersonIdentifier::trimOldSignatures() {
  std::vector<ColorSignature> trimmed;
  BOOST_FOREACH(ColorSignature& signature, _signatures) {
    if(signature.getStamp().toSec() > (ros::Time::now() - ros::Time(SIGNATURE_LIFETIME)).toSec())
      trimmed.push_back(signature);
  }
  _signatures = trimmed;
}
