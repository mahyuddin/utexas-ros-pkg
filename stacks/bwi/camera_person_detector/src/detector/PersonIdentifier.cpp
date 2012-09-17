#include "PersonIdentifier.h"

PersonIdentifier::PersonIdentifier() {
}

int PersonIdentifier::getPersonId(cv::Mat& image, cv::Mat& mask, cv::Rect detection) {
  trimOldSignatures();
  ColorSignature signature = getMatchingSignature(image, mask, detection);
  return signature.getId();
}

ColorSignature PersonIdentifier::getMatchingSignature(cv::Mat& image, cv::Mat& mask, cv::Rect detection) {
  ColorSignature test(image,mask,detection,generateGuid());
  BOOST_FOREACH(ColorSignature& signature, _signatures) {
    if(signature == test) {
      signature.update(test);
      return signature;
    }
  }
  _signatures.push_back(test);
  return test;
}

ColorSignature PersonIdentifier::generateSignature(cv::Mat& image, cv::Mat& mask, cv::Rect detection, GUID guid) {
  ColorSignature signature(image,mask,detection,generateGuid());
  return signature;
}

GUID PersonIdentifier::generateGuid() {
  static Random generator(SEED);
  return generator();
}

std::vector<bwi_msgs::ColorSignature> PersonIdentifier::getSignaturesById(int id) {
  std::vector<bwi_msgs::ColorSignature> signatures;
  BOOST_FOREACH(ColorSignature& signature, _signatures) {
    if(signature.getId() == id)
      signatures.push_back(signature.getMsg());
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

void PersonIdentifier::registerSignatures(const bwi_msgs::PersonDetection& detection) {
  BOOST_FOREACH(const bwi_msgs::ColorSignature& signature, detection.signatures) {
    _signatures.push_back(ColorSignature(signature, detection.id));
  }
}
