#include "PersonIdentifier.h"

#define SIGNATURE_MATCH_DISTANCE 5

PersonIdentifier::PersonIdentifier() {
}

GUID PersonIdentifier::registerSignature(cv::Mat& image, cv::Mat& mask, cv::Rect detection) {
  trimOldSignatures();
  ColorSignature signature = getMatchingSignature(image, mask, detection);
  return signature.getId();
}

ColorSignature PersonIdentifier::getMatchingSignature(cv::Mat& image, cv::Mat& mask, cv::Rect detection) {
  ColorSignature test(image,mask,detection,generateGuid());
  BOOST_FOREACH(ColorSignature& signature, _signatures) {
    if(test.distanceTo(signature) <= SIGNATURE_MATCH_DISTANCE) {
      signature.update(test);
      return signature;
    }
  }
  _signatures.push_back(test);
  return test;
}

GUID PersonIdentifier::getSignatureId(const ColorSignature& signature, double& distance) {
  distance = SIGNATURE_MATCH_DISTANCE;
  const ColorSignature* best = 0;
  BOOST_FOREACH(ColorSignature& s, _signatures) {
    double d = s.distanceTo(signature);
    if(d <= distance) {
      distance = d;
      best = &s;
    }
  }
  if(best) return best->getId();
  return 0;
}

ColorSignature PersonIdentifier::generateSignature(cv::Mat& image, cv::Mat& mask, cv::Rect detection, GUID guid) {
  ColorSignature signature(image,mask,detection,generateGuid());
  return signature;
}

GUID PersonIdentifier::generateGuid() {
  static Random generator(SEED);
  return generator();
}

bwi_msgs::ColorSignature PersonIdentifier::getSignatureById(int id) {
  BOOST_FOREACH(ColorSignature& signature, _signatures) {
    if(signature.getId() == id)
      return signature.getMsg();
  }
  ROS_ERROR("invalid signature requested");
  throw -1;
}

void PersonIdentifier::trimOldSignatures() {
  std::vector<ColorSignature> trimmed;
  BOOST_FOREACH(ColorSignature& signature, _signatures) {
    if(signature.getStamp().toSec() > (ros::Time::now() - ros::Time(SIGNATURE_LIFETIME)).toSec())
      trimmed.push_back(signature);
  }
  _signatures = trimmed;
}

void PersonIdentifier::registerDescriptor(const bwi_msgs::PersonDescriptor& descriptor) {
  BOOST_FOREACH(const bwi_msgs::ColorSignature& signature, descriptor.signatures) {
    _signatures.push_back(ColorSignature(signature, descriptor.id));
  }
}
