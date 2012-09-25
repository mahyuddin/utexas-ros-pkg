#include "PersonIdentifier.h"

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
    if(test == signature) {
      signature.update(test);
      return signature;
    }
  }
  _signatures.push_back(test);
  return test;
}

GUID PersonIdentifier::getSignatureId(const ColorSignature& signature) {
  BOOST_FOREACH(ColorSignature& s, _signatures) {
    if(s == signature) {
      return s.getId();
    }
  }
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
