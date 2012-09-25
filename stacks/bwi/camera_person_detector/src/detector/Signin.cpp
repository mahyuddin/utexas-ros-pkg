#include "Signin.h"

Signin::Signin(ros::NodeHandle& nh, ros::NodeHandle& nh_param) : _nh(nh) {
  nh_param.param("camera", _camera, std::string("camera1"));
  _collect = false;
  attachToCamera(_camera);
  _publisher = _nh.advertise<bwi_msgs::PersonDescriptor>("/bwi/registered_persons", 1000);
}

void Signin::collect() {
  _collect = true;
  _signatures = std::vector<bwi_msgs::ColorSignature>();
}

void Signin::stop() {
  _collect = false;
  bwi_msgs::PersonDescriptor descriptor;
  BOOST_FOREACH(const bwi_msgs::ColorSignature& signature, _signatures) {
    descriptor.signatures.push_back(signature);
  }
  descriptor.id = _identifier.generateGuid();
  _publisher.publish(descriptor);
}

void Signin::attachToCamera(std::string camera) {
  _camera = camera;
  _subscriber.shutdown();
  _subscriber = _nh.subscribe("/bwi/person_detections/" + _camera, 1000, &Signin::processDetections, this);
}

void Signin::processDetections(const bwi_msgs::PersonDetectionArray& detections) {
  if(!_collect) return;
  BOOST_FOREACH(const bwi_msgs::PersonDetection& detection, detections.detections) {
    _signatures.push_back(detection.signature);
  }
}



/*
 * 1. client app enables signature collection
 * 2. signin collects signatures from detections at the current camera
 * 3. client app completes signature collection
 * 4. signin requests a guid from the person identifier
 * 5. signin associates guid and signatures w/ PersonDescriptor, broadcasts descriptor to registered_persons
 * 6. detector nodes subscribed to registered_people
 * 7. detector nodes add in registered descriptor
 * 8. detectors attempt to match against this descriptor and use its guid when a match is made
 * 9. this is carried out w/ PersonIdentifier
 *
 */ 
