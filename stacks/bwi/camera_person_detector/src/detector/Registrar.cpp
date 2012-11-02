#include "Registrar.h"

Registrar::Registrar(ros::NodeHandle& nh, ros::NodeHandle& nh_param) : _nh(nh) {
  nh_param.param("camname", _camera, std::string("camera1"));
  _collect = false;
  setCamera(_camera);
  _publisher = _nh.advertise<bwi_msgs::PersonDescriptor>("/global/registered_persons", 1000);
  _collectService = nh.advertiseService(_camera + "/register_start", &Registrar::register_start, this);
  _stopService = nh.advertiseService(_camera + "/register_stop", &Registrar::register_stop, this);
}

void Registrar::start() {
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response resp;
  register_start(req, resp);
}

bool Registrar::register_start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  _collect = true;
  _signatures = std::vector<bwi_msgs::ColorSignature>();
  return true;
}

void Registrar::stop() {
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response resp;
  register_stop(req, resp);
}

bool Registrar::register_stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  _collect = false;
  bwi_msgs::PersonDescriptor descriptor;
  BOOST_FOREACH(const bwi_msgs::ColorSignature& signature, _signatures) {
    descriptor.signatures.push_back(signature);
  }
  descriptor.id = _identifier.generateGuid();
  _publisher.publish(descriptor);
  return true;
}

void Registrar::setCamera(std::string camera) {
  _camera = camera;
  _subscriber.shutdown();
  _subscriber = _nh.subscribe(_camera + "/person_detections", 1000, &Registrar::processDetections, this);
}

void Registrar::processDetections(const bwi_msgs::PersonDetectionArray& detections) {
  if(!_collect) return;
  BOOST_FOREACH(const bwi_msgs::PersonDetection& detection, detections.detections) {
    _signatures.push_back(detection.signature);
  }
}
