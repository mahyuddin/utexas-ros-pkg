#ifndef TRANSFORM_PROVIDER_H
#define TRANSFORM_PROVIDER_H

#include <opencv/cv.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <filter/extendedkalmanfilter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>

class TransformProvider {
  private:
    std::string _mapFrameID;
    bool _groundPlaneAvailable;
    tf::StampedTransform _tfCamFromMap; 
    tf::Transform _tfMapFromCam;
    image_geometry::PinholeCameraModel _model;
    tf::Point _groundNormal;
  public:
    TransformProvider();
    TransformProvider(std::string);
    void computeGroundPlane(std::string);
    tf::Point getWorldProjection(cv::Point, float = 0);
    tf::Point getWorldProjection(MatrixWrapper::ColumnVector);
    cv::Point getImageProjection(tf::Point);
    bool isGroundPlaneAvailable();
    void computeModel(const sensor_msgs::CameraInfoConstPtr&);
};

#endif
