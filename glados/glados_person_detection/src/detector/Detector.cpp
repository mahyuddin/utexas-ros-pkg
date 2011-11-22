#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <glados_person_detection/Detector.h>
#include <opencv2/highgui/highgui_c.h>
#include <geometry_msgs/PoseStamped.h>

const unsigned int Detector::SMALL_WIDTH  = 320;
const unsigned int Detector::SMALL_HEIGHT = 240;

static CvSeq* cvHOGDetectMultiScale( const CvArr* image, CvMemStorage* storage,
  const CvArr* svm_classifier=NULL, CvSize win_stride=cvSize(0,0),
  double hit_threshold=0, double scale=1.05,
  int group_threshold=2, CvSize padding=cvSize(0,0),
  CvSize win_size=cvSize(64,128), CvSize block_size=cvSize(16,16),
  CvSize block_stride=cvSize(8,8), CvSize cell_size=cvSize(8,8),
  int nbins=9, int gammaCorrection=1 )
{
    cv::HOGDescriptor hog(win_size, block_size, block_stride, cell_size, nbins, 1, -1, cv::HOGDescriptor::L2Hys, 0.2, gammaCorrection!=0);
    if(win_stride.width == 0 && win_stride.height == 0)
        win_stride = block_stride;
    cv::Mat img = cv::cvarrToMat(image);
    std::vector<cv::Rect> found;
    if(svm_classifier)
    {
        CvMat stub, *m = cvGetMat(svm_classifier, &stub);
        int sz = m->cols*m->rows;
        CV_Assert(CV_IS_MAT_CONT(m->type) && (m->cols == 1 || m->rows == 1) && CV_MAT_TYPE(m->type) == CV_32FC1);
        std::vector<float> w(sz);
        std::copy(m->data.fl, m->data.fl + sz, w.begin());
        hog.setSVMDetector(w);
    }
    else
        hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    hog.detectMultiScale(img, found, hit_threshold, win_stride, padding, scale, group_threshold);
    CvSeq* seq = cvCreateSeq(cv::DataType<cv::Rect>::type, sizeof(CvSeq), sizeof(cv::Rect), storage);
    if(found.size())
        cvSeqPushMulti(seq, &found[0], (int)found.size());
    return seq;
}

Detector::Detector(ros::Publisher *pub, bool doDisplay):
    pub(pub),
    doDisplay(doDisplay),
    storage(cvCreateMemStorage(0)),
    small_img(cvCreateImage(cvSize(SMALL_WIDTH,SMALL_HEIGHT),8,3)),
    fps_count(0),
    fps_freq(5),
    fps_start(ros::Time::now())
{ }

Detector::~Detector() {
  cvReleaseMemStorage(&storage);
  cvReleaseImage(&small_img);
  cvDestroyAllWindows();
}

void Detector::detect(const sensor_msgs::ImageConstPtr &msg,
                      const sensor_msgs::CameraInfoConstPtr &caminfo) {

  model_.fromCameraInfo(caminfo);

  cvClearMemStorage(storage);
  IplImage *img = bridge.imgMsgToCv(msg,"bgr8");

  // resize the image
  cvResize(img,small_img,CV_INTER_LINEAR);
//  cvEqualizeHist(small_img,small_img);
  // run the detector
  CvSeq *found = cvHOGDetectMultiScale(small_img,storage,NULL,cvSize(8,8),0,1.05,2,cvSize(32,32));

  float scale_x = img->width / (float)SMALL_WIDTH;
  float scale_y = img->height / (float)SMALL_HEIGHT;

  for (int i = 0; i < (found ? found->total: 0); i++) {
    CvRect *r = (CvRect*)cvGetSeqElem(found,i);
    r->x += r->width * 0.1;
    r->width *= (1.0 - 0.1 - 0.1);
    r->y += r->height * 0.07;
    r->height *= (1.0 - 0.07 - 0.13);
    publishDetection(r, msg->header.frame_id, scale_x, scale_y);
    if (doDisplay)
      displayDetection(img,r,scale_x,scale_y);
  }

  //CvRect rt;
  //rt.x = 120;
  //rt.y = 60;
  //rt.width=80;
  //rt.height=120;
  //publishDetection(&rt, msg->header.frame_id, scale_x, scale_y);
  //if (doDisplay)
    //displayDetection(img,&rt,scale_x,scale_y);

  if (doDisplay) {
    cvShowImage("result",img);
    cvWaitKey(6);
  }

  fps_count++;
  ros::Time now = ros::Time::now();
  float time_passed = (now - fps_start).toSec();
  if (time_passed > fps_freq) {
    ROS_INFO_STREAM("FPS: " << fps_count / time_passed);
    fps_count = 0;
    fps_start = now;
  }
}
  
void Detector::publishDetection(CvRect *r, std::string camera_frame_id, float scale_x, float scale_y) {

  int x = (r->x + r->width / 2) * scale_x;
  int y = (r->y + r->height) * scale_y;

  ROS_INFO("Person detected at: %i, %i in frame id %s", x, y, camera_frame_id.c_str());

  // Obtain transformation to camera
  tf::TransformListener listener;
  tf::StampedTransform transform_cam_from_map;
  bool transform_found = listener.waitForTransform(camera_frame_id, "/map",
                              ros::Time(), ros::Duration(1.0));
  if (transform_found) {
    try {
      listener.lookupTransform(camera_frame_id, "/map",
                               ros::Time(), transform_cam_from_map);
    } catch (tf::TransformException ex) {
      ROS_INFO("Transform unavailable (Exception): %s", ex.what());
    }
  } else {
    ROS_INFO("Transform unavailable: lookup failed");
  } 

  tf::Transform transform_map_from_cam(transform_cam_from_map.inverse());
  // Obtain ground plane in camera optical frame
  tf::Point o_map(0,0,0);
  tf::Point p_map(1,0,0);
  tf::Point q_map(0,1,0);

  tf::Point o_cam(transform_cam_from_map * o_map);
  tf::Point p_cam(transform_cam_from_map * p_map);
  tf::Point q_cam(transform_cam_from_map * q_map);

  //tf::Point o_map2(transform_map_from_cam * o_cam);
  //tf::Point p_map2(transform_map_from_cam * p_cam);
  //tf::Point q_map2(transform_map_from_cam * q_cam);

  //std::cout << "Transformed Points" << std::endl;
  //std::cout << "  " << o_cam.getX() << "," << o_cam.getY() << "," << o_cam.getZ() << std::endl;
  //std::cout << "  " << p_cam.getX() << "," << p_cam.getY() << "," << p_cam.getZ() << std::endl;
  //std::cout << "  " << q_cam.getX() << "," << q_cam.getY() << "," << q_cam.getZ() << std::endl;

  //std::cout << "Re-Transformed Points" << std::endl;
  //std::cout << "  " << o_map2.getX() << "," << o_map2.getY() << "," << o_map2.getZ() << std::endl;
  //std::cout << "  " << p_map2.getX() << "," << p_map2.getY() << "," << p_map2.getZ() << std::endl;
  //std::cout << "  " << q_map2.getX() << "," << q_map2.getY() << "," << q_map2.getZ() << std::endl;

  // Ground Plane normal
  tf::Point normal_cam = (p_cam - o_cam).cross(q_cam - o_cam);

  // Get ray corresponding to the bottom of the rectangle
  cv::Point2d person_image_point (x,y);
  cv::Point2d rectified_point (model_.rectifyPoint(person_image_point));

  //std::cout << "Rectified Point" << std::endl;
  //std::cout << rectified_point.x << "," << rectified_point.y << std::endl;
  
  cv::Point3d ray = model_.projectPixelTo3dRay(rectified_point);
  
  tf::Point ray_1(0, 0, 0);
  tf::Point ray_2(ray.x, ray.y, ray.z);

  //std::cout << "Person point ray" << std::endl;
  //std::cout << "  " << ray_2.getX() << "," << ray_2.getY() << "," << ray_2.getZ() << std::endl;

  // Obtain the person point on the ground plane
  float t = (o_cam - ray_1).dot(normal_cam) / (ray_2 - ray_1).dot(normal_cam);
  tf::Point person_point_cam = ray_1 + t * (ray_2 - ray_1);

  //std::cout << "Person point cam" << std::endl;
  //std::cout << "  " << person_point_cam.getX() << "," << person_point_cam.getY() << "," << person_point_cam.getZ() << std::endl;

  // Finally obtain this point in map frame, and publish
  tf::Point person_point_map = transform_map_from_cam * person_point_cam;

  //std::cout << "Person point map" << std::endl;
  //std::cout << "  " << person_point_map.getX() << "," << person_point_map.getY() << "," << person_point_map.getZ() << std::endl;

  geometry_msgs::PoseStamped person_pose;
  person_pose.header.stamp = ros::Time::now();
  person_pose.header.frame_id = "map";
  person_pose.pose.position.x = person_point_map.getX();
  person_pose.pose.position.y = person_point_map.getY();
  person_pose.pose.position.z = person_point_map.getZ();
  person_pose.pose.orientation.x = 0;
  person_pose.pose.orientation.y = 0;
  person_pose.pose.orientation.z = 0;
  person_pose.pose.orientation.w = 1;

  pub->publish(person_pose);
}

void Detector::displayDetection(IplImage *img, CvRect *r, float scale_x, float scale_y) {
  CvPoint pt1;
  CvPoint pt2;
  
  pt1.x = r->x * scale_x;
  pt1.y = r->y * scale_y;
  pt2.x = (r->x + r->width) * scale_x;
  pt2.y = (r->y + r->height) * scale_y;
  cvRectangle(img,pt1,pt2,CV_RGB(0,255,0),3,8,0);
}
