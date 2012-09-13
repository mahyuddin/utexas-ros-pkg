#ifndef POINT_H
#define POINT_H

#include <list>
#include <opencv/cv.h>

namespace sp {

  class Point : public cv::Point { 
    public:
      Point();
      Point(int,int);
      Point(Point*);
      bool operator==(const Point& right);
      bool operator!=(const Point& right);
      int getDeterminant(Point&, Point&);
      static bool onSameLine(std::list<Point*>*);
  };

}

#endif
