#include "Point.h"

using namespace sp;

Point::Point() : cv::Point(0,0) {}

Point::Point(int x, int y) : cv::Point(x,y) {}

Point::Point(Point* p) {x = p->x; y = p->y; }

bool Point::operator==(const Point& right) {
  return this->x == right.x && this->y == right.y;
}

bool Point::operator!=(const Point& right) {
  return this->x != right.x || this->y != right.y;
}

int Point::getDeterminant(Point& a, Point& b) {
  int determinant = (b.x - a.x) * (this->y - a.y) - (b.y - a.y) * (this->x - a.x);
  return determinant;
}

bool Point::onSameLine(std::list<Point*>* points) {
  if(points->size() <= 2) return true;
  Point *p = points->front(), *a = 0, *b = 0;
  int skip = 2;
  for(std::list<Point*>::iterator it = points->begin(); it != points->end(); it++) {
    if(skip == 2) { skip--; continue; }
    if(skip == 1) { skip--; a = *it; continue; }
    b = *it;
    if(p->getDeterminant(*a,*b) != 0) return false;
    a = b;
  }
  return true;
}
    
  
