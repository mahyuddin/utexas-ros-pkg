#include "Blob.h"

using namespace sp;

int Blob::_id = 0;

bool Blob::pointMap[MAX_IMAGE_WIDTH][MAX_IMAGE_HEIGHT];

Blob::Blob() { 
  id = _id++;
  colorID = 0; 
  topLeft = Point(0,0);
  topRight = Point(0,0);
  bottomLeft = Point(0,0);
  bottomRight = Point(0,0);
}

Blob::~Blob() {
  while(!points.empty()) delete points.front(), points.pop_front();
}

void Blob::addPoint(Point* p) {
  points.push_back(new Point(p));
}

void Blob::addRun(Run* r) {
  addPoint(&(r->left));
  if(r->left != r->right)
  addPoint(&(r->right));
}

bool Blob::isLeftOf(Point& p, Point& lineStart, Point& lineEnd) {
  int determinant = p.getDeterminant(lineStart,lineEnd);
  if (determinant < 0)
    return true;
  return false;
}

double Blob::getArea() {
  double area = 0;
  for(unsigned int i=2;i<vertices.size();i++) {
    Point* a = vertices[0];
    Point* b = vertices[i - 1];
    Point* c = vertices[i];
    double subArea = a->x * (b->y - c->y) + b->x * (c->y - a->y) + c->x * (a->y - b->y);
    subArea /= 2;
    area += fabs(subArea);
  }
  return area;
}

void Blob::build() {
  clearPointMap();
  removeDuplicatePoints();
  buildHull();
  buildCentroid();
  buildRadius();
  trimExtremes();
  buildHull();
  buildCentroid();
  buildRadius();
}

void Blob::clearPointMap() {
  memset(pointMap, false, MAX_IMAGE_WIDTH * MAX_IMAGE_HEIGHT * sizeof(bool));
}

void Blob::removeDuplicatePoints() {
  std::list<Point*> pruned;
  for(std::list<Point*>::iterator it = points.begin(); it != points.end(); it++) {
    if(!pointMap[(*it)->x][(*it)->y])
      pruned.push_back(*it);
    pointMap[(*it)->x][(*it)->y] = true;
  }
  points = std::list<Point*>(pruned);
}

void Blob::trimExtremes() {
  std::list<Point*> pruned;
  for(std::list<Point*>::iterator it = points.begin(); it != points.end(); it++)
    if(distance(**it,centroid) <= getRadius() * 1.2)
      pruned.push_back(*it);
  points = std::list<Point*>(pruned);
}

void Blob::buildHull() {
  vertices.clear();
  Point *left = points.front(), *right = points.front(), *top = points.front(), *bottom = points.front();
  for(std::list<Point*>::iterator it = points.begin(); it != points.end(); it++) {
    if((*it)->x < left->x) 
      left = *it;
    if((*it)->x > right->x)
      right = *it;
    if((*it)->y > top->y)
      top = *it;
    if((*it)->y < bottom->y)
      bottom = *it;
  }
  topLeft.y = top->y;
  topLeft.x = left->x;
  topRight.y = top->y;
  topRight.x = right->x;
  bottomLeft.y = bottom->y;
  bottomLeft.x = left->x;
  bottomRight.y = bottom->y;
  bottomRight.x = right->x;
  if(Point::onSameLine(&points)) {
    vertices.push_back(left);
    vertices.push_back(right);
    return;
  }
  Point* pointOnHull = left;
  Point* endpoint = 0;
  int i = 0;
  do {
    i++;
    vertices.push_back(pointOnHull);
    endpoint = points.front();
    for(std::list<Point*>::iterator it = points.begin(); it != points.end(); it++) {
      if(*it == points.front() || (*endpoint == (Point*)*it)) continue;
      if(*endpoint == *pointOnHull || isLeftOf(**it,*pointOnHull,*endpoint)) {
        endpoint = *it;
      }
    }
    pointOnHull = endpoint;
  } while(*endpoint != *vertices[0]);
}

void Blob::buildCentroid() {
  centroid.x = 0;
  centroid.y = 0;
  double totalArea = 0.0;
  int size = vertices.size();
  if(size == 2) {
    centroid.x = (vertices[0]->x + vertices[1]->x) / 2;
    centroid.y = (vertices[0]->y + vertices[1]->y) / 2;
    return;
  }
  for(int i=0; i<size; i++) {
    int x0,x1,y0,y1;
    x0 = vertices[i]->x;
    y0 = vertices[i]->y;
    
    if(i == size - 1) {
      x1 = vertices[0]->x;
      y1 = vertices[0]->y;
    }
    else {
      x1 = vertices[i+1]->x;
      y1 = vertices[i+1]->y;
    }
    int a = x0*y1 - x1*y0;
    totalArea += a;
    centroid.x += (x0 + x1)*a;
    centroid.y += (y0 + y1)*a;
  }
  totalArea /= 2;
  if(fabs(totalArea) >= 0.5) {
    centroid.x /= (6*totalArea);
    centroid.y /= (6*totalArea);
  }
}

double Blob::distance(Point a, Point b) {
  return sqrt(pow((double)a.x - b.x, 2) + pow((double)a.y - b.y, 2));
}

Blob* Blob::merge(Blob* a, Blob* b) {
  Blob* merged = new Blob();
  merged->colorID = a->colorID;
  for(std::list<Point*>::iterator it = a->points.begin(); it != a->points.end(); it++)
    merged->addPoint(*it);
  for(std::list<Point*>::iterator it = b->points.begin(); it != b->points.end(); it++)
    merged->addPoint(*it);
  return merged;
}

double Blob::closestDistance(Blob* a, Blob* b) {
  double d = 1000;
  for(std::list<Point*>::iterator itA = a->points.begin(); itA != a->points.end(); itA++) {
    for(std::list<Point*>::iterator itB = b->points.begin(); itB != b->points.end(); itB++) {
      double temp = distance(**itA,**itB);
      if(d > temp) d = temp;
    }
  }
  return d;
}

double Blob::gapDistance(Blob* a, Blob* b) {
  Point *closestA = 0, *closestB = 0;
  double d = 1000;
  for(std::list<Point*>::iterator it = a->points.begin(); it != a->points.end(); it++)
    if(distance(**it,b->centroid) < d)
      closestA = (Point*)*it;
  d = 1000;
  for(std::list<Point*>::iterator it = b->points.begin(); it != b->points.end(); it++)
    if(distance(**it,a->centroid) < d)
      closestB = (Point*)*it;
  d = distance(*closestA,*closestB);
  return d;
}

bool Blob::blobsOverlap(Blob* a, Blob* b) {
  bool overlap = (a->getRadius() + b->getRadius()) * 2.0 > distance(a->centroid,b->centroid);
  return overlap;
}

void Blob::buildRadius() {
  radius = 0;
  for(std::vector<Point*>::iterator it = vertices.begin(); it != vertices.end(); it++) {
    radius += distance(centroid, **it);
  }
  radius /= vertices.size();
}

int Blob::getWidth() {
  return topRight.x - topLeft.x;
}

int Blob::getHeight() {
  return topLeft.y - bottomLeft.y;
}

double Blob::getBoundingBoxError() {
  return fabs(getWidth() * getHeight() - getArea()) / getArea();
}

double Blob::getWidthHeightRatio() {
  return fabs((double)getWidth() / getHeight());
}

int Blob::getID() {
  return id;
}

// From http://stackoverflow.com/questions/2792443/finding-the-centroid-of-a-polygon
Point Blob::getCentroid() {
  return centroid;
}

double Blob::getRadius() {
  return radius;
}

std::string Blob::toXml(int indent) {
  std::string strIndent;
  for(int i=0;i<indent;i++) strIndent += "\t";
  std::stringstream s;
  s << strIndent << "<Blob";
    s << " BlobID=\"" << id << "\"";
    s << " ColorID=\"" << colorID << "\"";
    s << " Width=\"" << getWidth() << "\"";
    s << " Height=\"" << getHeight() << "\"";
    s << " Radius=\"" << getRadius() << "\"";
  s << ">\n";
  s << strIndent << "\t<Centroid X=\"" << centroid.x << "\" Y=\"" << centroid.y << "\"/>\n";
  for(std::vector<Point*>::iterator it = vertices.begin(); it != vertices.end(); it++)
    s << strIndent << "\t<Vertex X=\"" << (*it)->x << "\" Y=\"" << (*it)->y << "\"/>\n";
  s << strIndent << "</Blob>\n";
  return s.str();
}

void Blob::print() {
  printf("Color: %d\n", colorID);
  printf("ID: %d\n", id);
  printf("area: %f\n", getArea());
  printf("Centroid: %d,%d\n", centroid.x, centroid.y);
  for(std::vector<Point*>::iterator it = vertices.begin(); it != vertices.end(); it++)
    printf("Vertex: %d,%d\n", (*it)->x, (*it)->y);
  for(std::list<Point*>::iterator it = points.begin(); it != points.end(); it++)
    printf("Point: %d,%d\n", (*it)->x, (*it)->y);
}

bool Blob::discard() {
  if(colorID == 0) return true;
  return false;
}

int Blob::getLeft() {
  return topLeft.x;
}

int Blob::getRight() {
  return bottomRight.x;
}

int Blob::getTop() {
  return topLeft.y;
}

int Blob::getBottom() {
  return bottomRight.y;
}
