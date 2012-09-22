#ifndef BLOB_H
#define BLOB_H

#include <vector>
#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <list>
#include <math.h>

#include "Run.h"
#include "Point.h"
#include "Constants.h"

namespace sp {
  class Blob {
    private:
      static bool pointMap[SEG_IMAGE_WIDTH][SEG_IMAGE_HEIGHT];
      static int _id;
      int id;
      double radius;
      Point topLeft, topRight, bottomLeft, bottomRight;
      Point centroid;
    protected:
      std::vector<Point*> vertices;
      std::list<Point*> points;
      bool isLeftOf(Point&,Point&,Point&);
      void buildHull();
      void buildCentroid();
      void buildRadius();
      void clearPointMap();
      void removeDuplicatePoints();
      void trimExtremes();
    public:
      int colorID;
      Blob();
      ~Blob();
      void build();
      void addPoint(Point*);
      void addRun(Run*);
      std::string toXml(int indent);
      void print();
      bool discard();
      Point getCentroid();
      int getWidth();
      int getHeight();
      int getID();
      int getLeft();
      int getRight();
      int getTop();
      int getBottom();
      double getArea();
      double getRadius();
      double getWidthHeightRatio();
      double getBoundingBoxError();
      static double distance(Point, Point);
      static bool blobsOverlap(Blob*,Blob*);
      static double gapDistance(Blob*,Blob*);
      static double closestDistance(Blob*,Blob*);
      static Blob* merge(Blob*,Blob*);
      bool isNearBorder();
  };
}
#endif
