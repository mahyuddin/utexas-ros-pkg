#ifndef CLOUD_BLOB_H
#define CLOUD_BLOB_H

#include <vector>
#include "CloudSegment.h"
#include "CloudMacros.h"

class CloudBlob {
  private:
    std::vector<CloudSegment*> _segments;
    bool _isAdded;  
  public:
    long long id;
    static long long ID;
    bool deleted;
    CloudBlob();
    ~CloudBlob();
    void addSegment(CloudSegment*);
    void merge(CloudBlob*);
    float distanceTo(CloudBlob*);
    int size();
    CPoint getCentroid();
    float getHeight();
    float getWidth();
    float getDepth();
    float getArea();
    void isAdded(bool);
    bool isAdded();
    void output();
};

#endif
