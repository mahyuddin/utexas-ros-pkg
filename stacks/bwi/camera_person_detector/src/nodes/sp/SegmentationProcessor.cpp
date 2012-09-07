#include "SegmentationProcessor.h"

#define SEG_PROC_DEBUG false
#define segAt(i,j) _image.at<uchar>(j,i)

using namespace sp;

SegmentationProcessor::SegmentationProcessor() {
  initializeRleMap();
}

SegmentationProcessor::~SegmentationProcessor() {
  resetRleMap();
}

void SegmentationProcessor::initializeRleMap() {
  if(SEG_PROC_DEBUG) printf("Initializing the rle map\n");
  for(int i=0;i<SEG_IMAGE_WIDTH;i++)
    for(int j=0;j<SEG_IMAGE_HEIGHT;j++)
      rleMap[i][j] = 0;
}

void SegmentationProcessor::resetRleMap() {
  if(SEG_PROC_DEBUG) printf("Resetting the rle map\n");
  for(int j=0;j<SEG_IMAGE_HEIGHT;j++) {
    for(int i=0;i<SEG_IMAGE_WIDTH;i++) {
      if(rleMap[i][j]) {
        Run* r = rleMap[i][j];
        int k = i;
        for(;rleMap[k][j] == r;k++)
          rleMap[k][j] = 0;
        k--;
        delete r;
        i = k;
      }
    }
  }
}

Run* SegmentationProcessor::generateRun(int i, int j) {
  if(SEG_PROC_DEBUG) printf("Generating a run at %d, %d\n", i, j);
  Run* r = new Run();
  r->left.y = j;
  r->left.x = i;
  r->right.y = j;
  r->colorID = segAt(i,j);
  return r;
}

void SegmentationProcessor::completeRun(Run* r, int i) {
  if(SEG_PROC_DEBUG) printf("Completing a run at %d, %d\n", i + 1, r->right.y);
  r->right.x = i;
  if(r->parent) r->root = r->parent->root;
  else r->root = r;
}

void SegmentationProcessor::constructRleMap() {
  resetRleMap();
  Run *r = 0;
  for(int j=0;j<SEG_IMAGE_HEIGHT;j++) {
    for(int i=0;i<SEG_IMAGE_WIDTH;i++) {
      if(SEG_PROC_DEBUG) printf("Handling %d, %d\n", i, j);
      // First item in the row, so initialize values
      if(!i) r = generateRun(i,j);

      // Adjacent run found above
      if(j && rleMap[i][j-1]->colorID == r->colorID && !r->parent) {
        r->parent = rleMap[i][j-1];
      }

      // New color detected for this row
      if(r->colorID != segAt(i,j)) {
        completeRun(r, i - 1);

        r = generateRun(i,j);
        // Adjacent run found above
        if(j>0 && rleMap[i][j-1]->colorID == r->colorID && !r->parent) {
          r->parent = rleMap[i][j-1];
        }
        
      // Reached the end of the row
      } 
      if (i == SEG_IMAGE_WIDTH - 1) {
        completeRun(r, i);
      }
      rleMap[i][j] = r;
    }
  }
}

void SegmentationProcessor::mergeOverlaps() {
  for(int j=1;j<SEG_IMAGE_HEIGHT;j++) {
    for(int i=0;i<SEG_IMAGE_WIDTH;i++) {
      Run *bottom = rleMap[i][j];
      int width = 1, height = 1;
      int iMin = (i -  width < 0 ? 0 : i -  width), iMax = ( SEG_IMAGE_WIDTH - 1 > i +  width ? i +  width :  SEG_IMAGE_WIDTH - 1);
      int jMin = (j - height < 1 ? 1 : j - height), jMax = j - 1;
      for(int jPrev = jMin; jPrev <= jMax; jPrev++) {
        for(int iPrev = iMin; iPrev <= iMax; iPrev++) {
          Run *top = rleMap[iPrev][jPrev];
          Run *dr;
          if(bottom->colorID == top->colorID && top->deepRoot() != (dr = bottom->deepRoot())) {
            top->deepRoot()->root = dr;
          }
        }
      }
    }
  }
}

std::vector<Blob*> SegmentationProcessor::mergeOverlappedBlobs(std::vector<Blob*>& blobs) {
  std::map<Blob*,bool> mergeMap;
  BOOST_FOREACH(Blob* blob, blobs)
    mergeMap[blob] = false;
  
  std::vector<Blob*> finalBlobs;
  BOOST_FOREACH(Blob* blobA, blobs) {
    if(mergeMap[blobA]) continue;
    Blob* merged = 0;
    if(blobA->getArea() <= 1) continue;
    BOOST_FOREACH(Blob* blobB, blobs) {
      if(mergeMap[blobB]) continue;
      if(blobB == blobA) continue;
      if(blobA->colorID == blobB->colorID && Blob::blobsOverlap(blobA,blobB)) {
        Blob* temp;
        if(merged) {
          temp = Blob::merge(merged,blobB);
          delete merged;
          merged = temp;
        }
        else merged = Blob::merge(blobA,blobB);
        mergeMap[blobA] = true;
        mergeMap[blobB] = true;
      }
    }
    if(mergeMap[blobA]) {
      merged->build();
      finalBlobs.push_back(merged);
    }
    else {
      finalBlobs.push_back(blobA);
    }
  }
  BOOST_FOREACH(Blob* blob, blobs)
    if(mergeMap[blob])
      delete blob;
  return finalBlobs;
}

std::vector<Blob*> SegmentationProcessor::constructBlobs(cv::Mat& image) {
  _image = image;
  std::vector<Blob*> blobs;
  constructRleMap();
  mergeOverlaps();  

  std::map<Run*,Blob*> blobMap;
  std::map<Blob*,Run*> rootMap;
  std::map<Run*,bool> seen;
  Blob* b;
  int runsAdded = 0;
  for(int j=0;j<SEG_IMAGE_HEIGHT;j++) {
    for(int i=0;i<SEG_IMAGE_WIDTH;i++) {
      Run* r = rleMap[i][j];
      if(r->colorID == c_UNDEFINED) continue;
      if(seen[r])
        continue;
      seen[r] = true;
      Run* root = r->deepRoot();
      if(blobMap[root]) {
        b = blobMap[root];
      } else {
        b = new Blob();
        rootMap[b] = root;
        b->colorID = r->colorID;
        blobs.push_back(b);
        blobMap[root]=b;
      }
      runsAdded++;
      b->addRun(r);
    }
  }
  
  std::vector<Blob*> final;
  BOOST_FOREACH(Blob* b, blobs) {
    b->build();
    if(b->getArea() > 1)
      final.push_back(b);
  }
  
  for(int i=0; i<1; i++) {
    final = mergeOverlappedBlobs(final);
  }

  return final;
}