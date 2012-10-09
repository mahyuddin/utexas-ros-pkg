#ifndef PERSON_READING_H
#define PERSON_READING_H
#include "EkfModel.h"

struct PersonReading {
  double x;
  double y;
  double height;
  EkfModel* model; 
  cv::Rect box;
  uint64_t id;
  PersonReading(double x, double y, double height, cv::Rect box, EkfModel* model) {
    this->x = x;
    this->y = y;
    this->height = height;
    this->box = box;
    this->model = model;
    this->id = 0;
  }
  PersonReading(double x, double y, double height, int id) {
    this->x = x;
    this->y = y;
    this->height = height;
    this->model = 0;
    this->id = id;
  }
  bool hasId() { return this->id; }  
  bool hasModel() { return this->model; }
};

#endif
