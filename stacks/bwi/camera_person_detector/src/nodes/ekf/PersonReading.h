#ifndef PERSON_READING_H
#define PERSON_READING_H

struct PersonReading {
  double x;
  double y;
  double height;
  int id;
  PersonReading() { }
  PersonReading(double x, double y, double height, int id) {
    this->x = x;
    this->y = y;
    this->height = height;
    this->id = id;
  }
};

#endif
