#ifndef PERSON_READING_H
#define PERSON_READING_H

struct PersonReading {
  double x;
  double y;
  double height;
  PersonReading() { }
  PersonReading(double x, double y, double height) {
    this->x = x;
    this->y = y;
    this->height = height;
  }
};

#endif
