#ifndef PERSON_EKF_H
#define PERSON_EKF_H

#include "EkfTypeDefs.h"
#include "EkfModel.h"
#include "PersonReading.h"

class PersonEkf : public BFL::ExtendedKalmanFilter {

  public:
    PersonEkf(PersonReading);
    int getId();
    void setId(int);
    void updateWithoutMeasurement();
    void updateWithMeasurement(PersonReading);
  private:
    BFL::Gaussian* createGaussian(double,double,double,EkfModel*);
    int _id;
    EkfModel* _lastModel;
};

#endif
