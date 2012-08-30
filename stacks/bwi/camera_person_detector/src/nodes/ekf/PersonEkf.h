#ifndef PERSON_EKF_H
#define PERSON_EKF_H

#include "EkfTypeDefs.h"
#include "EkfModel.h"

class PersonEkf : public BFL::ExtendedKalmanFilter {

  public:
    PersonEkf(int,int,int,EkfModel*);
  private:
    BFL::Gaussian* createGaussian(int,int,int,EkfModel*);
};

#endif
