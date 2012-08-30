#ifndef HOG_MODEL_H
#define HOG_MODEL_H

#include "EkfModel.h"

class HogModel : public EkfModel {
  public:
    HogModel();
    SystemModel* constructSysModel();
    MeasureModel* constructMeasureModel();
    MeasureModel* constructInfMeasureModel();
};

#endif
