#ifndef EKF_MODEL_H
#define EKF_MODEL_H

#include "EkfTypeDefs.h"
#include "EkfParams.h"

class EkfModel {

  public:
    EkfModel();
    SystemModel* getSysModel();
    MeasureModel* getMeasureModel();
    MeasureModel* getInfMeasureModel();
    EkfParams* getParams();
    
  protected:
    virtual SystemModel* constructSysModel() = 0;
    virtual MeasureModel* constructMeasureModel() = 0;
    virtual MeasureModel* constructInfMeasureModel() = 0;
    EkfParams _params;
    
  private:
    SystemModel* _sysModel;
    MeasureModel* _measureModel;
    MeasureModel* _infMeasureModel;
    
};

#endif
