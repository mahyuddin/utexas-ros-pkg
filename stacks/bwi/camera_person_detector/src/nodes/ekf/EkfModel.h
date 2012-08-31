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
    EkfParams _params;  
    
  private:
    SystemModel* constructSysModel();
    MeasureModel* constructMeasureModel();
    MeasureModel* constructInfMeasureModel();
    
    SystemModel* _sysModel;
    MeasureModel* _measureModel;
    MeasureModel* _infMeasureModel;
    
};

#endif
