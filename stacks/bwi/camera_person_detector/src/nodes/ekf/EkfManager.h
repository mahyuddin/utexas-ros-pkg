#ifndef EKF_MANAGER_H
#define EKF_MANAGER_H

#include "PersonEkf.h"
#include "EkfModel.h"

class EkfManager {
  
  private:
      std::vector<PersonEkf*> _filters;
      const double MAX_X_COVARIANCE;
      const double MAX_Y_COVARIANCE;
  
  public:
    EkfManager();
    void updateFilters(std::vector<cv::Rect>, EkfModel*);
    std::vector<PersonEkf*> getValidEstimates();
    
};

#endif
