#ifndef PERSON_EKF_H
#define PERSON_EKF_H

#include <boost/foreach.hpp>
#include <opencv/cv.h>
#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>

typedef BFL::LinearAnalyticConditionalGaussian Gaussian;
typedef BFL::LinearAnalyticSystemModelGaussianUncertainty SystemModel;
typedef BFL::LinearAnalyticMeasurementModelGaussianUncertainty MeasureModel;
#define EkfPtr boost::shared_ptr<PersonEKF> 

class PersonEKF : public BFL::ExtendedKalmanFilter {

  public:
    PersonEKF(int,int,int);
    static SystemModel* getSysModel();
    static MeasureModel* getMeasureModel();
    static MeasureModel* getInfMeasureModel();
    static void updateFilters(std::vector<cv::Rect>);
    static std::vector<PersonEKF*> getValidEstimates();
  private:
    BFL::Gaussian* createGaussian(int,int,int);
    static SystemModel* _sysModel;
    static MeasureModel* _measureModel;
    static MeasureModel* _infMeasureModel;

    static std::vector<PersonEKF*> _filters;

    static const double SIGMA_SYSTEM_NOISE_X;
    static const double SIGMA_SYSTEM_NOISE_Y;
    static const double SIGMA_SYSTEM_NOISE_VEL_X;
    static const double SIGMA_SYSTEM_NOISE_VEL_Y;
    static const double SIGMA_SYSTEM_NOISE_HEIGHT;
    static const double SIGMA_SYSTEM_NOISE_HEIGHT_CHANGE;
    static const double SIGMA_MEAS_NOISE_X;
    static const double SIGMA_MEAS_NOISE_Y;
    static const double SIGMA_MEAS_NOISE_HEIGHT;
    static const double MAX_X_COVARIANCE;
    static const double MAX_Y_COVARIANCE;
};

#endif
