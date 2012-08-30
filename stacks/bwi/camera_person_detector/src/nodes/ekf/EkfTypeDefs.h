#ifndef EKF_TYPEDEFS_H
#define EKF_TYPEDEFS_H

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

#endif
