#include "PersonEKF.h"


const double PersonEKF::SIGMA_SYSTEM_NOISE_X = 0.25;
const double PersonEKF::SIGMA_SYSTEM_NOISE_Y = 0.25;
const double PersonEKF::SIGMA_SYSTEM_NOISE_VEL_X = 0.25;
const double PersonEKF::SIGMA_SYSTEM_NOISE_VEL_Y = 0.25;
const double PersonEKF::SIGMA_SYSTEM_NOISE_HEIGHT = 0.25;
const double PersonEKF::SIGMA_SYSTEM_NOISE_HEIGHT_CHANGE = 0.25;
const double PersonEKF::SIGMA_MEAS_NOISE_X = 1;
const double PersonEKF::SIGMA_MEAS_NOISE_Y = 4;
const double PersonEKF::SIGMA_MEAS_NOISE_HEIGHT = 4;
const double PersonEKF::MAX_X_COVARIANCE = 5;
const double PersonEKF::MAX_Y_COVARIANCE = 5;
std::vector<PersonEKF*> PersonEKF::_filters;

PersonEKF::PersonEKF(int x, int y, int height) : BFL::ExtendedKalmanFilter(createGaussian(x,y,height)) {
}    

BFL::Gaussian* PersonEKF::createGaussian(int x, int y, int height) {
  MatrixWrapper::ColumnVector prior_mu(6);
  prior_mu(1) = x;
  prior_mu(2) = y;
  prior_mu(3) = 0;
  prior_mu(4) = 0;
  prior_mu(5) = height;
  prior_mu(6) = 0;
  MatrixWrapper::SymmetricMatrix prior_cov(6);
  for (int i = 1; i<=6; i++) {
    for (int j = 1; j <=6; j++) {
      prior_cov(i,j) = 0.0;
    }
  } 
  prior_cov(1,1) = SIGMA_MEAS_NOISE_X; 
  prior_cov(2,2) = SIGMA_MEAS_NOISE_Y;
  prior_cov(3,3) = SIGMA_MEAS_NOISE_HEIGHT;
  return new BFL::Gaussian(prior_mu,prior_cov);
}

SystemModel* PersonEKF::_sysModel = 0;
SystemModel* PersonEKF::getSysModel() {
  if(_sysModel) return _sysModel;
  MatrixWrapper::Matrix A(6,6);
  for (int i = 1; i <= 6; i++)
    for (int j = 1; j <= 6; j++)
      A(i,j) = 0.0;
  A(1,1) = 1.0;
  A(1,3) = 1.0;
  A(2,2) = 1.0;
  A(2,4) = 1.0;
  A(3,3) = 1.0;
  A(4,4) = 1.0;
  A(5,5) = 1.0;
  A(5,6) = 1.0;
  A(6,6) = 1.0;

  std::vector<MatrixWrapper::Matrix> AB(1);
  AB[0] = A;

  // No biases
  MatrixWrapper::ColumnVector sys_noise_mu(6);
  sys_noise_mu(1) = 0;
  sys_noise_mu(2) = 0;
  sys_noise_mu(3) = 0;
  sys_noise_mu(4) = 0;
  sys_noise_mu(5) = 0;
  sys_noise_mu(6) = 0;

  // system noise. this should probably not be independent
  MatrixWrapper::SymmetricMatrix sys_noise_cov(6);
  for (int i = 1; i <= 6; i++)
    for (int j = 1; j <= 6; j++)
      sys_noise_cov(i,j) = 0.0;

  sys_noise_cov(1,1) = SIGMA_SYSTEM_NOISE_X;
  sys_noise_cov(2,2) = SIGMA_SYSTEM_NOISE_Y;
  sys_noise_cov(3,3) = SIGMA_SYSTEM_NOISE_VEL_X;
  sys_noise_cov(4,4) = SIGMA_SYSTEM_NOISE_VEL_Y;
  sys_noise_cov(5,5) = SIGMA_SYSTEM_NOISE_HEIGHT;
  sys_noise_cov(6,6) = SIGMA_SYSTEM_NOISE_HEIGHT_CHANGE;
  BFL::Gaussian system_uncertainity(sys_noise_mu, sys_noise_cov);

  // create the system model
  Gaussian* g = new Gaussian(AB, system_uncertainity);
  _sysModel = new SystemModel(g);
  return _sysModel;
}

MeasureModel* PersonEKF::_measureModel = 0;
MeasureModel* PersonEKF::getMeasureModel() {
  if(_measureModel) return _measureModel;
  // create the measurement matrix
  MatrixWrapper::Matrix H(3,6);
  for (int i = 1; i <= 3; i++)
    for (int j = 1; j <= 6; j++)
      H(i,j) = 0.0;

  H(1,1) = 1.0;
  H(1,3) = 1.0;
  H(2,2) = 1.0;
  H(2,4) = 1.0;
  H(3,5) = 1.0;
  H(3,6) = 1.0;

  // Construct the measurement noise (a scalar in this case)
  MatrixWrapper::ColumnVector meas_noise_mu(3);
  meas_noise_mu(1) = 0;
  meas_noise_mu(2) = 0;
  meas_noise_mu(3) = 0;

  MatrixWrapper::SymmetricMatrix meas_noise_cov(3);
  meas_noise_cov(1,1) = SIGMA_MEAS_NOISE_X;
  meas_noise_cov(1,2) = 0.0;
  meas_noise_cov(1,3) = 0.0;
  meas_noise_cov(2,1) = 0.0;
  meas_noise_cov(2,2) = SIGMA_MEAS_NOISE_Y;
  meas_noise_cov(2,3) = 0.0;
  meas_noise_cov(3,1) = 0.0;
  meas_noise_cov(3,2) = 0.0;
  meas_noise_cov(3,3) = SIGMA_MEAS_NOISE_HEIGHT;

  BFL::Gaussian measurement_uncertainity(meas_noise_mu, meas_noise_cov);
  Gaussian* g = new Gaussian(H, measurement_uncertainity);
  _measureModel = new MeasureModel(g);
  return _measureModel;
}

MeasureModel* PersonEKF::_infMeasureModel = 0;
MeasureModel* PersonEKF::getInfMeasureModel() {
  if(_infMeasureModel) return _infMeasureModel;

  // create the measurement matrix
  MatrixWrapper::Matrix H(3,6);
  for (int i = 1; i <= 3; i++)
    for (int j = 1; j <= 6; j++)
      H(i,j) = 0.0;

  MatrixWrapper::ColumnVector meas_noise_mu(3);
  meas_noise_mu(1) = 0;
  meas_noise_mu(2) = 0;
  meas_noise_mu(3) = 0;
  // Construct infinite measurement uncertainity (update when no measurement is available)
  MatrixWrapper::SymmetricMatrix inf_meas_noise_cov(3);
  inf_meas_noise_cov(1,1) = 1e30;
  inf_meas_noise_cov(1,2) = 0.0;
  inf_meas_noise_cov(1,3) = 0.0;
  inf_meas_noise_cov(2,1) = 0.0;
  inf_meas_noise_cov(2,2) = 1e30;
  inf_meas_noise_cov(2,3) = 0.0;
  inf_meas_noise_cov(3,1) = 0.0;
  inf_meas_noise_cov(3,2) = 0.0;
  inf_meas_noise_cov(3,3) = 1e30;

  BFL::Gaussian inf_measurement_uncertainity(meas_noise_mu, inf_meas_noise_cov);
  Gaussian* g = new Gaussian(H, inf_measurement_uncertainity);
  _infMeasureModel = new MeasureModel(g);
  return _infMeasureModel;
}

void PersonEKF::updateFilters(std::vector<cv::Rect> locations) {
  // Check which locations have been matched
  std::vector<bool> used_locations;
  used_locations.resize(locations.size());
  for (size_t i = 0; i < locations.size(); i++)
    used_locations[i] = false;

  BOOST_FOREACH(PersonEKF* filter, _filters) {

    // Get filter prediction
    BFL::Pdf<MatrixWrapper::ColumnVector>* posterior = filter->PostGet();
    MatrixWrapper::ColumnVector mean = posterior->ExpectedValueGet();
    //MatrixWrapper::SymmetricMatrix covariance = posterior->CovarianceGet();

    size_t i = 0;
    for (i = 0; i < locations.size(); i++) {
      if (used_locations[i])
        continue;
      
      // if location is close enough
      MatrixWrapper::ColumnVector measurement(3);
      measurement(1) = locations[i].x + locations[i].width / 2;
      measurement(2) = locations[i].y + locations[i].height;
      measurement(3) = locations[i].height;
      bool location_is_close = abs(measurement(1) - mean(1)) < 100 && 
                               abs(measurement(2) - mean(2)) < 100 &&
                               abs(measurement(3) - mean(5) < 100);
      if (location_is_close) {
        filter->Update(PersonEKF::getSysModel(), PersonEKF::getMeasureModel(), measurement);
        used_locations[i] = true;
        break;
      }
    }

    // No matching correspondences found, update without measurement
    if (i == locations.size()) {
      MatrixWrapper::ColumnVector measurement(3);
      measurement(1) = mean(1);
      measurement(2) = mean(2);
      measurement(3) = mean(5);
      filter->Update(PersonEKF::getSysModel(), PersonEKF::getInfMeasureModel(), measurement);
    }

  }

  for (size_t i = 0; i < locations.size(); i++) {
    if (used_locations[i])
      continue;
    // Create new EKF
    int x = locations[i].x + locations[i].width / 2;
    int y = locations[i].y + locations[i].height;
    int height = locations[i].height;

    PersonEKF* filter = new PersonEKF(x,y,height);
    _filters.push_back(filter);
  }
}

std::vector<PersonEKF*> PersonEKF::getValidEstimates() {
  std::vector<PersonEKF*> estimates;
  BOOST_FOREACH(PersonEKF* filter, _filters) {
    BFL::Pdf<MatrixWrapper::ColumnVector>* posterior = filter->PostGet();
    MatrixWrapper::SymmetricMatrix covariance = posterior->CovarianceGet();
    if(covariance(1,1) <= MAX_X_COVARIANCE && covariance(2,2) <= MAX_Y_COVARIANCE)
      estimates.push_back(filter);
  }
  return estimates;
}
