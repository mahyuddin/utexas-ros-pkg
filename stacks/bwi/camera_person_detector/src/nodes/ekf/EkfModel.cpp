#include "EkfModel.h"

EkfModel::EkfModel() : _sysModel(0), _measureModel(0), _infMeasureModel(0) {
}

SystemModel* EkfModel::getSysModel() {
  if(!_sysModel) 
    _sysModel = constructSysModel();
  return _sysModel;
}

MeasureModel* EkfModel::getMeasureModel() {
  if(!_measureModel) 
    _measureModel = constructMeasureModel();
  return _measureModel;
}

MeasureModel* EkfModel::getInfMeasureModel() {
  if(!_infMeasureModel) 
    _infMeasureModel = constructInfMeasureModel();
  return _infMeasureModel;
}

EkfParams* EkfModel::getParams() {
  return &_params;
}

SystemModel* EkfModel::constructSysModel() {
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

  sys_noise_cov(1,1) = _params.SIGMA_SYSTEM_NOISE_X;
  sys_noise_cov(2,2) = _params.SIGMA_SYSTEM_NOISE_Y;
  sys_noise_cov(3,3) = _params.SIGMA_SYSTEM_NOISE_VEL_X;
  sys_noise_cov(4,4) = _params.SIGMA_SYSTEM_NOISE_VEL_Y;
  sys_noise_cov(5,5) = _params.SIGMA_SYSTEM_NOISE_HEIGHT;
  sys_noise_cov(6,6) = _params.SIGMA_SYSTEM_NOISE_HEIGHT_CHANGE;
  BFL::Gaussian system_uncertainity(sys_noise_mu, sys_noise_cov);

  // create the system model
  Gaussian* g = new Gaussian(AB, system_uncertainity);
  return new SystemModel(g);
}

MeasureModel* EkfModel::constructMeasureModel() {
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
  meas_noise_cov(1,1) = _params.SIGMA_MEAS_NOISE_X;
  meas_noise_cov(1,2) = 0.0;
  meas_noise_cov(1,3) = 0.0;
  meas_noise_cov(2,1) = 0.0;
  meas_noise_cov(2,2) = _params.SIGMA_MEAS_NOISE_Y;
  meas_noise_cov(2,3) = 0.0;
  meas_noise_cov(3,1) = 0.0;
  meas_noise_cov(3,2) = 0.0;
  meas_noise_cov(3,3) = _params.SIGMA_MEAS_NOISE_HEIGHT;

  BFL::Gaussian measurement_uncertainity(meas_noise_mu, meas_noise_cov);
  Gaussian* g = new Gaussian(H, measurement_uncertainity);
  return new MeasureModel(g);
}

MeasureModel* EkfModel::constructInfMeasureModel() {
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
  return new MeasureModel(g);
}
