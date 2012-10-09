#include "PersonEkf.h"

PersonEkf::PersonEkf(PersonReading reading) : BFL::ExtendedKalmanFilter(createGaussian(reading.x, reading.y, reading.height, reading.model)) {
  _id = reading.id;
  _lastModel = reading.model;
}    

BFL::Gaussian* PersonEkf::createGaussian(double x, double y, double height, EkfModel* model) {
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
  prior_cov(1,1) = model->getParams()->SIGMA_MEAS_NOISE_X; 
  prior_cov(2,2) = model->getParams()->SIGMA_MEAS_NOISE_Y;
  prior_cov(3,3) = model->getParams()->SIGMA_MEAS_NOISE_HEIGHT;
  return new BFL::Gaussian(prior_mu,prior_cov);
}

int PersonEkf::getId() {
  return _id;
}

void PersonEkf::setId(int id) {
  _id = id;
}

void PersonEkf::updateWithMeasurement(PersonReading reading) {
  MatrixWrapper::ColumnVector measurement(3);
  measurement(1) = reading.x;
  measurement(2) = reading.y;
  measurement(3) = reading.height;
  _lastModel = reading.model;
  Update(_lastModel->getSysModel(), _lastModel->getMeasureModel(), measurement);
}

void PersonEkf::updateWithoutMeasurement() {
  BFL::Pdf<MatrixWrapper::ColumnVector>* posterior = PostGet();
  MatrixWrapper::ColumnVector mean = posterior->ExpectedValueGet();
  MatrixWrapper::ColumnVector measurement(3);
  measurement(1) = mean(1);
  measurement(2) = mean(2);
  measurement(3) = mean(5);
  Update(_lastModel->getSysModel(), _lastModel->getInfMeasureModel(), measurement);
}

