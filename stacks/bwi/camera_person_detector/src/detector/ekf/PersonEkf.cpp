#include "PersonEkf.h"

PersonEkf::PersonEkf(PersonReading reading, EkfModel* model) : BFL::ExtendedKalmanFilter(createGaussian(reading.x,reading.y,reading.height,model)) {
  _id = reading.id;
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
