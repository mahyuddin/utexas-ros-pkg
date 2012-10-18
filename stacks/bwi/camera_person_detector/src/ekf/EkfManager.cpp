#include "EkfManager.h"

EkfManager::EkfManager() : MAX_X_COVARIANCE(7), MAX_Y_COVARIANCE(7) {
}

void EkfManager::updateFilters(std::vector<PersonReading> readings) {
  // Check which locations have been matched
  std::vector<bool> used_locations;
  used_locations.resize(readings.size());
  for (size_t i = 0; i < readings.size(); i++)
    used_locations[i] = false;

  BOOST_FOREACH(PersonEkf* filter, _filters) {

    // Get filter prediction
    BFL::Pdf<MatrixWrapper::ColumnVector>* posterior = filter->PostGet();
    MatrixWrapper::ColumnVector mean = posterior->ExpectedValueGet();

    size_t i = 0;
    for (i = 0; i < readings.size(); i++) {
      if (used_locations[i])
        continue;
      PersonReading r(readings[i]);
      if(filter->isMatch(r)) {
        filter->updateWithMeasurement(r);
        used_locations[i] = true;
      }
    }

    // No matching correspondences found, update without measurement
    if (i == readings.size()) {
      filter->updateWithoutMeasurement();
    }
  }

  for (size_t i = 0; i < readings.size(); i++) {
    if (used_locations[i])
      continue;
    if (!readings[i].model->canInit())
      continue;   
    PersonEkf* filter = new PersonEkf(readings[i]);
    _filters.push_back(filter);
  }
}

std::vector<PersonEkf*> EkfManager::getValidEstimates() {
  std::vector<PersonEkf*> valid;
  BOOST_FOREACH(PersonEkf* filter, _filters) {
    BFL::Pdf<MatrixWrapper::ColumnVector>* posterior = filter->PostGet();
    MatrixWrapper::SymmetricMatrix covariance = posterior->CovarianceGet();
    if(covariance(1,1) <= MAX_X_COVARIANCE && covariance(2,2) <= MAX_Y_COVARIANCE)
      valid.push_back(filter);
    else
      delete filter;
  }
  _filters = valid;
  return _filters;
}
