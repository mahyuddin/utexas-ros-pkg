#include "EkfManager.h"

EkfManager::EkfManager() : MAX_X_COVARIANCE(5), MAX_Y_COVARIANCE(5) {
}

void EkfManager::updateFilters(std::vector<PersonReading> readings, EkfModel* model) {
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
      // if location is close enough
      MatrixWrapper::ColumnVector measurement(3);
      measurement(1) = r.x;
      measurement(2) = r.y;
      measurement(3) = r.height;
      bool close = abs(measurement(1) - mean(1)) < .75 && 
                               abs(measurement(2) - mean(2)) < .75 &&
                               abs(measurement(3) - mean(5) < .5);
      if (close) {
        filter->Update(model->getSysModel(), model->getMeasureModel(), measurement);
        if(r.hasId()) filter->setId(r.id);
        used_locations[i] = true;
        break;
      }
    }

    // No matching correspondences found, update without measurement
    if (i == readings.size()) {
      MatrixWrapper::ColumnVector measurement(3);
      measurement(1) = mean(1);
      measurement(2) = mean(2);
      measurement(3) = mean(5);
      filter->Update(model->getSysModel(), model->getInfMeasureModel(), measurement);
    }

  }

  for (size_t i = 0; i < readings.size(); i++) {
    if (used_locations[i])
      continue;

    PersonEkf* filter = new PersonEkf(readings[i], model);
    _filters.push_back(filter);
  }
}

std::vector<PersonEkf*> EkfManager::getValidEstimates() {
  std::vector<PersonEkf*> estimates;
  BOOST_FOREACH(PersonEkf* filter, _filters) {
    BFL::Pdf<MatrixWrapper::ColumnVector>* posterior = filter->PostGet();
    MatrixWrapper::SymmetricMatrix covariance = posterior->CovarianceGet();
    if(covariance(1,1) <= MAX_X_COVARIANCE && covariance(2,2) <= MAX_Y_COVARIANCE)
      estimates.push_back(filter);
  }
  return estimates;
}
