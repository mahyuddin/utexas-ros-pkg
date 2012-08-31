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
      
      // if location is close enough
      MatrixWrapper::ColumnVector measurement(3);
//      measurement(1) = locations[i].x + locations[i].width / 2;
//      measurement(2) = locations[i].y + locations[i].height;
//      measurement(3) = locations[i].height;
      measurement(1) = readings[i].x;
      measurement(2) = readings[i].y;
      measurement(3) = readings[i].height;
      bool location_is_close = abs(measurement(1) - mean(1)) < 100 && 
                               abs(measurement(2) - mean(2)) < 100 &&
                               abs(measurement(3) - mean(5) < 100);
      if (location_is_close) {
        filter->Update(model->getSysModel(), model->getMeasureModel(), measurement);
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
    // Create new EKF
//    int x = locations[i].x + locations[i].width / 2;
//    int y = locations[i].y + locations[i].height;
//    int height = locations[i].height;

    PersonEkf* filter = new PersonEkf(readings[i].x,readings[i].y,readings[i].height,model);
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
