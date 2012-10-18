#include "BsModel.h"

BsModel::BsModel() : EkfModel() {
  _params.SIGMA_SYSTEM_NOISE_X = 1.0;
  _params.SIGMA_SYSTEM_NOISE_Y = 1.0;
  _params.SIGMA_SYSTEM_NOISE_VEL_X = 0.25;
  _params.SIGMA_SYSTEM_NOISE_VEL_Y = 0.25;
  _params.SIGMA_SYSTEM_NOISE_HEIGHT = 0.25;
  _params.SIGMA_SYSTEM_NOISE_HEIGHT_CHANGE = 0.5;
  _params.SIGMA_MEAS_NOISE_X = 4.0;
  _params.SIGMA_MEAS_NOISE_Y = 4.0;
  _params.SIGMA_MEAS_NOISE_HEIGHT = 4.0; 
  _canInit = true;
}
