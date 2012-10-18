#include "HogModel.h"

HogModel::HogModel() : EkfModel() {
  _params.SIGMA_SYSTEM_NOISE_X = 0.25;
  _params.SIGMA_SYSTEM_NOISE_Y = 0.25;
  _params.SIGMA_SYSTEM_NOISE_VEL_X = 0.25;
  _params.SIGMA_SYSTEM_NOISE_VEL_Y = 0.25;
  _params.SIGMA_SYSTEM_NOISE_HEIGHT = 0.25;
  _params.SIGMA_SYSTEM_NOISE_HEIGHT_CHANGE = 0.25;
  _params.SIGMA_MEAS_NOISE_X = 0.6;
  _params.SIGMA_MEAS_NOISE_Y = 0.6;
  _params.SIGMA_MEAS_NOISE_HEIGHT = 0.6; 
  _canInit = true;
}
