#include "BsModel.h"

BsModel::BsModel() : EkfModel() {
  _params.SIGMA_SYSTEM_NOISE_X = 0.1;
  _params.SIGMA_SYSTEM_NOISE_Y = 0.1;
  _params.SIGMA_SYSTEM_NOISE_VEL_X = 0.25;
  _params.SIGMA_SYSTEM_NOISE_VEL_Y = 0.25;
  _params.SIGMA_SYSTEM_NOISE_HEIGHT = 0.25;
  _params.SIGMA_SYSTEM_NOISE_HEIGHT_CHANGE = 0.01;
  _params.SIGMA_MEAS_NOISE_X = .2;
  _params.SIGMA_MEAS_NOISE_Y = .2;
  _params.SIGMA_MEAS_NOISE_HEIGHT = .1; 
}
