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
