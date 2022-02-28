#include "Sensing_Unit.h"
#include<cmath>
using namespace cnoid;
using namespace std;

void Sensors::InitializeSensors(ForceSensorPtr _LeftAnkleForceSensor, ForceSensorPtr _RightAnkleForceSensor, AccelerationSensorPtr _CoMAccelSensor, Vector3 CoMin){
  LeftAnkleForceSensor = _LeftAnkleForceSensor;
  RightAnkleForceSensor = _RightAnkleForceSensor;
  CoMAccelSensor = _CoMAccelSensor;
  vCoM = Vector3(0.0, 0.0, 0.0);
  vCP = Vector3(0.0, 0.0, 0.0);
  CP = CoMin;
}

void Sensors::Sensing(BodyPtr ioBody){
  //重心、CPの位置速度を計測して計算する
  CoM = ioBody->calcCenterOfMass();
  vCoM += CoMAccelSensor->dv()*0.001;
  CP = CoM + sqrt(CoM[2]/g)*vCoM;
  vCP = vCoM + sqrt(CoM[2]/g)*CoMAccelSensor->dv();
}
