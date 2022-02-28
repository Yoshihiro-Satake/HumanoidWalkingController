#ifndef SENSING_UNIT_H_
#define SENSING_UNIT_H_

#include<iostream>
#include<cnoid/Body>
#include<cnoid/BasicSensors>
#include<cnoid/EigenTypes>
using namespace std;
using namespace cnoid;

class Sensors{
public:
  ForceSensorPtr LeftAnkleForceSensor;  //ZMP計測用。今はまだ使わない
  ForceSensorPtr RightAnkleForceSensor; //ZMP計測用。今はまだ使わない
  AccelerationSensorPtr CoMAccelSensor; //重心加速度を計測

  Vector3 CoM;    //実際のCoM
  Vector3 CP;     //実際のCP
  Vector3 vCoM;   //実際の重心速度
  Vector3 vCP;    //実際のCP速度
  void InitializeSensors(ForceSensorPtr _LeftAnkleForceSensor,
                         ForceSensorPtr _RightAnkleForceSensor,
                         AccelerationSensorPtr _CoMAccelSensor);
  void Sensing(BodyPtr ioBody);
};
#endif
