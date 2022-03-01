#ifndef STABILIZER_H_
#define STABILIZER_H_

#include<iostream>
#include<cmath>
#include<cnoid/EigenTypes>
using namespace std;
using namespace cnoid;

class Stabilizer{
public:
  Vector3 CoM;
  Vector3 vCoM;
  Vector3 CP_d;    //trajectory_plannerからの入力
  Vector3 vCP_d;   //trajectory_plannerからの入力
  Vector3 VRP_mod; //修正されたVRP
  Vector3 CP_mod;  //修正されたCP
  Vector3 vCP_mod; //修正されたvCP
  double b;        //時定数
  double k = 800;//2000だといきすぎて倒れる。500だとたりなくて倒れる
  double g = 9.81;
  void InitializeStabilizer(Vector3 CoMin);
  void Stabilize(Vector3 CP_d, Vector3 vCP_d, Vector3 CP, Vector3 vCP);
};
#endif
