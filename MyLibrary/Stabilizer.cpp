#include "Stabilizer.h"
#include<cmath>
using namespace std;
using namespace cnoid;

void Stabilizer::InitializeStabilizer(Vector3 CoMin){
  CoM = CoMin;
  vCoM = Vector3(0.0, 0.0, 0.0);
}

void Stabilizer::Stabilize(Vector3 CP_d, Vector3 vCP_d, Vector3 CP, Vector3 vCP){
  b = sqrt(CoM[2]/g);
  //VRPを修正する
  VRP_mod = CP + k*b*(CP - CP_d) - b*vCP_d;
  //重心の位置を少し動かす
  CoM += vCoM*0.001;
  //加速度を更新
  Vector3 accel_mod = 1/(b*b)*(CoM - VRP_mod + Vector3(0.0, 0.0, b*b*g));
  //速度を更新
  vCoM = accel_mod*0.001;
}
