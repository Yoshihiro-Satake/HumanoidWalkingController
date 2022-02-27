#ifndef IK_H_
#define IK_H_

#include<iostream>
#include "Footprint.h"

using namespace std;
using namespace cnoid;

class IK{
public:
  //股関節は英語でhip joint
  Vector3 west2hip2;   //右股関節の腰リンクからみた位置
  Vector3 west2hip8;   //左股関節の腰リンクからみた位置
  Vector3 west2CoM;      //腰リンクから見た重心位置
  Vector3 RightAnkle;  //右足首のワールド位置
  Vector3 LeftAnkle;   //左足首のワールド位置
  double l1;           //股関節から膝までの長さ
  double l2;           //股関節から足首までの長さ
  //出力する関節角度
  vector<double> q = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  double pi = 3.141592;

  void InitializeIK(double _l1, double _l2, Vector3 _west2hip2, Vector3 _west2hip8, Vector3 _west2CoM);
  void IKLeg(vector<Vector3> Ankle, vector<Matrix3d> R, Vector3 CoM);
};
#endif
