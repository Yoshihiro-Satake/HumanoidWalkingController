#include "IK.h"
#include<cnoid/EigenTypes>
#include<cnoid/Link>
#include<cnoid/Body>

void IK::InitializeIK(double _l1, double _l2, Vector3 _west2hip2 , Vector3 _west2hip8, Vector3 _west2CoM){
  l1 = _l1;
  l2 = _l2;
  west2hip2 = _west2hip2;
  west2hip8 = _west2hip8;
  west2CoM  = _west2CoM;
}

void IK::IKLeg(vector<Vector3> Ankle, vector<Matrix3d> FootRotation, Vector3 CoM){
  //Rにはfoot_rotationが入る
  //逆運動学

  //腰リンクの位置を算出
  Vector3 west = CoM - west2CoM;
  Vector3 hip2 = west + west2hip2; //本当はwest2p2に腰リンクの回転行列をかける必要がある
  Vector3 hip8 = west + west2hip8;
  Matrix3d R7T = FootRotation[0].transpose();
  Matrix3d R13T = FootRotation[1].transpose();
  //足首から見た股関節の位置ベクトル
  Vector3 RightAnkle2hip2 = R7T.transpose()*(hip2 - Ankle[0]);
  Vector3 LeftAnkle2hip8  = R13T.transpose()*(hip8 - Ankle[1]);
  double C2 = RightAnkle2hip2.norm();
  double C8 = LeftAnkle2hip8.norm();
  q[5] = -acos((l1*l1 + l2*l2 - C2*C2)/(2*l1*l2)) + pi;
}
