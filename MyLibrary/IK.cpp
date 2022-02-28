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
  //「ヒューマノイドロボット改訂２版」p65~67参照

  //腰リンクの位置を算出
  Vector3 west = CoM - west2CoM;
  Vector3 hip2 = west + west2hip2; //本当はwest2p2に腰リンクの回転行列をかける必要がある
  Vector3 hip8 = west + west2hip8;
  Matrix3d R7T = FootRotation[0].transpose();
  Matrix3d R13T = FootRotation[1].transpose();
  //足首から見た股関節の位置ベクトル
  //p65の式2.60
  Vector3 RightAnkle2hip2 = R7T*(hip2 - Ankle[0]);
  Vector3 LeftAnkle2hip8  = R13T*(hip8 - Ankle[1]);
  //ノルム
  //p66の一番上の式
  double C2 = RightAnkle2hip2.norm();
  double C8 = LeftAnkle2hip8.norm();
  //膝の角度
  //p66の上から3番目の式
  q[5] = -acos((l1*l1 + l2*l2 - C2*C2)/(2*l1*l2)) + pi;
  q[11] = -acos((l1*l1 + l2*l2 - C8*C8)/(2*l1*l2)) + pi;
  //p66の上から6番目の式
  double alphaRight = asin(l1*sin(pi-q[5])/C2);
  double alphaLeft = asin(l1*sin(pi-q[11])/C8);
  //足首関節
  //p66の上から7番目の式
  q[7] = atan2(RightAnkle2hip2[1],RightAnkle2hip2[2]);
  q[13] = atan2(LeftAnkle2hip8[1],LeftAnkle2hip8[2]);
  //p66の上から8番目の式
  q[6] = -atan2(RightAnkle2hip2[0], sqrt(RightAnkle2hip2[1]*RightAnkle2hip2[1]+RightAnkle2hip2[2]*RightAnkle2hip2[2]))-alphaRight;
  q[12] = -atan2(LeftAnkle2hip8[0], sqrt(LeftAnkle2hip8[1]*LeftAnkle2hip8[1]+LeftAnkle2hip8[2]*LeftAnkle2hip8[2]))-alphaLeft;

  Matrix3d Rx7;
  Rx7 << 1,         0,          0,
         0, cos(q[7]), -sin(q[7]),
         0, sin(q[7]),  cos(q[7]);
  Matrix3d Ry56;
  Ry56 <<  cos(q[5]+q[6]), 0, -sin(q[5]+q[6]),
                        0, 1,               0,
           sin(q[5]+q[6]), 0,  cos(q[5]+q[6]);
  Matrix3d Rx13;
  Rx13 << 1,          0,           0,
          0, cos(q[13]), -sin(q[13]),
          0, sin(q[13]),  cos(q[13]);
  Matrix3d Ry1112;
  Ry1112 << cos(q[11]+q[12]), 0, -sin(q[11]+q[12]),
                           0, 1,                 0,
            sin(q[11]+q[12]), 0,  cos(q[11]+q[12]);
  //p66一番下の式の右辺,なおR1は今回は単位行列
  Matrix3d R234 = FootRotation[0]*Rx7*Ry56;
  Matrix3d R8910 = FootRotation[1]*Rx13*Ry1112;
  //p67式2.61
  q[2] = atan2(-R234(0,1), R234(1,1));
  q[8] = atan2(-R8910(0,1), R8910(1,1));
  //p67式2.62多分本の式はマイナスが欠けてる
  q[3] = -atan2(R234(2,1), -R234(0,1)*sin(q[2])+R234(1,1)*cos(q[2]));
  q[9] = -atan2(R8910(2,1), -R8910(0,1)*sin(q[8])+R8910(1,1)*cos(q[8]));
  //p67式2.63
  q[4] = atan2(-R234(2,0),R234(2,2));
  q[10] = atan2(-R8910(2,0),R8910(2,2));
}
