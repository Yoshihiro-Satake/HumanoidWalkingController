#ifndef FOOTPRINT_H_
#define FOOTPRINT_H_

#include<iostream>
#include<cnoid/EigenTypes>
#include<vector>
using namespace std;
using namespace cnoid;

struct FootprintData{
  //歩くのに必要な着地データ
  //着地データは着地点と着地時の足の姿勢が必要
  //クウォータ二オンがよくわからない
  Vector3 foot_position;   //足の3次元座標(足裏の足首関節直下をここにもってくる)
  Matrix3d R;              //回転行列
  int RightOrLeft;         //右足か左足か。0なら右1なら左
};

class Footprints{
  //着地データの集まり
public:
  vector<FootprintData> datas; //着地位置姿勢データの集まり
  void SetFootprints(vector<double> data, int _RightOrLeft); //[x,y,z,z周り,y周り,x周り]、オイラー角
};
#endif
