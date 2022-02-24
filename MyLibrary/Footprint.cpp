#include "Footprint.h"
#include<cnoid/EigenTypes>
#include<cmath>
#include<vector>
using namespace std;
using namespace cnoid;

//Vector6はかなり使いにくい
void Footprints::SetFootprints(vector<double> data, int _RightOrLeft){
  //着地位置姿勢データを追加する。
  FootprintData footprint;
  footprint.RightOrLeft = _RightOrLeft;
  footprint.foot_position = Vector3(data[0], data[1], data[2]);
  Matrix3d Rz;
  Rz << cos(data[3]), -sin(data[3]), 0,
        sin(data[3]),  cos(data[3]), 0,
                   0,             0, 1;

  Matrix3d Ry;
  Ry << cos(data[4]), 0, -sin(data[4]),
                   0, 1,             0,
        sin(data[4]), 0,  cos(data[4]);

  Matrix3d Rx;
  Rx << 1,            0,             0,
        0, cos(data[5]), -sin(data[5]),
        0, sin(data[5]),  cos(data[5]);
  footprint.R = Rz*Ry*Rx;
  datas.push_back(footprint);
}
