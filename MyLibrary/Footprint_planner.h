#ifndef FOOTPRINT_PLAMMER_H_
#define FOOTPRINT_PLANNER_H_

#include "Footprint.h"
#include<iostream>
#include<cnoid/EigenTypes>
#include<vector>
using namespace std;
using namespace cnoid;

class FootPrintPlanner{
  //これで着地点を設定する
  //着地点は「SSPの支持位置姿勢」、
  //シミュレーション開始時の足の位置姿勢」、
  //シミュレーション終了時の足の位置姿勢」
  //をそれぞれ指定
public:
  Footprints support_point;     //SSPの支持足の位置姿勢
  Footprints initial_footpoint; //シミュレーション開始時の足の位置姿勢
  Footprints end_footpoint;     //シミュレーション終了時の足の位置姿勢

  void InitializeFootPrintPlanner(vector<vector<double>> sup_data, int sup_RoL,
                                  vector<vector<double>> ini_data,
                                  vector<vector<double>> end_data);
};
#endif
