#ifndef TRAJECTORY_PLANNER_H_
#define TRAJECTORY_PLANNER_H_

#include "Footprint.h"
#include "Footprint_planner.h"
#include<iostream>
#include<cnoid/EigenTypes>
#include<vector>
#include<deque>
using namespace std;
using namespace cnoid;

class TrajectoryPlanner{
public:
  //desired,つまり計画した値を格納する変数は一番最後に_dをつける（例：CoM_d）
  //初期値は後ろにinをつける（例：CPin_d）
  //終端の値は後ろにenをつける（例：CPen_d）

  //重心に関するパラメータ
  Vector3  CoM_d;           //目標重心位置
  Vector3 vCoM_d;           //目標重心速度
  //CPに関するパラメータ
  Vector3   CP_d;           //目標CP位置
  Vector3  vCP_d;           //目標CP速度
  deque<Vector3> CPin_d;    //目標CP初期位置
  deque<Vector3> CPen_d;    //目標CP終端位置
  //ZMP,CMP,に関するパラメータ
  vector<Vector3>  CMP_d;   //目標CMP位置
  vector<Vector3>  VRP_d;   //目標VRP位置
  //ゆう客に関するパラメータ
  vector<Vector3> Ankle_d[2]; //目標足首座標
  Matrix3d FootRotation_d[2]; //目標足姿勢
  //以下がFootstepplannerからの入力となる
  vector<FootprintData> support_point;     //支持脚位置
  vector<FootprintData> initial_footpoint; //足の初期位置
  vector<FootprintData> end_footpoint;     //足の終端位置

  //その他歩行に関するパラメータ
  int Nmax;                //歩数
  int n;                   //今何歩目かカウントする
  double zVRP;             //ΔZvrp
  double Tssp;             //SSPの時間
  double b;                //時定数
  double dt;               //制御サイクル
  double t;                //1歩内の時間、つまり0<=t<=Tssp
  //定数
  double g = 9.81;
  double pi = 3.141592;

  void InitializeTrajectoryPlanner(FootPrintPlanner _footprint_planner, Vector3 CoMin, Vector3 vCoMin, double _Tssp, double _zVRP, double _dt);
  void SetCMPandCP();
  void SSPtrajectory();
  void LegTrajectory();
  void AllTrajectoryPlanner();

};
#endif
