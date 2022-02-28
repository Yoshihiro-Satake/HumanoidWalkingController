#include "Trajectory_planner.h"
#include<iostream>
#include<cmath>
#include<cnoid/EigenTypes>
#include<vector>
#include<deque>
using namespace std;
using namespace cnoid;

void TrajectoryPlanner::InitializeTrajectoryPlanner(FootPrintPlanner _footprint_planner, Vector3 CoMin, Vector3 vCoMin, double _Tssp, double _zVRP, double _dt){
  //パラメータ設定をする関数

  //FootPrintPlannerからの入力
  for(int i=0;i<_footprint_planner.support_point.datas.size();i++){
    support_point.push_back(_footprint_planner.support_point.datas[i]);
  }
  for(int i=0;i<_footprint_planner.initial_footpoint.datas.size();i++){
    initial_footpoint.push_back(_footprint_planner.initial_footpoint.datas[i]);
    end_footpoint.push_back(_footprint_planner.end_footpoint.datas[i]);
  }

  //単脚支持期の時間を設定
  Tssp = _Tssp;
  //ΔZvrpを設定
  zVRP = _zVRP;
  //重心の初期位置を設定
  CoM_d = CoMin;
  //重心の初速度を設定
  vCoM_d = vCoMin;
  //時定数を設定
  b = sqrt(zVRP/g);
  //制御サイクルを設定
  dt = _dt;
  //Tssp内の時間をカウントするために初期化
  t = 0.0;
  //歩数を初期化
  n = 0;
}

void TrajectoryPlanner::SetCMPandCP(){
  //CMP,VRP,CPin,CPenを求める。
  //これがないと軌道が求められない

  //着地点を目標CMP位置に設定
  Nmax = support_point.size();
  for(int i=0;i<Nmax;i++){
    CMP_d.push_back(support_point[i].foot_position);
    //式7
    VRP_d.push_back(CMP_d[i] + Vector3(0.0, 0.0, zVRP));
  }

  //CPの終端位置を最後のVRPに一致させる
  CPen_d.push_back(VRP_d[Nmax-1]);

  //各CPin_d,CPen_dを求める
  for(int i=Nmax-1;i>0;i--){
    //式8
    CPin_d.push_front(VRP_d[i] + exp(-Tssp/b)*(CPen_d[0] - VRP_d[i]));
    CPen_d.push_front(CPin_d[0]);
  }
  //CPの初期位置を設定
  CPin_d.push_front(CoM_d);
}

void TrajectoryPlanner::SSPtrajectory(){
  //満を持して登場、軌道生成関数
  //オンラインで軌道生成する。

  //以下の値は時刻tのときの値

  //重心位置を少し動かす
  CoM_d += vCoM_d*dt;
  //CPを少し動かす.式9
  CP_d = VRP_d[n] + exp((t-Tssp)/b)*(CPen_d[n] - VRP_d[n]);
  //CP速度を求める。式5
  vCP_d = 1/b*(CP_d - VRP_d[n]);
  //重心速度を求める。式2
  vCoM_d = -1/b*(CoM_d - CP_d);
}

void TrajectoryPlanner::LegTrajectory(){
  //足首軌道生成
  //サイクロイドで生成。階段など床が平らでない場合は軌道背性ができないけど、それは制御でなんとかして

  i = support_point[n].RightOrLeft; //支持脚がどちらか判定。簡略化のためiとおく
  Vector3 pre_landing_point;            //前の着地点
  Vector3 next_landing_point;           //次の着地点
  Matrix3d pre_landing_R;               //前の足の姿勢
  Matrix3d next_landing_R;              //次の足の姿勢
  Vector3 pre_landing_Ankle_d;
  Vector3 next_landing_Ankle_d;
  //最初の1歩（0歩目）は遊脚開始点が足の初期位置
  if(n == 0){
    pre_landing_point = initial_footpoint[1-i].foot_position;
    pre_landing_R = initial_footpoint[1-i].R;
    next_landing_point = support_point[1].foot_position;
    next_landing_R = support_point[1].R;
  }
  else if(n == Nmax-1){
    pre_landing_point = support_point[n-1].foot_position;
    pre_landing_R = support_point[n-1].R;
    next_landing_point = end_footpoint[1-i].foot_position;
    next_landing_R = end_footpoint[1-i].R;
  }
  else{
    pre_landing_point = support_point[n-1].foot_position;
    pre_landing_R = support_point[n-1].R;
    next_landing_point = support_point[n+1].foot_position;
    next_landing_R = support_point[n+1].R;
  }
  pre_landing_Ankle_d = pre_landing_point - pre_landing_R*Vector3(0,0,-0.1);
  next_landing_Ankle_d = next_landing_point - next_landing_R*Vector3(0,0,-0.1);

  //支持脚
  //足首位置を計算
  Ankle_d[i] = support_point[n].foot_position - support_point[n].R*Vector3(0,0,-0.1);
  //足の姿勢を設定
  FootRotation_d[i] = support_point[n].R;

  //遊脚
  //サイクロイド
  pi = 3.141592;
  double theta = t*2*pi/Tssp;
  Ankle_d[1-i] = (next_landing_Ankle_d - pre_landing_Ankle_d)/(2*pi)*(theta - sin(theta)) + pre_landing_Ankle_d;
  Ankle_d[1-i][2] = max(next_landing_Ankle_d[2], pre_landing_Ankle_d[2])*0.2*(1-cos(theta)) + pre_landing_Ankle_d[2];
  //今は回転は考慮しないことにする
  //１軸回転法で計画する
  FootRotation_d[1-i] = support_point[n].R;
}

void TrajectoryPlanner::AllTrajectoryPlanner(){
  //nが最大歩幅Nmaxに到達したら値を更新しない
  if(n>=Nmax){
    t += 0.001;
  }
  else{
    //時間を進める。
    t += 0.001;
    if(t < Tssp){
      //SSPの重心とCPの軌道生成
      SSPtrajectory();
      //遊脚軌道生成
      LegTrajectory();
    }
  }
  //tがTssp-0.001に到達したらtを0にリセット、歩数nを1つ進める
  if(t >= Tssp){
    t = 0.0;
    n += 1;
  }
}
