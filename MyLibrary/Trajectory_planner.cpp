#include "Trajectory_planner.h"
#include<iostream>
#include<cmath>
#include<cnoid/EigenTypes>
#include<vector>
#include<deque>
using namespace std;
using namespace cnoid;

void TrajectoryPlanner::InitializeTrajectoryPlanner(Vector3 CoMin, Vector3 vCoMin, double _Tssp, double _zVRP, double _dt){
  //パラメータ設定をする関数

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

void TrajectoryPlanner::SetCMPandCP(vector<Vector3> support_point){
  //CMP,VRP,CPin,CPenを求める。
  //これがないと軌道が求められない

  //着地点を目標CMP位置に設定
  Nmax = support_point.size();
  for(int i=0;i<Nmax;i++){
    CMP_d.push_back(support_point[i]);
    //式7
    VRP_d.push_back(CMP_d[i] + Vector3(0.0, 0.0, zVRP));
  }

  //CPの終端位置を最後のVRPに一致させる
  CPen_d.push_back(VRP_d[Nmax-1]);

  //各CPin_d,CPen_dを求める
  for(int i=Nmax-1;i>0;i--){
    //式8
    //CPin_d.insert(CPin_d.begin(),VRP_d[i] + exp(-Tssp)*(CPen_d[0] - VRP_d[i]));
    CPin_d.push_front(VRP_d[i] + exp(-Tssp/b)*(CPen_d[0] - VRP_d[i]));
    //CPen_d.insert(CPin_d.begin(),CPin_d[0]);
    CPen_d.push_front(CPin_d[0]);
  }
  //CPの初期位置を設定
  //CPin_d.insert(CPin_d.begin(),CoM_d);
  CPin_d.push_front(CoM_d);
}

void TrajectoryPlanner::SSPtrajectory(){
  //満を持して登場、軌道生成関数
  //オンラインで軌道生成する。

  //nが最大歩幅Nmaxに到達したら値を更新しない
    //時間を進める。
  t += 0.001;
    //以下の値は時刻tのときの値

    //重心位置を少し動かす
  CoM_d += vCoM_d*dt;
    //CPを少し動かす.式9
  CP_d = VRP_d[n] + exp((t-Tssp)/b)*(CPen_d[n] - VRP_d[n]);
    //CP速度を求める。式5
  vCP_d = 1/b*(CP_d - VRP_d[n]);
    //重心速度を求める。式2
  vCoM_d = -1/b*(CoM_d - CP_d);
    //tがTssp-0.001に到達したらtを0にリセット、歩数nを1つ進める
  if(t >= Tssp){
    t = 0.0;
    n += 1;
  }
}
