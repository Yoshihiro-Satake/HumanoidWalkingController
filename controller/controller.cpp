#include "Trajectory_planner.h"
#include "IK.h"
//#include "Footprint_planner.h"
#include<cnoid/SimpleController>
#include<cnoid/Body>
#include<cnoid/EigenTypes>
#include<cnoid/JointPath>
#include<cnoid/BasicSensors>
#include<cnoid/Link>
#include<iostream>
#include<fmt/format.h>
#include<vector>
#include<fstream>

using namespace cnoid;
using namespace std;
using fmt::format;

//私が作った3次元用2足ロボットを動かす。
//オンラインで軌道生成する
//事前に着地点は決めておく

class Biped_Online_Controller_test : public SimpleController
{
  //Bodyクラス
  BodyPtr ioBody;
  //ジョイントパスクラス
  JointPath supLeg2swingLeg;
  //シミュレータクラス
  SimpleControllerIO* io;
  //シミュレータに関する値
  double startTime;
  double total_t = 0.0;
  //テスト：Footprint_planner.cppのバグ発見
  FootPrintPlanner footprint_planner;
  //テスト：Trajectory_planner.cppのバグを発見するため
  TrajectoryPlanner trajectory_planner;
  //テスト：IKのバグを発見するため
  IK iksolver;
  //値確認用
  ofstream ofs;

  //以下足を踏み出す位置に関する値
  //Vector6がめちゃくちゃ使いにくいので苦肉の策でvector<vector>
  //もはやVector3を２倍用意したほうがいいのか？
  vector<vector<double>> support_footpoint = {{0.0,   0.15, 0.0, 0.0, 0.0, 0.0},
                                              {0.15, -0.15, 0.0, 0.0, 0.0, 0.0},
                                              {0.15,  0.15, 0.0, 0.0, 0.0, 0.0}};
  int sup_RoL =  1; //左足支持からスタート
  vector<vector<double>> initial_footpoint = {{0.0, -0.15, 0.0, 0.0, 0.0, 0.0},
                                              {0.0,  0.15, 0.0, 0.0, 0.0, 0.0}};
  vector<vector<double>> end_footpoint = {{0.15, -0.15, 0.0, 0.0, 0.0, 0.0},
                                          {0.15,  0.15, 0.0, 0.0, 0.0, 0.0}};

  //以下軌道生成に必要なパラメータ
  const double zVRP = 0.75;
  Vector3 CoMin = Vector3(0.0, 0.0, zVRP);
  Vector3 vCoMin = Vector3(0.0, 0.0, 0.0);
  const double Tssp = 0.800;
  const double dt = 0.001;
  const double pi = 3.141592;
  //IKに必要なロボットの固有パラメータ
  const double l1 = 0.3;  //股関節から膝までの長さ
  const double l2 = 0.3;  //膝から足首まで
  Vector3 west2hip2 = Vector3(0.0, -0.15, -0.1);
  Vector3 west2hip8 = Vector3(0.0,  0.15, -0.1);
  Vector3 west2CoM = Vector3(0.0, 0.0, 0.0);

public:
  virtual bool initialize(SimpleControllerIO* io) override
  {
    //シミュレーションアイテム生成
    this->io = io;
    ostream& os = io->os();
    //ボディオブジェクト生成
    ioBody = io->body();

    Link* joint = ioBody->joint(1);
    io->enableInput(joint, Link::LinkPosition);

    for(int i=2; i < ioBody->numJoints(); ++i){
      //リンクオブジェクト生成
      Link* joint = ioBody->joint(i);
      //アクチュエーションモードを関節角度に指定
      joint->setActuationMode(Link::JointAngle);
      //関節角度をコントローラの入出力可能にする
      io->enableIO(joint);
      //リンク位置をコントローラに入力して値を確認できるようにする
      io->enableInput(joint, Link::LinkPosition);
    }

    ofs.open("/home/yoshihiro/choreonoid/ext/Humanoid/Trajectory.csv");

    //着地位置計画クラスを初期化
    footprint_planner.InitializeFootPrintPlanner(support_footpoint, sup_RoL, initial_footpoint, end_footpoint);

    //軌道生成クラスを初期化
    trajectory_planner.InitializeTrajectoryPlanner(footprint_planner, CoMin, vCoMin, Tssp, zVRP, dt);
    trajectory_planner.SetCMPandCP();

    //IKを初期化
    iksolver.InitializeIK(l1, l2, west2hip2, west2hip8, west2CoM);
    return true;
  }
  virtual bool control() override
  {
    if(startTime == 0.0){
      //開始時間が0.0の場合
      //開始時間に現在のシミュレーション時間を設定
      startTime = io->currentTime();
    }
    //リンクオブジェクト生成
    Link* joint[14];
    for(int i=1; i <= ioBody->numJoints(); ++i){
      joint[i] = ioBody->joint(i);
    }
    //テスト用に角度を固定する
    joint[2]->q_target() = 0.0;
    joint[3]->q_target() = 0.0;
    joint[4]->q_target() = -30*pi/180;
    joint[5]->q_target() = 60*pi/180;
    joint[6]->q_target() = -30*pi/180;
    joint[7]->q_target() = 0.0;
    joint[8]->q_target() = 0.0;
    joint[9]->q_target() = 0.0;
    joint[10]->q_target() = -30*pi/180;
    joint[11]->q_target() = 60*pi/180;
    joint[12]->q_target() = -30*pi/180;
    joint[13]->q_target() = 0.0;

    trajectory_planner.AllTrajectoryPlanner();
    iksolver.IKLeg(trajectory_planner.Ankle_d, trajectory_planner.FootRotation_d, trajectory_planner.CoM_d);

    //cout << "," << trajectory_planner.t << "," << trajectory_planner.CoM_d[0] << endl;
    //cout << "|" << trajectory_planner.VRP_d[0][1] << "," << trajectory_planner.CPin_d[0][1] << "|" << trajectory_planner.VRP_d[1][1] << "," << trajectory_planner.CPin_d[1][1] << "|" << trajectory_planner.CPin_d[2][1] << endl;
    cout << "|" << trajectory_planner.FootRotation_d[0](0,0) << endl;
    //ofs << trajectory_planner.Ankle_d[0][0] << "," << trajectory_planner.Ankle_d[0][1] << "," << trajectory_planner.Ankle_d[0][2] << "," << trajectory_planner.CoM_d[0] << endl;
    ofs << iksolver.q[1] << "," << iksolver.q[5] << endl;

    //時間を進める
    total_t += 0.001;
    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Biped_Online_Controller_test)
