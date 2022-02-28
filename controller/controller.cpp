#include "Trajectory_planner.h"
#include "IK.h"
#include "Sensing_Unit.h"
#include "Stabilizer.h"
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
  //センサたち
  ForceSensorPtr LeftAnkleForceSensor;
  ForceSensorPtr RightAnkleForceSensor;
  AccelerationSensorPtr CoMAccelSensor;
  //シミュレータに関する値
  double startTime;
  double total_t = 0.0;
  //
  FootPrintPlanner footprint_planner;
  //
  TrajectoryPlanner trajectory_planner;
  //
  IK iksolver;
  //
  Sensors sensors;
  //テスト：Stabilizerのバグを発見する
  Stabilizer stabilizer;
  //値確認用
  ofstream ofs;

  //以下足を踏み出す位置に関する値
  //Vector6がめちゃくちゃ使いにくいので苦肉の策でvector<vector>
  vector<vector<double>> support_footpoint = {{0.0,   0.15, 0.0, 0.0, 0.0, 0.0},
                                              {0.15, -0.10, 0.0, 0.0, 0.0, 0.0},
                                              {0.30,  0.10, 0.0, 0.0, 0.0, 0.0},
                                              {0.45, -0.10, 0.0, 0.0, 0.0, 0.0}};
  int sup_RoL =  1; //左足支持からスタート
  vector<vector<double>> initial_footpoint = {{0.0, -0.15, 0.0, 0.0, 0.0, 0.0},
                                              {0.0,  0.15, 0.0, 0.0, 0.0, 0.0}};
  vector<vector<double>> end_footpoint = {{0.45, -0.10, 0.0, 0.0, 0.0, 0.0},
                                          {0.45,  0.10, 0.0, 0.0, 0.0, 0.0}};

  //以下軌道生成に必要なパラメータ
  const double zVRP = 0.75;
  Vector3 CoMin = Vector3(0.05, 0.15, zVRP);
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
    //センサ生成
    LeftAnkleForceSensor = ioBody->findDevice<ForceSensor>("LeftAnkleForceSensor");
    RightAnkleForceSensor = ioBody->findDevice<ForceSensor>("RightAnkleForceSensor");
    CoMAccelSensor = ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
    //上記のデバイスをコントローラに入力可能とする
    io->enableInput(LeftAnkleForceSensor);
    io->enableInput(RightAnkleForceSensor);
    io->enableInput(CoMAccelSensor);

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

    //Sensorsを初期化
    sensors.InitializeSensors(LeftAnkleForceSensor, RightAnkleForceSensor, CoMAccelSensor, CoMin);

    //Stabilizerを初期化
    stabilizer.InitializeStabilizer(CoMin);

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
    //軌道を生成
    trajectory_planner.AllTrajectoryPlanner();
    //安定化制御器
    stabilizer.Stabilize(trajectory_planner.CP_d, trajectory_planner.vCP_d, sensors.CP, sensors.vCP);
    //逆運動学
    //iksolver.IKLeg(trajectory_planner.Ankle_d, trajectory_planner.FootRotation_d, trajectory_planner.CoM_d);
    iksolver.IKLeg(trajectory_planner.Ankle_d, trajectory_planner.FootRotation_d, stabilizer.CoM);
    //ロボットへ関節角度を入力
    for(int i=2;i<14;i++){
      joint[i]->q_target() = iksolver.q[i];
    }
    //順運動学でリンク位置更新をする
    //まずは支持脚から末端リンクまでのジョイントパスを取得する
    if(trajectory_planner.i == 0){
      supLeg2swingLeg = JointPath(ioBody->link("RLEG_J6"), ioBody->link("LLEG_J6"));
    }
    else{
      supLeg2swingLeg = JointPath(ioBody->link("LLEG_J6"), ioBody->link("RLEG_J6"));
    }
    //取得したジョイントパスで支持脚をベースとして順運動学を行う
    //これをしないとベースリンク(腰リンク)が空中に固定されて足だけが動く感じになる
    supLeg2swingLeg.calcForwardKinematics(true,true);
    //ベースリンクの位置が更新されたら再度順運動学
    ioBody->calcForwardKinematics(true,true);
    //値をセンシング
    sensors.Sensing(ioBody);

    //cout << "," << trajectory_planner.t << "," << trajectory_planner.CoM_d[0] << endl;
    //cout << "|" << trajectory_planner.VRP_d[0][1] << "," << trajectory_planner.CPin_d[0][1] << "|" << trajectory_planner.VRP_d[1][1] << "," << trajectory_planner.CPin_d[1][1] << "|" << trajectory_planner.CPin_d[2][1] << endl;
    //cout << sensors.CoM[0] << "|" << sensors.vCoM[0]  << "|" << sensors.vCP[0] << "|" <<endl;
    //cout << stabilizer.VRP_mod[0] << "," << stabilizer.VRP_mod[1] << endl;
    //cout << trajectory_planner.vCP_d[0] << "," << trajectory_planner.vCP_d[1] << endl;
    cout << trajectory_planner.VRP_d[0][1] << "," << stabilizer.VRP_mod[1] << endl;
    //ofs << trajectory_planner.Ankle_d[0][0] << "," << trajectory_planner.Ankle_d[0][1] << "," << trajectory_planner.Ankle_d[0][2] << "," << trajectory_planner.CoM_d[0] << endl;
    //ofs << iksolver.q[5] << "," << iksolver.q[6] << endl;

    //時間を進める
    total_t += 0.001;
    return true;
  }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Biped_Online_Controller_test)
