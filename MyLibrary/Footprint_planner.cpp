#include "Footprint_planner.h"
#include<iostream>
#include<cnoid/EigenTypes>
#include<vector>
using namespace std;
using namespace cnoid;

//Vector6がめちゃくちゃ使いにくいので苦肉の策でvector<vector<6>>使う
void FootPrintPlanner::InitializeFootPrintPlanner(vector<vector<double>> sup_data, int sup_RoL, vector<vector<double>> ini_data, vector<vector<double>> end_data){
  //RoLはRightOrLeftの略
  //ini_dataは0番目に右足の、1番目に左足のデータとする
  //end_dataも同様。
  for(int i=0;i<sup_data.size();i++){
    support_point.SetFootprints(sup_data[i], sup_RoL);
    sup_RoL = 1-sup_RoL;//右足左足交換。
  }
  for(int i=0;i<ini_data.size();i++){
    initial_footpoint.SetFootprints(ini_data[i], i);
    end_footpoint.SetFootprints(end_data[i], i);
  }
}
