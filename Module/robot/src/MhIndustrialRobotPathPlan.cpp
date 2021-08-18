#include"MhIndustrialRobotPathPlan.h"

Mh::MhIndustrialRobotPathPlan::MhIndustrialRobotPathPlan()
:LINE_PATH_DIFF(0.01)
{   
}

void Mh::MhIndustrialRobotPathPlan::PathPlan_PTP(AXISPOS_SCARA pos,std::map<int,std::vector<double>>& record){
    std::vector<double> pos_record;
    pos_record.push_back(pos.a1);pos_record.push_back(pos.a2);pos_record.push_back(pos.d);pos_record.push_back(pos.a4);
    record.insert(std::pair<int,std::vector<double>> (0,pos_record));
}