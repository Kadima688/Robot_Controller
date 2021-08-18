#ifndef MhIndustrialRobotPathPlan_H
#define MhIndustrialRobotPathPlan_H

#include<eigen3/Eigen/Dense>
#include<map>
#include"daraDeclaration.h"

namespace Mh{
class MhIndustrialRobotPathPlan{
    public:
        MhIndustrialRobotPathPlan();
        void set_LINE_PATH_DIFF(double diff){LINE_PATH_DIFF=diff;};
        double get_LINE_PATH_DIFF(){return LINE_PATH_DIFF;};
        void PathPlan_PTP(AXISPOS_SCARA pos,std::map<int,std::vector<double>>& record);
    private:
        double LINE_PATH_DIFF;//直线插补运动的最短距离
};
}

#endif