#ifndef MOTIONINSTRUCTION_H
#define MOTIONINSTRUCTION_H

#include"PLCOpenMotion.h"
#include"MotionControl.h"
#include"FBIdManager.h"
#include<vector>
#include"ControllerData.h"
#include"daraDeclaration.h"
//指令单轴绝对运动参数结构体
struct SingleAxisMoveAbsolute_Parameter
{
    int Axis;
    bool Excute=true;
    bool ContinousUpdate=true;
    double Pos;
    double Vel;
    double Acc;
    double Dec;
    double Jerk;
    MC_DIRECTION Direction=mcPositiveDirection;//目前用不到
    MC_BUFFER_MODE BufferMode=mcAborting;
};
std::vector<double> SCARAAngleToPulse(AXISPOS_SCARA angle,Mh::MhIndustrialSCARA* RobotSCARA);
#endif