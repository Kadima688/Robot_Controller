#ifndef CONTROLLERDATA_H
#define CONTROLLERDATA_H
#include"MhIndustrialSCARA.h"
#include"MhIndustrialRobotKAWASAKI.h"
#include"PLCOpenMotion.h"
struct ControllerData
{
    Mh::MhIndustrialSCARA robotscara;
    PLCOpenMotion motor;
    // ControllerData()
    // {
    //     for (int i = 0; i < 3; i++)
    //     {
    //         motor.Mc_AddAxisToGroup(0, i, TRUE, i);
    //     }
    //     MC_KIN_REF Type;
    //     Type.Device=DeviceType_ScaraJointCsp;
    //     motor.MC_SetKinTransform(0,&Type,mcImmediately,sizeof(Type));
    //     motor.MC_GroupEnable(0,true);
    // }
};

extern ControllerData controllerdata;
#endif