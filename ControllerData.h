#ifndef CONTROLLERDATA_H
#define CONTROLLERDATA_H
#include"MhIndustrialSCARA.h"
#include"MhIndustrialRobotKAWASAKI.h"
#include"PLCOpenMotion.h"
#include<iostream>
#include<unistd.h>
struct ControllerData
{
    Mh::MhIndustrialSCARA robotscara;
#ifdef USE_MCKERNEL
    PLCOpenMotion motor;
    ControllerData()
    {
        for(int i= 0 ; i < 4; i++){
            motor.MC_Power(i,true,true,true);
            usleep(1000);
        }
        for (int i = 0; i < 4; i++)
        {
            motor.Mc_AddAxisToGroup(0, i, TRUE, i);
            usleep(1000);
        }

        MC_KIN_REF Type;
        Type.Device=DeviceType_ScaraJointCsp;
        motor.MC_SetKinTransform(0,&Type,mcImmediately,sizeof(Type));
        usleep(1000);
        motor.MC_GroupEnable(0,true);
    }
#endif
};

extern ControllerData controllerdata;
#endif