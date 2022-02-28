#ifndef CONTROLLERDATA_H
#define CONTROLLERDATA_H
#include"MhIndustrialSCARA.h"
#include"MhIndustrialRobotKAWASAKI.h"
#include"PLCOpenMotion.h"
struct ControllerData
{
    Mh::MhIndustrialSCARA robotscara;
    PLCOpenMotion motor;
    
};

extern ControllerData controllerdata;
#endif