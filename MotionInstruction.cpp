#include"MotionIntruction.h"

std::vector<double> SCARAAngleToPulse(AXISPOS_SCARA angle,Mh::MhIndustrialSCARA* RobotSCARA){
    std::vector<double> Pluse;
    Pluse.resize(RobotSCARA->get_nDof());
    Pluse[0]=(angle.a1+RobotSCARA->RobotConfigData.offset2[0])/RobotSCARA->RobotConfigData.pulseEquivalent[0];
    Pluse[1]=(angle.a2+RobotSCARA->RobotConfigData.offset2[1])/RobotSCARA->RobotConfigData.pulseEquivalent[1];
    Pluse[2]=(angle.d+RobotSCARA->RobotConfigData.offset2[2])/RobotSCARA->RobotConfigData.pulseEquivalent[2];
    Pluse[3]=(angle.a4+RobotSCARA->RobotConfigData.offset2[3])/RobotSCARA->RobotConfigData.pulseEquivalent[3];
    return Pluse;
}