#include"MhMotionKernel.h"

int Mh::MhMotionkernel::OpenDevice(std::string TargetAdress){
    return IPMCOpenDevice();
}
int Mh::MhMotionkernel::OpenMCKernel(){
    return IPMCOpenMCKernel();
}
int Mh::MhMotionkernel::CloseDevice(){
    return IPMCCloseDevice();
}

int Mh::MhMotionkernel::EmergeStop(){
    return IPMCEmergeStop();
}
int Mh::MhMotionkernel::GetDriverPos(unsigned int CoE_Num,int* position){
    return IPMCGetDriverPos(CoE_Num,position);
}

int Mh::MhMotionkernel::GetAxisPosition(unsigned int nAxis,int* Position){
    return IPMCGetAxisPosition(nAxis,Position);
}

int Mh::MhMotionkernel::GetDriverState(unsigned int CoE_Num,unsigned int* value){
    return IPMCGetDriverState(CoE_Num,value);
}

int Mh::MhMotionkernel::SetAxisCommandMode(unsigned int nAxis,unsigned int mode){
    return IPMCSetAxisCommandMode(nAxis,mode);
}

int Mh::MhMotionkernel::SetVelCommand(unsigned int nAxis,int vel){
    return IPMCSetVelCommand(nAxis,vel);
}

int Mh::MhMotionkernel::SetAxisSoftLimit(unsigned int nAxis,unsigned int StopType,int Limp,int Limn){
    return IPMCSetAxisSoftLimit(nAxis,StopType,Limp,Limn);
}

int Mh::MhMotionkernel::EnableSoftLimit(unsigned int nAxis,unsigned int Enable){
    return IPMCEnableSoftlimit(nAxis,Enable);
}

int Mh::MhMotionkernel::SetAxisPositionMode(unsigned int nAxis,unsigned int PositionMode){
    return IPMCSetAxisPositionMode(nAxis,PositionMode);
}

int Mh::MhMotionkernel::SetAxisVel(unsigned int nAxis,double StartVel,double TargetVel,double EndV){
    return IPMCSetAxisVel(nAxis,StartVel,TargetVel,EndV);
}

int Mh::MhMotionkernel::SetAxisAcc(unsigned int nAxis,double Acc,double Dec){
    return IPMCSetAxisAcc(nAxis,Acc,Dec);
}

int Mh::MhMotionkernel::SetAxisJerk(unsigned int nAxis,double AccJerk,double DecJerk){
    return IPMCSetAxisJerk(nAxis,AccJerk,DecJerk);
}

int Mh::MhMotionkernel::SetInterpolationVel(double StartV,double TargetV,double EndV){
    return IPMCSetInterpolationVel(StartV,TargetV,EndV);
}

int Mh::MhMotionkernel::SetInterpolationAcc(double InterpAcc,double InterpDec){
    return IPMCSetInterpolationAcc(InterpAcc,InterpDec);
}

int Mh::MhMotionkernel::SetInterpolationJerk(double AccJerk,double DecJerk){
    return IPMCSetInterpolationJerk(AccJerk,DecJerk);
}

int Mh::MhMotionkernel::ContiOpenList(unsigned int Crd,unsigned int Axisnum,unsigned int* AxisList,unsigned int* MaxAcc){
    return IPMCContiOpenList(Crd,Axisnum,AxisList,MaxAcc);
}

int Mh::MhMotionkernel::ContiSetLookheadMode(unsigned int Crd,unsigned int enable,unsigned int LookheadSegments,double PathErr){
    return IPMCContiSetLookaheadMode(Crd,enable,LookheadSegments,PathErr);
}

int Mh::MhMotionkernel::ContiLineUnit(unsigned int Crd,unsigned int Axisnum,unsigned int* AxisList,int* Target_Pos,unsigned int posi_mode,int mark){
    return IPMCContiLineUnit(Crd,Axisnum,AxisList,Target_Pos,posi_mode,mark);
}

int Mh::MhMotionkernel::ConttiStartList(unsigned int Crd){
    return IPMCContiStartList(Crd);
}

int Mh::MhMotionkernel::ContiCloseList(unsigned int Crd){
    return IPMCContiCloseList(Crd);
}

int Mh::MhMotionkernel::ContiStopList(unsigned int Crd,unsigned int stop_mode){
    return IPMCContiStopList(Crd,stop_mode);
}

int Mh::MhMotionkernel::ContiPauseList(unsigned int Crd){
    return ContiPauseList(Crd);
}

int Mh::MhMotionkernel::PositionDrive(unsigned int nAxis,double Position){
    return IPMCPositionDrive(nAxis,Position);
}

int Mh::MhMotionkernel::StopAllAxis(unsigned int mode){
    return IPMCStopAllAxis(mode);
}