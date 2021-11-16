#include"MhMotionKernel.h"

int Mh::MhMotionkernel::OpenDevice(std::string TargetAdress){
    #ifdef USE_KERNEL
        return IPMCOpenDevice();
    #else  
        return 0;
    #endif
}
int Mh::MhMotionkernel::OpenMCKernel(){
    #ifdef USE_KERNEL
        return IPMCOpenMCKernel();
    #else
        return 0;
    #endif
}
int Mh::MhMotionkernel::CloseDevice(){
    #ifdef USE_KERNEL
        return IPMCCloseDevice();
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::EmergeStop(){
    #ifdef USE_KERNEL
        return IPMCEmergeStop();
    #else
        return 0;
    #endif
}
int Mh::MhMotionkernel::GetDriverPos(unsigned int CoE_Num,int* position){
    #ifdef USE_KERNEL
        return IPMCGetDriverPos(CoE_Num,position);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::GetAxisPosition(unsigned int nAxis,int* Position){
    #ifdef USE_KERNEL
        return IPMCGetAxisPosition(nAxis,Position);
    #else
        return 0;
    #endif
}

int Mh::MhMotionkernel::GetDriverState(unsigned int CoE_Num,unsigned int* value){
    #ifdef USE_KERNEL
        return IPMCGetDriverState(CoE_Num,value);
    #else
        return 0;
    #endif
}

int Mh::MhMotionkernel::SetAxisCommandMode(unsigned int nAxis,unsigned int mode){
    #ifdef USE_KERNEL
        return IPMCSetAxisCommandMode(nAxis,mode);
    #else
        return 0;
    #endif
}

int Mh::MhMotionkernel::SetVelCommand(unsigned int nAxis,int vel){
    #ifdef USE_KERNEL
        return IPMCSetVelCommand(nAxis,vel);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::SetAxisSoftLimit(unsigned int nAxis,unsigned int StopType,int Limp,int Limn){
    #ifdef USE_KERNEL
        return IPMCSetAxisSoftLimit(nAxis,StopType,Limp,Limn);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::EnableSoftLimit(unsigned int nAxis,unsigned int Enable){
    #ifdef USE_KERNEL
        return IPMCEnableSoftlimit(nAxis,Enable);
    #else
        return 0;
    #endif
}

int Mh::MhMotionkernel::SetAxisPositionMode(unsigned int nAxis,unsigned int PositionMode){
    #ifdef USE_KERNEL
        return IPMCSetAxisPositionMode(nAxis,PositionMode);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::SetAxisVel(unsigned int nAxis,double StartVel,double TargetVel,double EndV){
    #ifdef USE_KERNEL
        return IPMCSetAxisVel(nAxis,StartVel,TargetVel,EndV);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::SetAxisAcc(unsigned int nAxis,double Acc,double Dec){
    #ifdef USE_KERNEL
        return IPMCSetAxisAcc(nAxis,Acc,Dec);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::SetAxisJerk(unsigned int nAxis,double AccJerk,double DecJerk){
    #ifdef USE_KERNEL
        return IPMCSetAxisJerk(nAxis,AccJerk,DecJerk);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::SetInterpolationVel(double StartV,double TargetV,double EndV){
    #ifdef USE_KERNEL
        return IPMCSetInterpolationVel(StartV,TargetV,EndV);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::SetInterpolationAcc(double InterpAcc,double InterpDec){
    #ifdef USE_KERNEL
        return IPMCSetInterpolationAcc(InterpAcc,InterpDec);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::SetInterpolationJerk(double AccJerk,double DecJerk){
    #ifdef USE_KERNEL
        return IPMCSetInterpolationJerk(AccJerk,DecJerk);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::ContiOpenList(unsigned int Crd,unsigned int Axisnum,unsigned int* AxisList,unsigned int* MaxAcc){
    #ifdef USE_KERNEL
        return IPMCContiOpenList(Crd,Axisnum,AxisList,MaxAcc);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::ContiSetLookheadMode(unsigned int Crd,unsigned int enable,unsigned int LookheadSegments,double PathErr){
    #ifdef USE_KERNEL
        return IPMCContiSetLookaheadMode(Crd,enable,LookheadSegments,PathErr);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::ContiLineUnit(unsigned int Crd,unsigned int Axisnum,unsigned int* AxisList,int* Target_Pos,unsigned int posi_mode,int mark){
    #ifdef USE_KERNEL
        return IPMCContiLineUnit(Crd,Axisnum,AxisList,Target_Pos,posi_mode,mark);
    #else
        return 0;
    #endif
}

int Mh::MhMotionkernel::ConttiStartList(unsigned int Crd){
    #ifdef USE_KERNEL
        return IPMCContiStartList(Crd);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::ContiCloseList(unsigned int Crd){
    #ifdef USE_KERNEL
        return IPMCContiCloseList(Crd);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::ContiStopList(unsigned int Crd,unsigned int stop_mode){
    #ifdef USE_KERNEL
        return IPMCContiStopList(Crd,stop_mode);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::ContiPauseList(unsigned int Crd){
    #ifdef USE_KERNEL
        return ContiPauseList(Crd);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::PositionDrive(unsigned int nAxis,double Position){
    #ifdef USE_KERNEL
        return IPMCPositionDrive(nAxis,Position);
    #else 
        return 0;
    #endif
}

int Mh::MhMotionkernel::SetJogParam(unsigned int nAxis, double TargetVel, double LowVel, double Acc, double Jerk){
    #ifdef USE_KERNEL
        return IPMCSetAxisJogParam(nAxis,TargetVel,LowVel,Acc,Jerk);
    #else
        return 0;
    #endif
}

int Mh::MhMotionkernel::Jog(unsigned int nAxis, int Dir){
    #ifdef USE_KERNEL
        return IPMCJog(nAxis,Dir);
    #else
        return 0;
    #endif
}

int Mh::MhMotionkernel::StopAxis(unsigned int nAxis, unsigned int mode){
    #ifdef USE_KERNEL
        return IPMCStopAxis(nAxis,mode);
    #else
        return 0;
    #endif
}

int Mh::MhMotionkernel::StopAllAxis(unsigned int mode){
    #ifdef USE_KERNEL
        return IPMCStopAllAxis(mode);
    #else 
        return 0;
    #endif
}