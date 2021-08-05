#include"MhMotionKernel.h"

int Mh::MhMotionkernel::OpenDevice(std::string TargetAdress){
    return IPMCOpenDevice();
}

int Mh::MhMotionkernel::CloseDevice(){
    return IPMCCloseDevice();
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
