#pragma once 
#ifndef MOTIONCONTROL
#define MOTIONCONTROL
#include"XDDPHMI.h"
#include"CommandID.h"
#include"RWMutex.h"
#include<thread>
struct FeedBackDataStruct
{
    INT64 KernelVersion;
    INT32 COE_Slave_Num;
    INT32 IO_Slave_Num;
    FBStatus FBsArr[255];
    FBAxisState AxisState[32];
    UINT32 DO[16];
};
class PLCOpenMC
{
private:
    XDDP_HMI XddpMiddleWare;
    FeedBackDataStruct FeedBackData;
    m_RWMutex FeedBackMtx;
    std::thread Thread_Recv;
    bool bThread_Recv_Flag;
    void Thread_Recv_fun();
public:
    PLCOpenMC();
    ~PLCOpenMC();
    int Send(void *pData,const int len);
    INT32 GetPosition(int AxisId);
    INT32 GetVelocity(int AxisId);
    INT32 GetTorque(int AxisId);
    INT32 GetIO(int IOId);
    INT32 GetDriverNum();
    INT32 GetIONum();
    FBStatus GetFBStatus(int ID);
    
};



#endif