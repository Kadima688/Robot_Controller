#include"MotionControl.h"
#include<iostream>
PLCOpenMC::PLCOpenMC():bThread_Recv_Flag(false)
{
    bThread_Recv_Flag=true;
    Thread_Recv=std::thread(&PLCOpenMC::Thread_Recv_fun,this);
}
void PLCOpenMC::Thread_Recv_fun()
{
    FeedBackDataStruct Buffer;
    int ret=-1;
    while (bThread_Recv_Flag)
    {
        ret=XddpMiddleWare.RecvFromKernel(&Buffer,sizeof(FeedBackDataStruct));
        if(ret==sizeof(FeedBackDataStruct))
        {
            FeedBackMtx.LockWrite();
            FeedBackData=Buffer;
            FeedBackMtx.UnlockWrite();
            // std::cout<<FeedBackData.AxisState[0].CurrentVelocity<<std::endl;
            
        }
        else 
        {
            std::cout<<"Xddp read Error"<<std::endl;
        }
    }
    

}

PLCOpenMC::~PLCOpenMC()
{
    bThread_Recv_Flag=false;
    Thread_Recv.join();
}
int PLCOpenMC::Send(void *pData,const int len)
{
    return XddpMiddleWare.SendToKernel(pData,len);
}
INT32 PLCOpenMC::GetPosition(int AxisId)
{
    
    FeedBackMtx.LockRead();
    INT32 res=FeedBackData.AxisState[AxisId].CurrentPosition;
    FeedBackMtx.UnlockRead();
    return res;

}
INT32 PLCOpenMC::GetVelocity(int AxisId)
{
    FeedBackMtx.LockRead();
    INT32 res=FeedBackData.AxisState[AxisId].CurrentVelocity;
    FeedBackMtx.UnlockRead();
    return res;

}
INT32 PLCOpenMC::GetTorque(int AxisId)
{
    FeedBackMtx.LockRead();
    INT32 res=FeedBackData.AxisState[AxisId].CurrentTorque;
    FeedBackMtx.UnlockRead();
    return res;
}
INT32 PLCOpenMC::GetIO(int IOId)
{
    FeedBackMtx.LockRead();
    INT32 res=FeedBackData.DO[IOId];
    FeedBackMtx.UnlockRead();
    return res;
}
INT32 PLCOpenMC::GetDriverNum()
{
    FeedBackMtx.LockRead();
    INT32 res=FeedBackData.COE_Slave_Num;
    FeedBackMtx.UnlockRead();
    return res;
}
INT32 PLCOpenMC::GetIONum()
{
    FeedBackMtx.LockRead();
    INT32 res=FeedBackData.IO_Slave_Num;
    FeedBackMtx.UnlockRead();
    return res;

}
FBStatus PLCOpenMC::GetFBStatus(int ID)
{
    FeedBackMtx.LockRead();
    FBStatus res=FeedBackData.FBsArr[ID];
    FeedBackMtx.UnlockRead();
    return res;

}