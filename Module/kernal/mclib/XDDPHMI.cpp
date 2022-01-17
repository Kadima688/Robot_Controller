#include"XDDPHMI.h"
#include "XddpPort.h"
#include<unistd.h>
EndPort *XDDP_HMI::ToKernel=nullptr;
EndPort *XDDP_HMI::FromKernel=nullptr;
int XDDP_HMI::PidKernel=-1;
int XDDP_HMI::PidMaster=-1;
XDDP_HMI::XDDP_HMI()
{
    if(PidKernel<0) PidKernel=OpenProcess("/home/pi/SoftControl/mc-kernel/build/Kernel");
    if(PidMaster<0) PidMaster=OpenProcess("/home/pi/SoftControl/Master");
    if(ToKernel==nullptr) ToKernel=new XDDPEndPort("ToKernel",XDDP_H2K_HMI_PORT);
    if(FromKernel==nullptr) FromKernel=new XDDPEndPort("FromKernel",XDDP_K2H_HMI_PORT);


}
XDDP_HMI::~XDDP_HMI()
{
    if(ToKernel!=nullptr) delete ToKernel;
    if(FromKernel!=nullptr) delete FromKernel;

}
int XDDP_HMI::OpenProcess(const  char* ProcessPath)
{
    int PidProcess=fork();
    if(PidProcess==0)
    {
        setpgid(0,0);
        if(execv(ProcessPath,NULL))
            exit(-1);
    }

    return PidProcess;
}
int XDDP_HMI::SendToKernel(void *pData,const int len)
{
    return ToKernel->EndPortSend(pData,len,0);
}
int  XDDP_HMI::RecvFromKernel(void *pData ,const int len)
{
    return FromKernel->EndPortRecv(pData,len,0);
}