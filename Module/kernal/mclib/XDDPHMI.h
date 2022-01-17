#pragma once
#ifndef XDDP_HMI__FLAG
#define XDDP_HMI__FLAG
class EndPort;
class XDDP_HMI
{
    const int  XDDP_K2H_HMI_PORT=15;
    const int  XDDP_H2K_HMI_PORT=16;
    static EndPort *ToKernel,*FromKernel;

    int OpenProcess(const  char* ProcessPath);
    

public:
    XDDP_HMI();
    ~XDDP_HMI();
    int SendToKernel(void *pData,const int len);
    int  RecvFromKernel(void *pData ,const int len);

private:
    static int PidMaster;
    static int PidKernel;
};


#endif