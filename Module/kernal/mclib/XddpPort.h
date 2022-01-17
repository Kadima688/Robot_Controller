#pragma once 
#ifndef ENDPORTDEFINE
#define ENDPORTDEFINE
#include"TypeManager.h"
#include<string>
struct EndPort
{
protected:
    const INT INVALID_SOCKET_DESCRIPT=-1;
    std::string EndPortDescritption;
    UINT32 EndPortNum;
public:
    EndPort(std::string EndPortDefine,UINT32 PortNum);

    virtual INT EndPortInit()=0;
    virtual INT EndPortClose()=0;
    virtual INT EndPortSend(PVOID pData,UINT32 len,INT Flag)=0;
    virtual INT EndPortRecv(PVOID RecvBuffer,UINT32 RecvLen,INT Flag)=0;
    virtual ~EndPort(){};
};

//use for xddp communicate
class XDDPEndPort:public EndPort
{
private:
    INT XDDPPortSocket;
    
public:
    XDDPEndPort(std::string EndPortDefine,UINT32 EndPortNum);
    virtual ~XDDPEndPort();
    virtual INT EndPortInit()override;
    virtual INT EndPortClose()override;
    virtual INT EndPortSend(PVOID pData,UINT32 len,INT Flag)override;
    virtual INT EndPortRecv(PVOID RecvBuffer,UINT32 RecvLen,INT Flag)override;
};




#endif