#include"XddpPort.h"
#include <unistd.h>
#include <fcntl.h>
EndPort::EndPort(std::string EndPortDefine,UINT32 PortNum):EndPortDescritption(EndPortDefine),EndPortNum(PortNum)
{

}


XDDPEndPort::XDDPEndPort(std::string EndPortDefine,UINT32 EndPortNum):EndPort(EndPortDefine,EndPortNum),XDDPPortSocket(INVALID_SOCKET_DESCRIPT)
{
    EndPortInit();
}
INT XDDPEndPort::EndPortInit()
{
    char *devname = nullptr;
    struct timespec ts;
    asprintf(&devname, "/dev/rtp%u", EndPortNum);
    
    int time = 3;
    int PORT=-1;
    while (time--)
    {
        PORT = open(devname, O_RDWR);
        if (PORT > 0)
            break;
        sleep(1);
    }
    XDDPPortSocket=PORT;
    free(devname);
    return XDDPPortSocket;
}
INT XDDPEndPort::EndPortClose()
{
    if(XDDPPortSocket>=0) close(XDDPPortSocket);
    XDDPPortSocket=INVALID_SOCKET_DESCRIPT;
    return 0;
}
INT XDDPEndPort::EndPortSend(PVOID pData,UINT32 len,INT Flag)
{
    INT ret=-1;
    if(XDDPPortSocket<0)
        return -1;
    ret = write(XDDPPortSocket, pData, len);
    return ret;
}
INT XDDPEndPort::EndPortRecv(PVOID RecvBuffer,UINT32 RecvLen,INT Flag)
{
    INT ret=-1;
    if(XDDPPortSocket<0)
        return -1;
    ret = read(XDDPPortSocket, RecvBuffer, RecvLen);
    return ret;
}
XDDPEndPort::~XDDPEndPort()
{
    EndPortClose();
}
