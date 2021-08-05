#ifndef MotionKernel_H
#define MotionKernel_H

#include<string>
#include<IPMCMOTION.h>
namespace Mh{
    class MhMotionkernel{
        public:
            int OpenDevice(std::string TargetAdress="localhost");//打开设备，在实现运动的控制器功能为卡分配资源 0-执行函数成功
            int CloseDevice();//关闭运动控制内核，伺服下电，释放系统资源 0-执行函数成功
            int SetVelCommand(unsigned int nAxis,int vel);//设置轴的速度 0-执行函数成功
            int SetAxisCommandMode(unsigned int nAxis,unsigned int mode);//设置轴的模式 mode(0-CSP 1-CSV 2-CST)
            int GetDriverPos(unsigned int CoE_Num,int* position);//读取指定COE从站实际位置 0-执行函数成功
            int GetAxisPosition(unsigned int nAxis,int* Position);//读取轴的逻辑位置 0-执行函数成功
            /*取指定COE从站的信息
            5位  运动完成信号 	0代表处于运动中，1代表处于停止状态（运动完成）；
            4位  驱动器报警   		0代表未处于报警，1代表处于报警状态；	
            3位  驱动器使能  	 0代表未处于使能，1代表处于使能状态；
            2位  原点			 0代表未处于原点，1代表处于原点状态；
            1位  硬件正限位   		0代表未处于硬件正限位，1代表处于硬件正限位；
            0位  硬件负限位   		0代表未处于硬件负限位，1代表处于硬件负限位；	
            */
            int GetDriverState(unsigned int CoE_Num,unsigned int* value);
    };
}

#endif