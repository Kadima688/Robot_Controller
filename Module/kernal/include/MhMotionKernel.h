#ifndef MotionKernel_H
#define MotionKernel_H

#include<string>
#ifdef USE_KERNEL
#include<IPMCMOTION.h>
#else
#include"IPMCMOTION.h"
#endif


namespace Mh{
    class MhMotionkernel{
        public:
            //--------------打开设备进行基本的初始化
            int OpenDevice(std::string TargetAdress="localhost");//打开设备，在实现运动的控制器功能为卡分配资源 0-执行函数成功
            int OpenMCKernel();//打开运动内核 0-函数执行成功 10000-执行函数失败
            //**************
            int CloseDevice();//关闭运动控制内核，伺服下电，释放系统资源 0-执行函数成功(正常下电调用这条指令即可)
            int EmergeStop();//紧急停止所有轴
            int SetVelCommand(unsigned int nAxis,int vel);//设置轴的速度 0-执行函数成功
            int SetAxisCommandMode(unsigned int nAxis,unsigned int mode);//设置轴的模式 mode(0-CSP 1-CSV 2-CST)
            //---------------读取轴的位置信息
            int GetDriverPos(unsigned int CoE_Num,int* position);//读取指定COE从站实际位置 0-执行函数成功
            int GetAxisPosition(unsigned int nAxis,int* Position);//读取轴的逻辑位置 0-执行函数成功
            int GetDriverState(unsigned int CoE_Num,unsigned int* value);
            /*取指定COE从站的信息
            5位  运动完成信号 	0代表处于运动中，1代表处于停止状态（运动完成）；
            4位  驱动器报警   		0代表未处于报警，1代表处于报警状态；	
            3位  驱动器使能  	 0代表未处于使能，1代表处于使能状态；
            2位  原点			 0代表未处于原点，1代表处于原点状态；
            1位  硬件正限位   		0代表未处于硬件正限位，1代表处于硬件正限位；
            0位  硬件负限位   		0代表未处于硬件负限位，1代表处于硬件负限位；	
            */     
            //------------位置控制模式前对电机进行的相关初始化工作
            int SetAxisSoftLimit(unsigned int nAxis,unsigned int StopType,int Limp,int Limn);//设置各轴的软限位
            int EnableSoftLimit(unsigned int nAxis,unsigned int Enable);//使能各轴软限位
            int SetAxisPositionMode(unsigned int nAxis,unsigned int PositionMode);//设置轴的位移模式
            int SetAxisVel(unsigned int nAxis,double StartVel,double TargetVel,double EndV);//设置轴的默认速度
            int SetAxisAcc(unsigned int nAxis,double Acc,double Dec);//设置轴的加速度
            int SetAxisJerk(unsigned int nAxis,double AccJerk,double DecJerk);//设置轴的加加速度
            //----------插补之前需要进行的设置
            int SetInterpolationVel(double StartV,double TargetV,double EndV);//设置插补速度曲线速度
            int SetInterpolationAcc(double InterpAcc,double InterpDec);//设置插补速度加速度
            int SetInterpolationJerk(double AccJerk,double DecJerk);//设置插补速度曲线加加速度和加减速度
            //-------------正式插补调用的指令
            int ContiOpenList(unsigned int Crd,unsigned int Axisnum,unsigned int* AxisList,unsigned int* MaxAcc);//开启插补缓冲区
            int ContiSetLookheadMode(unsigned int Crd,unsigned int enable,unsigned int LookheadSegments,double PathErr);//设置前瞻参数
            int ContiLineUnit(unsigned int Crd,unsigned int Axisnum,unsigned int* AxisList,int* Target_Pos,unsigned int posi_mode,int mark);//连续插补中直线插补指令
            int ConttiStartList(unsigned int Crd);//开始连续插补
            //-------------停止插补运动的相关指令
            int ContiStopList(unsigned int Crd,unsigned int stop_mode);//停止连续插补运动
            int ContiCloseList(unsigned int Crd);//关闭插补缓冲区
            int ContiPauseList(unsigned int Crd);//暂停插补缓冲，配合startList可实现运动的暂停和继续运行
            //-------------单轴定长运动
            int PositionDrive(unsigned int nAxis,double Position);
            //-------------单轴连续运动
            int SetJogParam(unsigned int nAxis, double TargetVel, double LowVel, double Acc, double Jerk);//单轴连续转动之前的参数设置函数
            int Jog(unsigned int nAxis, int Dir);
            //--------------停止所有轴运动
            int StopAllAxis( unsigned int mode);    
            int StopAxis(unsigned int nAxis, unsigned int mode);//停止特定轴的运动     
    };
}

#endif