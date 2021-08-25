#include<iostream>
#include"MhIndustrialSCARA.h"
#include<thread>
#include"VisualServo.h"

void Controlthread(Mh::MhIndustrialSCARA* RobotSCARA){
    int retn;//函数返回值
    int hasEnable=0;//判断是否已经上电
    bool first_PTP=0;
    while (true)
    {
        if(RobotSCARA->Dem2ConData.emergeStop==0){
            if(RobotSCARA->Dem2ConData.enableState==1){
                if(hasEnable==0){
                    retn=RobotSCARA->OpenDevice();
                    RobotSCARA->set_retn(retn,Mh::OPENDEVICE);
                    retn=RobotSCARA->OpenMCKernel();
                    RobotSCARA->set_retn(retn,Mh::OPENMCKERNEL);
                    //判断电机是否已经处于使能状态
                    bool enable=false;
                    while(!enable){       
                        unsigned int value[RobotSCARA->get_nDof()];
                        int n=0;
                        for(int i=0;i<RobotSCARA->get_nDof();++i){
                            retn=RobotSCARA->GetDriverState(i,&value[i]);
                            RobotSCARA->set_retn(retn,Mh::GETDRIVERSTATE);
                            #ifdef USE_KERNEL
                                if(value[i]==8){
                                    n++;
                                }
                            #else 
                                n=3;
                            #endif
                        }
                        if(n==3){
                            std::cout<<"伺服上电"<<std::endl;
                            enable=true;RobotSCARA->ConChargeData.isEnable=1;
                            RobotSCARA->RobotMotorInitial();
                            RobotSCARA->RobotDynInitial();
                        }
                    }
                    hasEnable=1;
                }
                //在这里开始添加一条PTP指令，每次只执行一次
                if(first_PTP==0){
                    //开始执行PTP指令
                    AXISPOS_SCARA des_axispos={-39.9107,84.2493,54.7567,142.57737};
                    std::map<int,std::vector<double>> record;
                    RobotSCARA->path_plan.PathPlan_PTP(des_axispos,record);
                    //设置插补的运动信息
                    // RobotSCARA->RobotInterpolationDynInitial();
                    //开启插补缓冲区、设置速度前瞻
                    // RobotSCARA->RobotOpenConti();
                    //开始具体的运动
                    // RobotSCARA->FollowPathMove(record,Mh::PTP);//目前采用单轴定长运动的方式让机器人达到视觉伺服开始之前期望的位置
                    first_PTP=1;
                }
                //视觉伺服
                if(RobotSCARA->ConChargeData.startServo==1){
                    if(RobotSCARA->ConChargeData.endServo==0){
                        std::thread VisualServoThread(VisualServoSCARA,RobotSCARA);
                        VisualServoThread.detach();
                        RobotSCARA->ConChargeData.endServo=1;
                    }
                }
            }
            else{
                if(hasEnable==1){
                    std::cout<<"伺服下电"<<std::endl;
                    RobotSCARA->RobotCloseConti();
                    int retn=RobotSCARA->CloseDevice();
                    RobotSCARA->set_retn(retn,Mh::CLOSEDEVICE);
                    hasEnable=0;
                }
                //伺服下电状态下的操作
                /*
                1、退出连续插补
                2、停止程序运行
                3、调用closeDevice关闭设备
                4、hasEnable置为0
                */
            }
        }
        else{
            /*
            急停状态下的操作
            1、停止程序
            2、退出连续插补
            3、停止程序与运行
            4、结束视觉伺服
            5、调用emergeStop
            6、调用closeDevice
            */
        }
    }
}