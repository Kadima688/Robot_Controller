#include<iostream>
#include"MhIndustrialSCARA.h"
#include<thread>
#include"VisualServo.h"
#include"ControllerData.h"
#include"MotionIntruction.h"
#include<stdlib.h>
void Controlthread(ControllerData* controllerdata){
    Mh::MhIndustrialSCARA* RobotSCARA=&(controllerdata->robotscara);
    PLCOpenMotion* motor=&(controllerdata->motor);
    int retn;//函数返回值
    int hasEnable=0;//判断是否已经上电
    int oldOvr=0;//保存旧得动态倍率参数，当动态倍率参数改变时才调用
    int keepTeach=0;//配合示教模式进行逻辑控制的变量
    bool first_PTP=0;
    while (true)
    {
        if(RobotSCARA->Dem2ConData.emergeStop==0){
            if(RobotSCARA->Dem2ConData.enableState==1){
                #ifndef USE_MCKERNEL
                if(hasEnable==0){
                    retn=RobotSCARA->OpenDevice();
                    RobotSCARA->set_retn(retn,Mh::OPENDEVICE);
                    retn=RobotSCARA->OpenMCKernel();
                    RobotSCARA->set_retn(retn,Mh::OPENMCKERNEL);
                    // 判断电机是否已经处于使能状态
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
                            // RobotSCARA->RobotDynInitial();
                            // RobotSCARA->RobotInterpolationDynInitial();
                        }
                    }
                    hasEnable=1;
                }
                #endif
                //设置动态倍率参数
                if(RobotSCARA->Dem2ConData.ovr!=oldOvr){
                    // RobotSCARA->RobotDynInitial();
                    // RobotSCARA->RobotInterpolationDynInitial();
                    oldOvr=RobotSCARA->Dem2ConData.ovr;
                }
                //手动模式
                // if(RobotSCARA->Dem2ConData.operateMode==0){
                //     // RobotSCARA->ConChargeData.curDynamic=RobotSCARA->RobotConfigData.jogspeed;
                //     // RobotSCARA->RobotDynInitial();
                //     // RobotSCARA->RobotInterpolationDynInitial();
                //     //开始示教
                //     if(RobotSCARA->Dem2ConData.coordinate!=0 && RobotSCARA->Dem2ConData.PressOrRelease==1 && keepTeach==0){
                //         //首先考虑轴关节坐标系的
                //         int dir;//运动方向
                //         if(RobotSCARA->Dem2ConData.upOrDown==0){
                //             dir=-1;
                //         }
                //         else{
                //             dir=1;
                //         }
                //         double targetVel = 1000*RobotSCARA->Dem2ConData.ovr/5;
                //         double lowVel = 0;
                //         double acc = 100*RobotSCARA->Dem2ConData.ovr/5;
                //         double jerk = 10*RobotSCARA->Dem2ConData.ovr/5;
                //         int retn=RobotSCARA->SetJogParam(RobotSCARA->Dem2ConData.coordinate-1,targetVel,lowVel,acc,jerk);
                //         RobotSCARA->set_retn(retn,Mh::IPMCJOGSETAXISPARAM);
                //         retn=RobotSCARA->Jog(RobotSCARA->Dem2ConData.coordinate-1,dir*RobotSCARA->RobotConfigData.direction[RobotSCARA->Dem2ConData.coordinate-1]);
                //         RobotSCARA->set_retn(retn,Mh::IPMCJOG);
                //         keepTeach=1;
                //     }
                //     else if(RobotSCARA->Dem2ConData.PressOrRelease==0 && keepTeach==1){
                //         //现在coordinate=0，表示按键松开了
                //         int retn=RobotSCARA->StopAxis(RobotSCARA->Dem2ConData.coordinate-1,1);
                //         RobotSCARA->set_retn(retn,Mh::STOPAXIS);
                //         keepTeach=0;
                //     }
                // }
                //在这里开始添加一条PTP指令，每次只执行一次
                if(first_PTP==0){
                    #ifdef USE_MCKERNEL
                    //在这里走一个单轴的点位运动
                    AXISPOS_SCARA zero_axispos={0,0,0,0};
                    AXISPOS_SCARA init_axispos={-39.9150,84.2464,54.75512,120.4467};
                    //将角度转换成名脉冲
                    std::vector<double> Pulse=SCARAAngleToPulse(init_axispos,RobotSCARA);
                    // double jointvelpulse[4]={5247.96,7165.07,27348.6,94.6986};
                    for(int i=0;i<RobotSCARA->get_nDof();++i){
                        //将角度转换成脉冲
                        motor->MC_MoveAbsolute(i,true,true,Pulse[i],10000,1000,1000,1,mcPositiveDirection,mcAborting);
                        // double jointpulse;  
                        // jointpulse=jointvelpulse[i]*30;//30ms这个轴应该移动得位移
                        // jointvelpulse[i]=abs(jointvelpulse[i]);
                        // motor->MC_MoveContinuousRelative(i,true,true,jointpulse,0,jointvelpulse[i],200,200,1,mcAborting);
                    }
                    
                    #else
                    #ifdef USE_KERNEL
                    // RobotSCARA->RobotDynInitial();
                    //开始执行PTP指令
                    AXISPOS_SCARA des_axispos={-39.9107,84.2493,54.7567,142.57737};
                    // AXISPOS_SCARA des_axispos={0,0,0,0};
                    std::map<int,std::vector<double>> record;
                    RobotSCARA->path_plan.PathPlan_PTP(des_axispos,record);
                    //设置插补的运动信息
                    RobotSCARA->RobotInterpolationDynInitial();
                    //开启插补缓冲区、设置速度前瞻
                    RobotSCARA->RobotOpenConti();
                    //开始具体的运动
                    RobotSCARA->FollowPathMove(record,Mh::PTP);//目前采用单轴定长运动的方式让机器人达到视觉伺服开始之前期望的位置
                    #endif
                    #endif
                    first_PTP=1;
                }
                //视觉伺服
                if(RobotSCARA->ConChargeData.startServo==1){
                    if(RobotSCARA->ConChargeData.hasServo==0){
                        if(RobotSCARA->Dem2ConData.visualServo.servotype==1){
                            std::thread VisualServoPBVSThread(VisualServoSCARA_PBVS,RobotSCARA);
                            VisualServoPBVSThread.detach();
                        }
                        else if(RobotSCARA->Dem2ConData.visualServo.servotype==2){
                            std::thread VisualServoIBVSThread(VisualServoSCARA_IBVS,RobotSCARA);
                            VisualServoIBVSThread.detach();
                        }
                        RobotSCARA->ConChargeData.hasServo=1;
                    }
                }
            }
            else{
                //伺服下电状态下的操作
                /*
                1、退出连续插补
                2、停止程序运行
                3、调用closeDevice关闭设备
                4、hasEnable置为0
                */
                if(hasEnable==1){
                    std::cout<<"伺服下电"<<std::endl;
                    #ifdef USE_MCKERNEL

                    #else
                    #ifdef USE_KERNEL
                    // RobotSCARA->RobotCloseConti();
                    int retn=RobotSCARA->CloseDevice();
                    RobotSCARA->set_retn(retn,Mh::CLOSEDEVICE);
                    #endif
                    #endif
                    hasEnable=0;
                    RobotSCARA->ConChargeData.isEnable=0;
                }           
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