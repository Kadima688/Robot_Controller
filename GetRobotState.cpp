
#include<iostream>
#include<unistd.h>
#include"MhIndustrialSCARA.h"
#include"ControllerData.h"
void GetRobotState(ControllerData* controllerdata){
    Mh::MhIndustrialSCARA* RobotSCARA=&(controllerdata->robotscara);
#ifdef USE_MCKERNEL
    while(true)
    {
        if(RobotSCARA->Dem2ConData.emergeStop==1){
            return ;
        }
        if(RobotSCARA->Dem2ConData.enableState==1){
            double Position[RobotSCARA->get_nDof()];
            for(int i=0; i<RobotSCARA->get_nDof();++i){
                Position[i]=controllerdata->motor.MC_ReadActualPosition(i,TRUE);
            }
            //计算轴关节坐标系的位置
            RobotSCARA->Con2DemData.axisPos_scara.a1=Position[0]*RobotSCARA->RobotConfigData.pulseEquivalent[0]-RobotSCARA->RobotConfigData.offset2[0];
            RobotSCARA->Con2DemData.axisPos_scara.a2=Position[1]*RobotSCARA->RobotConfigData.pulseEquivalent[1]-RobotSCARA->RobotConfigData.offset2[1];
            RobotSCARA->Con2DemData.axisPos_scara.a4=Position[3]*RobotSCARA->RobotConfigData.pulseEquivalent[3]-RobotSCARA->RobotConfigData.offset2[3];
            RobotSCARA->Con2DemData.axisPos_scara.d=Position[2]*RobotSCARA->RobotConfigData.pulseEquivalent[2]-RobotSCARA->RobotConfigData.offset2[2];//+RobotSCARA->Con2DemData.axisPos_scara.a4/360*RobotSCARA->get_a4_Compensation();
            //计算伺服电机的位置
            RobotSCARA->Con2DemData.drivePos.d1=RobotSCARA->Con2DemData.axisPos_scara.a1*RobotSCARA->RobotConfigData.direction[0]*RobotSCARA->RobotConfigData.ratio[0];
            RobotSCARA->Con2DemData.drivePos.d2=RobotSCARA->Con2DemData.axisPos_scara.a2*RobotSCARA->RobotConfigData.direction[1]*RobotSCARA->RobotConfigData.ratio[1];
            RobotSCARA->Con2DemData.drivePos.d3=RobotSCARA->Con2DemData.axisPos_scara.d*RobotSCARA->RobotConfigData.direction[2]*RobotSCARA->RobotConfigData.ratio[2];
            RobotSCARA->Con2DemData.drivePos.d4=RobotSCARA->Con2DemData.axisPos_scara.a4*RobotSCARA->RobotConfigData.direction[3]*RobotSCARA->RobotConfigData.ratio[3];
            //计算工具坐标系相对世界坐标系的位置
            std::vector<double> cartesian;
            std::vector<double> scara_input={RobotSCARA->Con2DemData.axisPos_scara.a1,RobotSCARA->Con2DemData.axisPos_scara.a2,RobotSCARA->Con2DemData.axisPos_scara.d,RobotSCARA->Con2DemData.axisPos_scara.a4};        
            if(RobotSCARA->forwardkinematics(scara_input,cartesian)){
                RobotSCARA->Con2DemData.cartPos=cartesian;
            }
            //输出脉冲
            // std::cout<<"第一个轴的脉冲:"<<1964610*RobotSCARA->RobotConfigData.pulseEquivalent[0]-RobotSCARA->RobotConfigData.offset2[0]<<std::endl;
            // std::cout<<"第二个轴的脉冲:"<<2782020*RobotSCARA->RobotConfigData.pulseEquivalent[1]-RobotSCARA->RobotConfigData.offset2[1]<<std::endl;
            // std::cout<<"第三个轴的脉冲"<<12582900*RobotSCARA->RobotConfigData.pulseEquivalent[2]-RobotSCARA->RobotConfigData.offset2[2]<<std::endl;
            // std::cout<<"第四个轴的脉冲"<<-63017200*RobotSCARA->RobotConfigData.pulseEquivalent[3]-RobotSCARA->RobotConfigData.offset2[3]<<std::endl;
            //输出角度
            // std::cout<<"关节角度：  "<<RobotSCARA->Con2DemData.axisPos_scara.a1<<" ";
            // std::cout<<RobotSCARA->Con2DemData.axisPos_scara.a2<<" ";
            // std::cout<<RobotSCARA->Con2DemData.axisPos_scara.d<<"  ";
            // std::cout<<RobotSCARA->Con2DemData.axisPos_scara.a4<<std::endl;
            std::cout<<"空间位姿"<<RobotSCARA->Con2DemData.cartPos.x<<" ";
            std::cout<<RobotSCARA->Con2DemData.cartPos.y<<" ";
            std::cout<<RobotSCARA->Con2DemData.cartPos.z<<" ";
            std::cout<<RobotSCARA->Con2DemData.cartPos.a<<" ";
            std::cout<<RobotSCARA->Con2DemData.cartPos.b<<" ";
            std::cout<<RobotSCARA->Con2DemData.cartPos.c<<std::endl;
            // std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
#else
    #ifdef USE_KERNEL
    //记录axispos和cartpos
    // RobotSCARA->MhRobotText.AxisPos_SCARA_out.open("AxisPos_scara.txt");
    // RobotSCARA->MhRobotText.CartPos_out.open("Cartpos.txt");
    int retn;//函数的返回值
    while (true)
    {
        if(RobotSCARA->Dem2ConData.emergeStop==1){
            return ;
        }
        if(RobotSCARA->Dem2ConData.enableState==1){ 
            if(RobotSCARA->ConChargeData.isEnable==1){
                int Position[RobotSCARA->get_nDof()];
                switch (RobotSCARA->ConChargeData.startServo)
                {
                case 0:
                    for(int i=0;i<RobotSCARA->get_nDof();++i){
                        retn=RobotSCARA->GetAxisPosition(i,&Position[i]);
                        RobotSCARA->set_retn(retn,Mh::GETAXISPOSITION);
                    }
                    break;
                case 1:
                    for(int i =0;i<RobotSCARA->get_nDof();++i){
                        retn=RobotSCARA->GetDriverPos(i,&Position[i]);
                        RobotSCARA->set_retn(retn,Mh::GETDRIVERPOS);
                    }
                    break;
                default:
                    break;
                }
                //计算轴关节坐标系的位置
                RobotSCARA->Con2DemData.axisPos_scara.a1=Position[0]*RobotSCARA->RobotConfigData.pulseEquivalent[0]-RobotSCARA->RobotConfigData.offset2[0];
                RobotSCARA->Con2DemData.axisPos_scara.a2=Position[1]*RobotSCARA->RobotConfigData.pulseEquivalent[1]-RobotSCARA->RobotConfigData.offset2[1];
                RobotSCARA->Con2DemData.axisPos_scara.a4=Position[3]*RobotSCARA->RobotConfigData.pulseEquivalent[3]-RobotSCARA->RobotConfigData.offset2[3];
                RobotSCARA->Con2DemData.axisPos_scara.d=Position[2]*RobotSCARA->RobotConfigData.pulseEquivalent[2]-RobotSCARA->RobotConfigData.offset2[2];//+RobotSCARA->Con2DemData.axisPos_scara.a4/360*RobotSCARA->get_a4_Compensation();
                //计算伺服电机的位置
                RobotSCARA->Con2DemData.drivePos.d1=RobotSCARA->Con2DemData.axisPos_scara.a1*RobotSCARA->RobotConfigData.direction[0]*RobotSCARA->RobotConfigData.ratio[0];
                RobotSCARA->Con2DemData.drivePos.d2=RobotSCARA->Con2DemData.axisPos_scara.a2*RobotSCARA->RobotConfigData.direction[1]*RobotSCARA->RobotConfigData.ratio[1];
                RobotSCARA->Con2DemData.drivePos.d3=RobotSCARA->Con2DemData.axisPos_scara.d*RobotSCARA->RobotConfigData.direction[2]*RobotSCARA->RobotConfigData.ratio[2];
                RobotSCARA->Con2DemData.drivePos.d4=RobotSCARA->Con2DemData.axisPos_scara.a4*RobotSCARA->RobotConfigData.direction[3]*RobotSCARA->RobotConfigData.ratio[3];
                //计算工具坐标系相对世界坐标系的位置
                std::vector<double> cartesian;
                std::vector<double> scara_input={RobotSCARA->Con2DemData.axisPos_scara.a1,RobotSCARA->Con2DemData.axisPos_scara.a2,RobotSCARA->Con2DemData.axisPos_scara.d,RobotSCARA->Con2DemData.axisPos_scara.a4};        
                if(RobotSCARA->forwardkinematics(scara_input,cartesian)){
                    RobotSCARA->Con2DemData.cartPos=cartesian;
                }
                //将数据打印到文本
                // RobotSCARA->MhRobotText.AxisPos_SCARA_out<<RobotSCARA->Con2DemData.axisPos_scara.a1<<"    "<<RobotSCARA->Con2DemData.axisPos_scara.a2<<"    "<<
                // RobotSCARA->Con2DemData.axisPos_scara.d<<"    "<<RobotSCARA->Con2DemData.axisPos_scara.a4<<std::endl;
                // RobotSCARA->MhRobotText.CartPos_out<<cartesian[0]<<"    "<<cartesian[1]<<"    "<<cartesian[2]<<"    "<<cartesian[3]<<"    "<<cartesian[4]<<"    "<<cartesian[5]<<std::endl;
                // std::cout<<"第1个轴的关节角度为: "<<RobotSCARA->Con2DemData.axisPos_scara.a1<<std::endl;
                // std::cout<<"第2个轴的关节角度为: "<<RobotSCARA->Con2DemData.axisPos_scara.a2<<std::endl;
                // std::cout<<"第3个轴的关节角度为: "<<RobotSCARA->Con2DemData.axisPos_scara.d<<std::endl;
                // std::cout<<"第4个轴的关节角度为: "<<RobotSCARA->Con2DemData.axisPos_scara.a4<<std::endl;
                // std::cout<<"笛卡尔空间坐标: "<<RobotSCARA->Con2DemData.cartPos.x<<"  "<<RobotSCARA->Con2DemData.cartPos.y<<"  "<<RobotSCARA->Con2DemData.cartPos.z<<"  "
                // <<RobotSCARA->Con2DemData.cartPos.a<<"  "<<RobotSCARA->Con2DemData.cartPos.b<<"  "<<RobotSCARA->Con2DemData.cartPos.c<<std::endl;
                // EC_TRACE("第1个轴的关节角度为：%f",RobotSCARA->Con2DemData.axisPos_scara.a1);
                // EC_TRACE("第2个轴的关节角度为：%f",RobotSCARA->Con2DemData.axisPos_scara.a2);
                // EC_TRACE("第3个轴的关节角度为：%f",RobotSCARA->Con2DemData.axisPos_scara.d);
                // EC_TRACE("第4个轴的关节角度为：%f",RobotSCARA->Con2DemData.axisPos_scara.a4);
            }   
        }
    }
    #else 
    //设置当前的轴关节位置和空间位置
        RobotSCARA->Con2DemData.axisPos_scara.a1=-39.9150;
        RobotSCARA->Con2DemData.axisPos_scara.a2=84.2464;
        RobotSCARA->Con2DemData.axisPos_scara.d=54.75512;
        RobotSCARA->Con2DemData.axisPos_scara.a4=161.4467;
        std::vector<double> cartesian;
        std::vector<double> scara_input={RobotSCARA->Con2DemData.axisPos_scara.a1,RobotSCARA->Con2DemData.axisPos_scara.a2,RobotSCARA->Con2DemData.axisPos_scara.d,RobotSCARA->Con2DemData.axisPos_scara.a4};        
        if(RobotSCARA->forwardkinematics(scara_input,cartesian)){
            RobotSCARA->Con2DemData.cartPos=cartesian;
        }
    #endif
#endif 
}