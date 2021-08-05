
#include<iostream>
#include<unistd.h>
#include"MhIndustrialSCARA.h"
#include"os.h"
void GetRobotState(void *scara){
    Mh::MhIndustrialSCARA* RobotSCARA=static_cast<Mh::MhIndustrialSCARA*>(scara);
    while (true)
    {
        if(RobotSCARA->emergeStop==1){
            return ;
        }
        if(RobotSCARA->enableState==1){
            //判断电机是否已经处于使能状态
            unsigned int value[RobotSCARA->get_nDof()];
            int n=0;
            for(int i=0;i<RobotSCARA->get_nDof();++i){
                RobotSCARA->GetDriverState(i,&value[i]);
                if(value[i]==8){
                    n++;
                }
            }
            if(n==RobotSCARA->get_nDof()){
                //处于使能状态
                RobotSCARA->isEnable=1;
                int Position[RobotSCARA->get_nDof()];
                for(int i=0;i<RobotSCARA->get_nDof();++i){
                    switch (RobotSCARA->startServo)
                    {
                    case 0:
                        RobotSCARA->retn=RobotSCARA->GetAxisPosition(i,&Position[i]);
                        if(RobotSCARA->retn!=0){
                            std::cout<<"读取"<<i<<"轴的逻辑位置失败"<<std::endl;
                            EC_TRACE("hello");
                        }
                        break;
                    case 1:
                        RobotSCARA->retn=RobotSCARA->GetDriverPos(i,&Position[i]);
                        if(RobotSCARA->retn!=0){
                            std::cout<<"读取"<<i<<"从站的逻辑位置失败"<<std::endl;
                        }
                        break;
                    default:
                        break;
                    }
                }
                //计算轴关节坐标系的位置
                RobotSCARA->axisPos_scara.a1=Position[0]*RobotSCARA->get_pulseEquivalent()[0]-RobotSCARA->get_offset2()[0];
                RobotSCARA->axisPos_scara.a2=Position[1]*RobotSCARA->get_pulseEquivalent()[1]-RobotSCARA->get_offset2()[1];
                RobotSCARA->axisPos_scara.a4=Position[3]*RobotSCARA->get_pulseEquivalent()[3]-RobotSCARA->get_offset2()[3];
                RobotSCARA->axisPos_scara.d=Position[2]*RobotSCARA->get_pulseEquivalent()[2]-RobotSCARA->get_offset2()[2]+RobotSCARA->axisPos_scara.a4/360*RobotSCARA->get_a4_Compensation();
                //计算伺服电机的位置
                RobotSCARA->drivePos.d1=RobotSCARA->axisPos_scara.a1*RobotSCARA->get_direction()[0]*RobotSCARA->get_ratio()[0];
                RobotSCARA->drivePos.d2=RobotSCARA->axisPos_scara.a2*RobotSCARA->get_direction()[1]*RobotSCARA->get_ratio()[1];
                RobotSCARA->drivePos.d3=RobotSCARA->axisPos_scara.d*RobotSCARA->get_direction()[2]*RobotSCARA->get_ratio()[2];
                RobotSCARA->drivePos.d4=RobotSCARA->axisPos_scara.a4*RobotSCARA->get_direction()[3]*RobotSCARA->get_ratio()[3];
                //计算工具坐标系相对世界坐标系的位置
                std::vector<double> cartesian;
                std::vector<double> scara_input={RobotSCARA->axisPos_scara.a1,RobotSCARA->axisPos_scara.a2,RobotSCARA->axisPos_scara.d,RobotSCARA->axisPos_scara.a4};        
                if(RobotSCARA->forwardkinematics(scara_input,cartesian)){
                    RobotSCARA->cartPos=cartesian;
                }
                EC_TRACE("第1个轴的关节角度为：%f",RobotSCARA->axisPos_scara.a1);
                EC_TRACE("第2个轴的关节角度为：%f",RobotSCARA->axisPos_scara.a2);
                EC_TRACE("第3个轴的关节角度为：%f",RobotSCARA->axisPos_scara.d);
                EC_TRACE("第4个轴的关节角度为：%f",RobotSCARA->axisPos_scara.a4);
            }   
        }
    }
    
}