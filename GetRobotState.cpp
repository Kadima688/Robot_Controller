
#include<iostream>
#include<unistd.h>
#include"MhIndustrialSCARA.h"
#include"os.h"
void GetRobotState(void *scara){
    Mh::MhIndustrialSCARA* RobotSCARA=static_cast<Mh::MhIndustrialSCARA*>(scara);
    while (true)
    {
        if(RobotSCARA->Dem2ConData.emergeStop==1){
            return ;
        }
        if(RobotSCARA->Dem2ConData.enableState==1){
            //判断电机是否已经处于使能状态
            unsigned int value[RobotSCARA->get_nDof()];
            int n=0;
            for(int i=0;i<RobotSCARA->get_nDof();++i){
                if(RobotSCARA->GetDriverState(i,&value[i])!=0){
                    EC_TRACE("读取从站逻辑位置失败");
                }
                if(value[i]==8){
                    n++;
                }
            }
            if(n==RobotSCARA->get_nDof()){
                //处于使能状态
                RobotSCARA->ConChargeData.isEnable=1;
                int Position[RobotSCARA->get_nDof()];
                for(int i=0;i<RobotSCARA->get_nDof();++i){
                    switch (RobotSCARA->ConChargeData.startServo)
                    {
                    case 0:
                        RobotSCARA->ConChargeData.retn=RobotSCARA->GetAxisPosition(i,&Position[i]);
                        if(RobotSCARA->ConChargeData.retn!=0){
                            std::cout<<"读取"<<i<<"轴的逻辑位置失败"<<std::endl;
                            EC_TRACE("hello");
                        }
                        break;
                    case 1:
                        RobotSCARA->ConChargeData.retn=RobotSCARA->GetDriverPos(i,&Position[i]);
                        if(RobotSCARA->ConChargeData.retn!=0){
                            std::cout<<"读取"<<i<<"从站的逻辑位置失败"<<std::endl;
                        }
                        break;
                    default:
                        break;
                    }
                }
                //计算轴关节坐标系的位置
                RobotSCARA->Con2DemData.axisPos_scara.a1=Position[0]*RobotSCARA->RobotConfigData.pulseEquivalent[0]-RobotSCARA->RobotConfigData.offset2[0];
                RobotSCARA->Con2DemData.axisPos_scara.a2=Position[1]*RobotSCARA->RobotConfigData.pulseEquivalent[1]-RobotSCARA->RobotConfigData.offset2[1];
                RobotSCARA->Con2DemData.axisPos_scara.a4=Position[3]*RobotSCARA->RobotConfigData.pulseEquivalent[3]-RobotSCARA->RobotConfigData.offset2[3];
                RobotSCARA->Con2DemData.axisPos_scara.d=Position[2]*RobotSCARA->RobotConfigData.pulseEquivalent[2]-RobotSCARA->RobotConfigData.offset2[2]+RobotSCARA->Con2DemData.axisPos_scara.a4/360*RobotSCARA->get_a4_Compensation();
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
                EC_TRACE("第1个轴的关节角度为：%f",RobotSCARA->Con2DemData.axisPos_scara.a1);
                EC_TRACE("第2个轴的关节角度为：%f",RobotSCARA->Con2DemData.axisPos_scara.a2);
                EC_TRACE("第3个轴的关节角度为：%f",RobotSCARA->Con2DemData.axisPos_scara.d);
                EC_TRACE("第4个轴的关节角度为：%f",RobotSCARA->Con2DemData.axisPos_scara.a4);
            }   
        }
    }
    
}