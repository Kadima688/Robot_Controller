#include<iostream>
#include"MhIndustrialSCARA.h"
#include"ThreadManage.h"
void GetRobotState(void *scara);
void VisualServoSCARA_PBVS(void *scara);
#include<eigen3/Eigen/Core>

int main(){
    //---------------thread
    Mh::MhIndustrialSCARA RobotSCARA;
    if(!RobotSCARA.loadRobotConfigFile("RobotConfig.xml")){
        return 0;
    }  
    RobotSCARA.set_dh_table();
    // RobotSCARA.OpenDevice();
    // m_Thread RobotStateThread(GetRobotState,0,&RobotSCARA,"GetRobotStateThread");
    // m_Thread VisualServoThread(VisualServoSCARA_PBVS,0,&RobotSCARA,"VisualServoThread");
    // RobotStateThread.Start();
    // VisualServoThread.Start();    
    return 0;

    //添加linux平台相关的代码
    //----------------init DH-TABLE
    // double alapha_[4]={0,0,180,0};
    // double a_[4]={0,100,100,0};
    // double d_[4]={100,0,0,0};
    // double offset_[4]={0,0,0,0};
    // std::vector<double> alapha(alapha_,alapha_+4);
    // std::vector<double> a(a_,a_+4);
    // std::vector<double> d(d_,d_+4);
    // std::vector<double> offset(offset_,offset_+4);
    // Mh::MhDH scara_dh_table;
    // scara_dh_table.set_link_number(4);
    // scara_dh_table.set_alapha(alapha);
    // scara_dh_table.set_a(a);
    // scara_dh_table.set_d(d);
    // scara_dh_table.set_theta(offset);
    // //---------------init ROBOT
    // Mh::MhIndustrialSCARA scara;
    // scara.set_dh_table(scara_dh_table);
    // scara.get_dh_table();
    // //--------------forwardkinematics
    // double input[4]={-22.69,88.37,20,33.33};
    // std::vector<double> scara_input(input,input+4);  
    // std::vector<double> Cartesian;
    // if(scara.forwardkinematics(scara_input,Cartesian)){
    //     std::cout<<"正解成功"<<std::endl;
    //     std::cout<<Cartesian[0]<<"    "<<Cartesian[1]<<"    "<<Cartesian[2]<<"    "<<Cartesian[3]<<"    "<<Cartesian[4]<<"    "<<Cartesian[5]<<std::endl;
    //     scara.cartPos=Cartesian;
    //     std::cout<<scara.cartPos.x<<"    "<<scara.cartPos.y<<"    "<<scara.cartPos.z<<"    "<<scara.cartPos.a<<"    "<<scara.cartPos.b<<"   "<<scara.cartPos.c<<std::endl;
    // }
    // //-------------inversekinematics
    // double Axis[4]={0,0,0,0};
    // scara_input.clear();
    // for(int i=0;i<4;++i){
    //     scara_input.push_back(Axis[i]);
    // }
    // if(scara.inversekinematics(scara_input,Cartesian,0)){
    //     std::cout<<"逆解成功"<<std::endl;
    //     std::cout<<scara_input[0]<<"    "<<scara_input[1]<<"    "<<scara_input[2]<<"    "<<scara_input[3]<<std::endl;
    //     scara.axisPos_scara=scara_input;
    //     std::cout<<scara.axisPos_scara.a1<<"    "<<scara.axisPos_scara.a2<<"    "<<scara.axisPos_scara.d<<"    "<<scara.axisPos_scara.a4<<std::endl;
    // } 
    // return 0;
}