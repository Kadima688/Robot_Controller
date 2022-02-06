#include<iostream>
#include"MhIndustrialSCARA.h"
#include"MhIndustrialRobotKAWASAKI.h"
#include"ThreadManage.h"
#include<eigen3/Eigen/Core>
#include<string>
#include<thread>
#include"MhgRPCServer.h"
#include<ctime>
#include<unistd.h>
#include"ControllerData.h"

void DataTransfer(Mh::MhIndustrialSCARA* RobotSCARA);
void Controlthread(Mh::MhIndustrialSCARA* RobotSCARA);
void GetRobotState(ControllerData* controllerdata);

void Serverrun(Mh::MhIndustrialSCARA* scara){
    std::string server_address("0.0.0.0:50051");
    Mh::MhgRPCServer service(scara);
    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    ServerBuilder builder;
    builder.AddListeningPort(server_address,grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout<<"Server listening on "<<server_address<<std::endl;
    server->Wait();
}

int main(int argc, char **argv){
    ControllerData controllerdata;
    // PLCOpenMotion motor;
    if(!controllerdata.robotscara.loadRobotConfigFile("RobotConfig_CoolDrive.xml")){  
        return 0;
    } 
    controllerdata.robotscara.set_dh_table();
    std::thread DataTransferThread(Serverrun,&controllerdata.robotscara);
    // std::thread ControlThread(Controlthread,&RobotSCARA);
    std::thread RobotStateThread(GetRobotState,&controllerdata);
    DataTransferThread.join();
    // ControlThread.join();
    RobotStateThread.join();
    return 0;
   
    //添加linux平台相关的代码
    //----------------init DH-TABLE
    // double alapha_[4]={0,0,180,0};
    // double a_[4]={0,300,300,0};
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
    // // double input[4]={-39.9150,84.2464,54.75512,161.4467};
    // double input[4]={-44.24,55.5,0,66.6};
    // std::vector<double> scara_input(input,input+4);  
    // std::vector<double> Cartesian;
    // if(scara.forwardkinematics(scara_input,Cartesian)){
    //     std::cout<<"正解成功"<<std::endl;
    //     std::cout<<Cartesian[0]<<"    "<<Cartesian[1]<<"    "<<Cartesian[2]<<"    "<<Cartesian[3]<<"    "<<Cartesian[4]<<"    "<<Cartesian[5]<<std::endl;
    // }
    // //-------------inversekinematics
    // clock_t start=clock();
    // double Axis[4]={0,0,0,0};
    // scara_input.clear();
    // for(int i=0;i<4;++i){
    //     scara_input.push_back(Axis[i]);
    // }
    // std::cout<<scara.transform.ZYZ2homomatrix(Cartesian)<<std::endl;
    // Eigen::MatrixXd fTe=scara.transform.ZYZ2homomatrix(Cartesian);
    // // std::cout<<fTe<<std::endl;
    // if(scara.inversekinematics(scara_input,Cartesian,0)){
    //     std::cout<<"逆解成功"<<std::endl;
    //     std::cout<<scara_input[0]<<"    "<<scara_input[1]<<"    "<<scara_input[2]<<"    "<<scara_input[3]<<std::endl;
    // } 
    // clock_t end=clock();
    // std::cout<<"总花费时间："<<(double)(end-start)/CLOCKS_PER_SEC*1000<<"毫秒"<<std::endl;
    // return 0;
}