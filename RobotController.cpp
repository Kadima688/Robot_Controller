#include<iostream>
#include"MhIndustrialSCARA.h"
#include"ThreadManage.h"
void DataTransfer(Mh::MhIndustrialSCARA* RobotSCARA);
void Controlthread(Mh::MhIndustrialSCARA* RobotSCARA);
void GetRobotState(Mh::MhIndustrialSCARA* RobotSCARA);
#include<eigen3/Eigen/Core>
#include<string>
#include<thread>
#include"MhgRPCServer.h"

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
    Mh::MhIndustrialSCARA RobotSCARA;
    if(!RobotSCARA.loadRobotConfigFile("RobotConfig_CoolDrive.xml")){
        return 0;
    }  
    RobotSCARA.set_dh_table();
    std::thread DataTransferThread(Serverrun,&RobotSCARA);
    std::thread ControlThread(Controlthread,&RobotSCARA);
    std::thread RobotStateThread(GetRobotState,&RobotSCARA);
    DataTransferThread.join();
    ControlThread.join();
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
    // double input[4]={-39.9150,84.2464,54.75512,161.4467};
    // std::vector<double> scara_input(input,input+4);  
    // std::vector<double> Cartesian;
    // if(scara.forwardkinematics(scara_input,Cartesian)){
    //     std::cout<<"正解成功"<<std::endl;
    //     std::cout<<Cartesian[0]<<"    "<<Cartesian[1]<<"    "<<Cartesian[2]<<"    "<<Cartesian[3]<<"    "<<Cartesian[4]<<"    "<<Cartesian[5]<<std::endl;
    //     scara.Con2DemData.cartPos=Cartesian;
    //     std::cout<<scara.Con2DemData.cartPos.x<<"    "<<scara.Con2DemData.cartPos.y<<"    "<<scara.Con2DemData.cartPos.z<<"    "<<scara.Con2DemData.cartPos.a<<"    "<<scara.Con2DemData.cartPos.b<<"   "<<scara.Con2DemData.cartPos.c<<std::endl;
    // }
    // //-------------inversekinematics
    // double Axis[4]={-30,80,40,70};
    // scara_input.clear();
    // for(int i=0;i<4;++i){
    //     scara_input.push_back(Axis[i]);
    // }
    // std::cout<<scara.transform.ZYZ2homomatrix(Cartesian)<<std::endl;
    // Eigen::MatrixXd fTe=scara.transform.ZYZ2homomatrix(Cartesian);
    // std::cout<<fTe<<std::endl;
    // if(scara.inversekinematics(scara_input,Cartesian,0)){
    //     std::cout<<"逆解成功"<<std::endl;
    //     std::cout<<scara_input[0]<<"    "<<scara_input[1]<<"    "<<scara_input[2]<<"    "<<scara_input[3]<<std::endl;
    //     scara.Con2DemData.axisPos_scara=scara_input;
    //     std::cout<<scara.Con2DemData.axisPos_scara.a1<<"    "<<scara.Con2DemData.axisPos_scara.a2<<"    "<<scara.Con2DemData.axisPos_scara.d<<"    "<<scara.Con2DemData.axisPos_scara.a4<<std::endl;
    // } 
    // return 0;
}