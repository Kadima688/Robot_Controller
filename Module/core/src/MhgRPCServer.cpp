#include"MhgRPCServer.h"
#include<iostream>
#include"MhIndustrialSCARA.h"


::grpc::Status Mh::MhgRPCServer::SetEnableState(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_EnableState* request, ::ExternalDataTransfer::Pt_DataResult* response){
    SCARA->Dem2ConData.enableState=request->enablestate();
    std::cout<<SCARA->Dem2ConData.enableState<<std::endl;
    return Status::OK;
}

