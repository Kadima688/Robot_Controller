#ifndef MHGRPCSERVER_H
#define MHGRPCSERVER_H

#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>
#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include"DemConDataTransfer.grpc.pb.h"
#include"DemConDataTransfer.pb.h"
#include"MhIndustrialSCARA.h"
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

using namespace ExternalDataTransfer;

namespace Mh{
class MhgRPCServer final : public RPCDemConData::Service{ 
public:
    MhIndustrialSCARA* SCARA;
    MhgRPCServer(MhIndustrialSCARA* scara){
        SCARA=scara;
    }
    ::grpc::Status SetEnableState(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_EnableState* request, ::ExternalDataTransfer::Pt_DataResult* response) override;
};
}


#endif