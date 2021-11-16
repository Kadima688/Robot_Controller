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
    ::grpc::Status SetStartServo(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_StartServo* request, ::ExternalDataTransfer::Pt_DataResult* response) override;
    ::grpc::Status SendVisualServoData_ServoType(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_SERVOTYPE* request, ::ExternalDataTransfer::Pt_DataResult* response)override;
    ::grpc::Status SendVisualServoData_TargetPos_XYZ(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_TARGETPOS_XYZ* request, ::ExternalDataTransfer::Pt_DataResult* response)override;
    ::grpc::Status SendVisualServoData_TargetPos_ABC(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_TARGETPOS_ABC* request, ::ExternalDataTransfer::Pt_DataResult* response)override;
    ::grpc::Status SendVisualServoData_Error_XYZ(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_ERROR_XYZ* request, ::ExternalDataTransfer::Pt_DataResult* response)override;
    ::grpc::Status SendVisualServoData_Error_ABC(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_ERROR_ABC* request, ::ExternalDataTransfer::Pt_DataResult* response)override;
    ::grpc::Status SendVisualServoData_Error_IMAGE(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_ERROR_IMAGE* request, ::ExternalDataTransfer::Pt_DataResult* response)override;
    ::grpc::Status SendVisualServoData_EndServo(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_ENDSERVO* request, ::ExternalDataTransfer::Pt_DataResult* response)override;
    ::grpc::Status GetAXISPOSSCARA(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_DataVoid* request, ::ExternalDataTransfer::Pt_AXISPOS_SCARA* response)override;
    ::grpc::Status SendSpeedPercent(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_SPEED_PERCENT* request, ::ExternalDataTransfer::Pt_DataResult* response)override;
    ::grpc::Status SendInching(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_INCHING* request, ::ExternalDataTransfer::Pt_DataResult* response)override;
};
}


#endif