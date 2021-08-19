// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: DemConDataTransfer.proto

#include "DemConDataTransfer.pb.h"
#include "DemConDataTransfer.grpc.pb.h"

#include <functional>
#include <grpcpp/impl/codegen/async_stream.h>
#include <grpcpp/impl/codegen/async_unary_call.h>
#include <grpcpp/impl/codegen/channel_interface.h>
#include <grpcpp/impl/codegen/client_unary_call.h>
#include <grpcpp/impl/codegen/client_callback.h>
#include <grpcpp/impl/codegen/message_allocator.h>
#include <grpcpp/impl/codegen/method_handler.h>
#include <grpcpp/impl/codegen/rpc_service_method.h>
#include <grpcpp/impl/codegen/server_callback.h>
#include <grpcpp/impl/codegen/server_callback_handlers.h>
#include <grpcpp/impl/codegen/server_context.h>
#include <grpcpp/impl/codegen/service_type.h>
#include <grpcpp/impl/codegen/sync_stream.h>
namespace ExternalDataTransfer {

static const char* RPCDemConData_method_names[] = {
  "/ExternalDataTransfer.RPCDemConData/SetEnableState",
  "/ExternalDataTransfer.RPCDemConData/GetAXISPOSSCARA",
};

std::unique_ptr< RPCDemConData::Stub> RPCDemConData::NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options) {
  (void)options;
  std::unique_ptr< RPCDemConData::Stub> stub(new RPCDemConData::Stub(channel));
  return stub;
}

RPCDemConData::Stub::Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel)
  : channel_(channel), rpcmethod_SetEnableState_(RPCDemConData_method_names[0], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  , rpcmethod_GetAXISPOSSCARA_(RPCDemConData_method_names[1], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  {}

::grpc::Status RPCDemConData::Stub::SetEnableState(::grpc::ClientContext* context, const ::ExternalDataTransfer::Pt_EnableState& request, ::ExternalDataTransfer::Pt_DataResult* response) {
  return ::grpc::internal::BlockingUnaryCall< ::ExternalDataTransfer::Pt_EnableState, ::ExternalDataTransfer::Pt_DataResult, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_SetEnableState_, context, request, response);
}

void RPCDemConData::Stub::experimental_async::SetEnableState(::grpc::ClientContext* context, const ::ExternalDataTransfer::Pt_EnableState* request, ::ExternalDataTransfer::Pt_DataResult* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::ExternalDataTransfer::Pt_EnableState, ::ExternalDataTransfer::Pt_DataResult, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetEnableState_, context, request, response, std::move(f));
}

void RPCDemConData::Stub::experimental_async::SetEnableState(::grpc::ClientContext* context, const ::ExternalDataTransfer::Pt_EnableState* request, ::ExternalDataTransfer::Pt_DataResult* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_SetEnableState_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::ExternalDataTransfer::Pt_DataResult>* RPCDemConData::Stub::PrepareAsyncSetEnableStateRaw(::grpc::ClientContext* context, const ::ExternalDataTransfer::Pt_EnableState& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::ExternalDataTransfer::Pt_DataResult, ::ExternalDataTransfer::Pt_EnableState, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_SetEnableState_, context, request);
}

::grpc::ClientAsyncResponseReader< ::ExternalDataTransfer::Pt_DataResult>* RPCDemConData::Stub::AsyncSetEnableStateRaw(::grpc::ClientContext* context, const ::ExternalDataTransfer::Pt_EnableState& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncSetEnableStateRaw(context, request, cq);
  result->StartCall();
  return result;
}

::grpc::Status RPCDemConData::Stub::GetAXISPOSSCARA(::grpc::ClientContext* context, const ::ExternalDataTransfer::Pt_DataVoid& request, ::ExternalDataTransfer::Pt_AXISPOS_SCARA* response) {
  return ::grpc::internal::BlockingUnaryCall< ::ExternalDataTransfer::Pt_DataVoid, ::ExternalDataTransfer::Pt_AXISPOS_SCARA, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), rpcmethod_GetAXISPOSSCARA_, context, request, response);
}

void RPCDemConData::Stub::experimental_async::GetAXISPOSSCARA(::grpc::ClientContext* context, const ::ExternalDataTransfer::Pt_DataVoid* request, ::ExternalDataTransfer::Pt_AXISPOS_SCARA* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall< ::ExternalDataTransfer::Pt_DataVoid, ::ExternalDataTransfer::Pt_AXISPOS_SCARA, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetAXISPOSSCARA_, context, request, response, std::move(f));
}

void RPCDemConData::Stub::experimental_async::GetAXISPOSSCARA(::grpc::ClientContext* context, const ::ExternalDataTransfer::Pt_DataVoid* request, ::ExternalDataTransfer::Pt_AXISPOS_SCARA* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create< ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(stub_->channel_.get(), stub_->rpcmethod_GetAXISPOSSCARA_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::ExternalDataTransfer::Pt_AXISPOS_SCARA>* RPCDemConData::Stub::PrepareAsyncGetAXISPOSSCARARaw(::grpc::ClientContext* context, const ::ExternalDataTransfer::Pt_DataVoid& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderHelper::Create< ::ExternalDataTransfer::Pt_AXISPOS_SCARA, ::ExternalDataTransfer::Pt_DataVoid, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(channel_.get(), cq, rpcmethod_GetAXISPOSSCARA_, context, request);
}

::grpc::ClientAsyncResponseReader< ::ExternalDataTransfer::Pt_AXISPOS_SCARA>* RPCDemConData::Stub::AsyncGetAXISPOSSCARARaw(::grpc::ClientContext* context, const ::ExternalDataTransfer::Pt_DataVoid& request, ::grpc::CompletionQueue* cq) {
  auto* result =
    this->PrepareAsyncGetAXISPOSSCARARaw(context, request, cq);
  result->StartCall();
  return result;
}

RPCDemConData::Service::Service() {
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      RPCDemConData_method_names[0],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< RPCDemConData::Service, ::ExternalDataTransfer::Pt_EnableState, ::ExternalDataTransfer::Pt_DataResult, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](RPCDemConData::Service* service,
             ::grpc::ServerContext* ctx,
             const ::ExternalDataTransfer::Pt_EnableState* req,
             ::ExternalDataTransfer::Pt_DataResult* resp) {
               return service->SetEnableState(ctx, req, resp);
             }, this)));
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      RPCDemConData_method_names[1],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< RPCDemConData::Service, ::ExternalDataTransfer::Pt_DataVoid, ::ExternalDataTransfer::Pt_AXISPOS_SCARA, ::grpc::protobuf::MessageLite, ::grpc::protobuf::MessageLite>(
          [](RPCDemConData::Service* service,
             ::grpc::ServerContext* ctx,
             const ::ExternalDataTransfer::Pt_DataVoid* req,
             ::ExternalDataTransfer::Pt_AXISPOS_SCARA* resp) {
               return service->GetAXISPOSSCARA(ctx, req, resp);
             }, this)));
}

RPCDemConData::Service::~Service() {
}

::grpc::Status RPCDemConData::Service::SetEnableState(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_EnableState* request, ::ExternalDataTransfer::Pt_DataResult* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}

::grpc::Status RPCDemConData::Service::GetAXISPOSSCARA(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_DataVoid* request, ::ExternalDataTransfer::Pt_AXISPOS_SCARA* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}


}  // namespace ExternalDataTransfer

