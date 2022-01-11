#include"MhgRPCServer.h"
#include<iostream>
#include"MhIndustrialSCARA.h"
#include"MhMotionKernel.h"

::grpc::Status Mh::MhgRPCServer::SetEnableState(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_EnableState* request, ::ExternalDataTransfer::Pt_DataResult* response){
    SCARA->Dem2ConData.enableState=request->enablestate();
    return Status::OK;
}

::grpc::Status Mh::MhgRPCServer::SetStartServo(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_StartServo* request, ::ExternalDataTransfer::Pt_DataResult* response){
    SCARA->ConChargeData.startServo=request->startservo();
    return Status::OK;
}

::grpc::Status Mh::MhgRPCServer::SendVisualServoData_ServoType(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_SERVOTYPE* request, ::ExternalDataTransfer::Pt_DataResult* response){
    SCARA->Dem2ConData.visualServo.servotype=request->servotype();
    return Status::OK;
}

::grpc::Status Mh::MhgRPCServer::SendVisualServoData_TargetPos_XYZ(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_TARGETPOS_XYZ* request, ::ExternalDataTransfer::Pt_DataResult* response){
    SCARA->Dem2ConData.visualServo.desPos.x=request->target_x();
    SCARA->Dem2ConData.visualServo.desPos.y=request->target_y();
    SCARA->Dem2ConData.visualServo.desPos.z=request->target_z();
    std::cout<<"Target_x:"<<SCARA->Dem2ConData.visualServo.desPos.x<<std::endl;
    std::cout<<"Target_y:"<<SCARA->Dem2ConData.visualServo.desPos.y<<std::endl;
    std::cout<<"Target_z:"<<SCARA->Dem2ConData.visualServo.desPos.z<<std::endl;
    return Status::OK;
}

::grpc::Status Mh::MhgRPCServer::SendVisualServoData_TargetPos_ABC(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_TARGETPOS_ABC* request, ::ExternalDataTransfer::Pt_DataResult* response){
    SCARA->Dem2ConData.visualServo.desPos.a=request->target_a();
    SCARA->Dem2ConData.visualServo.desPos.b=request->target_b();
    SCARA->Dem2ConData.visualServo.desPos.c=request->target_c();
    std::cout<<"Target_a:"<<SCARA->Dem2ConData.visualServo.desPos.a<<std::endl;
    std::cout<<"Target_b:"<<SCARA->Dem2ConData.visualServo.desPos.b<<std::endl;
    std::cout<<"Target_c:"<<SCARA->Dem2ConData.visualServo.desPos.c<<std::endl;
    return Status::OK;
}

::grpc::Status Mh::MhgRPCServer::SendVisualServoData_Error_XYZ(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_ERROR_XYZ* request, ::ExternalDataTransfer::Pt_DataResult* response){
    SCARA->Dem2ConData.visualServo.error_xyz=request->error_xyz();
    std::cout<<"Error_xyz:"<<SCARA->Dem2ConData.visualServo.error_xyz<<std::endl;
    return Status::OK;
}

::grpc::Status Mh::MhgRPCServer::SendVisualServoData_Error_ABC(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_ERROR_ABC* request, ::ExternalDataTransfer::Pt_DataResult* response){
    SCARA->Dem2ConData.visualServo.error_abc=request->error_abc();
    std::cout<<"Error_abc:"<<SCARA->Dem2ConData.visualServo.error_abc<<std::endl;
    return Status::OK;
}

::grpc::Status Mh::MhgRPCServer::SendVisualServoData_Error_IMAGE(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_ERROR_IMAGE* request, ::ExternalDataTransfer::Pt_DataResult* response){
    SCARA->Dem2ConData.visualServo.error_image=request->error_image();
    std::cout<<"Error_image:"<<SCARA->Dem2ConData.visualServo.error_image<<std::endl;
    return Status::OK;
}

::grpc::Status Mh::MhgRPCServer::SendVisualServoData_EndServo(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_VISUAL_SERVO_ENDSERVO* request, ::ExternalDataTransfer::Pt_DataResult* response){
    SCARA->Dem2ConData.visualServo.ifEnd=request->endservo();
    return Status::OK;
}

::grpc::Status Mh::MhgRPCServer::GetAXISPOSSCARA(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_DataVoid* request, ::ExternalDataTransfer::Pt_AXISPOS_SCARA* response){
    response->set_a1(SCARA->Con2DemData.axisPos_scara.a1);
    response->set_a2(SCARA->Con2DemData.axisPos_scara.a2);
    response->set_d(SCARA->Con2DemData.axisPos_scara.d);
    response->set_a4(SCARA->Con2DemData.axisPos_scara.a4);
    return Status::OK;
}

::grpc::Status Mh::MhgRPCServer::SendSpeedPercent(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_SPEED_PERCENT* request, ::ExternalDataTransfer::Pt_DataResult* response){
    SCARA->Dem2ConData.ovr=request->percent_speed();
    return Status::OK;
}

::grpc::Status Mh::MhgRPCServer::SendInching(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_INCHING* request, ::ExternalDataTransfer::Pt_DataResult* response){
    SCARA->Dem2ConData.coordinate=request->axis_num();
    SCARA->Dem2ConData.upOrDown=request->axis_up_dwon();
    SCARA->Dem2ConData.PressOrRelease=request->axis_press_release();
    if(SCARA->Dem2ConData.operateMode==0)
    {
        // RobotSCARA->ConChargeData.curDynamic=RobotSCARA->RobotConfigData.jogspeed;
        // RobotSCARA->RobotDynInitial();
        // RobotSCARA->RobotInterpolationDynInitial();
        //开始示教
        if(SCARA->Dem2ConData.coordinate!=0 && SCARA->Dem2ConData.PressOrRelease==1){
            SCARA->Con2DemData.IsMoving = 1;
            //首先考虑轴关节坐标系的
            int dir;//运动方向
            if(SCARA->Dem2ConData.upOrDown==0){
                dir=-1;
            }
            else{
                dir=1;
            }
            double targetVel = 1000*SCARA->Dem2ConData.ovr/5;
            double lowVel = 0;
            double acc = 100*SCARA->Dem2ConData.ovr/5;
            double jerk = 10*SCARA->Dem2ConData.ovr/5;
            int retn=SCARA->SetJogParam(SCARA->Dem2ConData.coordinate-1,targetVel,lowVel,acc,jerk);
            SCARA->set_retn(retn,Mh::IPMCJOGSETAXISPARAM);
            retn=SCARA->Jog(SCARA->Dem2ConData.coordinate-1,dir*SCARA->RobotConfigData.direction[SCARA->Dem2ConData.coordinate-1]);
            SCARA->set_retn(retn,Mh::IPMCJOG);
            
        }
        else if(SCARA->Dem2ConData.PressOrRelease==0){
            //现在coordinate=0，表示按键松开了
            int retn=SCARA->StopAxis(SCARA->Dem2ConData.coordinate-1,1);
            SCARA->set_retn(retn,Mh::STOPAXIS);
        }
    }
    SCARA->Con2DemData.IsMoving = 0 ;
    return Status::OK;
}

::grpc::Status Mh::MhgRPCServer::SendSCARAPTP(::grpc::ServerContext* context, const ::ExternalDataTransfer::Pt_AXISPOS_SCARA* request, ::ExternalDataTransfer::Pt_DataResult* response){
    std::cout<<request->a1()<<std::endl;
    std::cout<<request->a2()<<std::endl;
    std::cout<<request->d()<<std::endl;
    std::cout<<request->a4()<<std::endl;
    return Status::OK;
}