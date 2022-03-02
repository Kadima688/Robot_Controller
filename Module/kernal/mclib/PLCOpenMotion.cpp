#include "PLCOpenMotion.h"
#include<memory.h>
#include <algorithm>    // std::copy
PLCOpenMotion::PLCOpenMotion(/* args */)
{
}

PLCOpenMotion::~PLCOpenMotion()
{
}

int PLCOpenMotion::MC_Power(INT Axis, BOOL Enable, BOOL EnablePositive, BOOL EnableNegative)
{
    unsigned char SendBuffer[sizeof(Command) + sizeof(STRUCT_MC_POWER)];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_POWER;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_POWER *data = (STRUCT_MC_POWER *)cmd->data;

    data->Axis = Axis;
    data->Enable = Enable;
    data->EnablePositive = EnablePositive;
    data->EnableNegative = EnableNegative;
    int ret = McWrapper.Send(SendBuffer, sizeof(Command) + sizeof(STRUCT_MC_POWER));
    if (ret == sizeof(Command) + sizeof(STRUCT_MC_POWER))
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
int PLCOpenMotion::MC_Stop(INT Axis, BOOL Execute, double Deceleration, double Jerk)
{
    unsigned char SendBuffer[sizeof(Command) + sizeof(STRUCT_MC_STOP)];
    Command *cmd = (Command *)SendBuffer;

    STRUCT_MC_STOP *data = (STRUCT_MC_STOP *)cmd->data;
    cmd->msgID = CMD_PLCOPENPART1_MC_STOP;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();
    data->Axis = Axis;
    data->Execute = Execute;
    data->Deceleration = Deceleration;
    data->Jerk = Jerk;
    int ret = McWrapper.Send(SendBuffer, sizeof(Command) + sizeof(STRUCT_MC_STOP));
    if (ret == sizeof(Command) + sizeof(STRUCT_MC_STOP))
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
int PLCOpenMotion::MC_MoveAbsolute(INT Axis, BOOL Execute, BOOL ContinuousUpdate, double Position, double Velocity, double Acceleration, double Decerelation, double Jerk, MC_DIRECTION Direction, MC_BUFFER_MODE BufferMode)
{
    unsigned char SendBuffer[sizeof(Command) + sizeof(STRUCT_MC_MOVEABSOLUTE)];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_MoveAbsolute;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_MOVEABSOLUTE *data = (STRUCT_MC_MOVEABSOLUTE *)cmd->data;

    data->Axis = Axis;
    data->Execute = Execute;
    data->ContinuousUpdate = ContinuousUpdate;
    data->Position = Position;
    data->Velocity = Velocity;
    data->Acceleration = Acceleration;
    data->Deceleration = Decerelation;
    data->Jerk = Jerk;
    data->Direction = Direction;
    data->BufferMode = BufferMode;
    int ret = McWrapper.Send(SendBuffer, sizeof(Command) + sizeof(STRUCT_MC_MOVEABSOLUTE));
    if (ret == sizeof(Command) + sizeof(STRUCT_MC_MOVEABSOLUTE))
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
int PLCOpenMotion::MC_SetPostion(INT Axis, BOOL Execute, double Position, BOOL Relative, MC_EXECUTION_MODE ExecutionMode)
{
    unsigned char SendBuffer[sizeof(Command) + sizeof(STRUCT_MC_SETPOSITION)];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_SetPostion;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_SETPOSITION *data = (STRUCT_MC_SETPOSITION *)cmd->data;

    data->Axis = Axis;
    data->Execute = Execute;
    data->Execute = Execute;
    data->Position = Position;
    data->ExecutionMode = ExecutionMode;
    int ret = McWrapper.Send(SendBuffer, sizeof(Command) + sizeof(STRUCT_MC_SETPOSITION));
    if (ret == sizeof(Command) + sizeof(STRUCT_MC_SETPOSITION))
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
int PLCOpenMotion::MC_MoveRelative(INT Axis, BOOL Execute, bool ContinuousUpdate, double Distance, double Velocity, double Acceleration, double Decerelation, double Jerk, MC_BUFFER_MODE BufferMode)
{
    unsigned char SendBuffer[sizeof(Command) + sizeof(STRUCT_MC_MOVEABSOLUTE)];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_MoveRelative;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_MOVEABSOLUTE *data = (STRUCT_MC_MOVEABSOLUTE *)cmd->data;

    data->Axis = Axis;
    data->Execute = Execute;
    data->ContinuousUpdate = ContinuousUpdate;
    data->Position = Distance;
    data->Velocity = Velocity;
    data->Acceleration = Acceleration;
    data->Deceleration = Decerelation;
    data->Jerk = Jerk;
    // data->Direction=Di;
    data->BufferMode = BufferMode;
    int ret = McWrapper.Send(SendBuffer, sizeof(Command) + sizeof(STRUCT_MC_MOVEABSOLUTE));
    if (ret == sizeof(Command) + sizeof(STRUCT_MC_MOVEABSOLUTE))
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
int PLCOpenMotion::MC_MoveContinuousAbsolute(INT Axis, BOOL Execute, bool ContinuousUpdate, double Position, double EndVelocity, double Velocity, double Acceleration, double Decerelation, double Jerk, MC_BUFFER_MODE BufferMode)
{
    unsigned char SendBuffer[sizeof(Command) + sizeof(STRUCT_MC_MOVECONTINUOUS)];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_MoveContinuousAbsolute;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_MOVECONTINUOUS *data = (STRUCT_MC_MOVECONTINUOUS *)cmd->data;

    data->Axis = Axis;
    data->Execute = Execute;
    data->ContinuousUpdate = ContinuousUpdate;
    data->Position = Position;
    data->Velocity = Velocity;
    data->EndVelocity = EndVelocity;
    data->Acceleration = Acceleration;
    data->Decerelation = Decerelation;
    data->jerk = Jerk;
    data->BufferMode = BufferMode;
    int ret = McWrapper.Send(SendBuffer, sizeof(Command) + sizeof(STRUCT_MC_MOVECONTINUOUS));
    if (ret == sizeof(Command) + sizeof(STRUCT_MC_MOVECONTINUOUS))
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
int PLCOpenMotion::MC_MoveContinuousRelative(INT Axis, BOOL Execute, bool ContinuousUpdate, double Distance, double EndVelocity, double Velocity, double Acceleration, double Decerelation, double Jerk, MC_BUFFER_MODE BufferMode)
{
    unsigned char SendBuffer[sizeof(Command) + sizeof(STRUCT_MC_MOVECONTINUOUS)];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_MoveContinuousRelative;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_MOVECONTINUOUS *data = (STRUCT_MC_MOVECONTINUOUS *)cmd->data;

    data->Axis = Axis;
    data->Execute = Execute;
    data->ContinuousUpdate = ContinuousUpdate;
    data->Position = Distance;
    data->Velocity = Velocity;
    data->EndVelocity = EndVelocity;
    data->Acceleration = Acceleration;
    data->Decerelation = Decerelation;
    data->jerk = Jerk;
    data->BufferMode = BufferMode;
    int ret = McWrapper.Send(SendBuffer, sizeof(Command) + sizeof(STRUCT_MC_MOVECONTINUOUS));
    if (ret == sizeof(Command) + sizeof(STRUCT_MC_MOVECONTINUOUS))
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
// int PLCOpenMotion::MC_ReadActualPosition(int Axis,bool Enable,double *Position)
// {
//     *Position=McWrapper.GetPosition(Axis);
//     return 0;

// }
// int PLCOpenMotion::MC_ReadActualVelocity(int Axis,bool Enable,double *Velocity)
// {
//     *Velocity=McWrapper.GetVelocity(Axis);
//     return 0;

// }
// int PLCOpenMotion::MC_ReadActualTorque(int Axis,bool Enable,double *Torque)
// {
//     *Torque=McWrapper.GetTorque(Axis);
//     return 0;

// }
// FBStatus PLCOpenMotion::GetIdStatus(int id)
// {
//     return McWrapper.GetFBStatus(id);

// }
double PLCOpenMotion::MC_ReadActualPosition(int Axis, bool Enable)
{
    return McWrapper.GetPosition(Axis);
}
int32_t PLCOpenMotion::MC_ReadActualVelocity(int Axis, bool Enable)
{
    return McWrapper.GetVelocity(Axis);
}
double PLCOpenMotion::MC_ReadActualTorque(int Axis, bool Enable)
{
    return McWrapper.GetTorque(Axis);
}
FBStatus PLCOpenMotion::GetIdStatus(int id)
{
    return McWrapper.GetFBStatus(id);
}









//group 
int PLCOpenMotion::MC_MoveLinearRelative( INT AxesGroup, BOOL Execute, std::vector<double> Distance, double Velocity, double Acceleration, double Deceleration, double Jerk, CoordSystemType CoordSystem, MC_BUFFER_MODE BufferMode, MC_TRANSITION_MODE TransitionMode, std::vector<double>TransitionParameter)
{
    int datalen=sizeof(Command) + sizeof(STRUCT_MC_MoveLinearRelative)+Distance.size()*sizeof(double)+TransitionParameter.size()*sizeof(double);
    unsigned char *SendBuffer=new BYTE[datalen];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_MoveLinearRelative;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_MoveLinearRelative *data = (STRUCT_MC_MoveLinearRelative *)cmd->data;

    data->AxesGroup = AxesGroup;
    data->Execute = Execute;
    data->PosLength = Distance.size();
    data->Velocity = Velocity;
    data->Acceleration = Acceleration;
    data->Deceleration = Deceleration;
    data->Jerk = Jerk;
    data->CoordSystem=CoordSystem;
    data->BufferMode = BufferMode;
    data->TransitionMode=TransitionMode;
    data->TranParamLength=TransitionParameter.size();
    std::copy(Distance.begin(),Distance.end(),data->PosAndTran);
    std::cout<<datalen<<" "<< data->PosAndTran[0]<<' '<<data->PosAndTran[1]<<" "<<data->PosAndTran[2]<<std::endl;
    std::copy(TransitionParameter.begin(),TransitionParameter.end(),data->PosAndTran+Distance.size());

    
    int ret = McWrapper.Send(SendBuffer, datalen);
    delete[] SendBuffer;
    if (ret == datalen)
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
int PLCOpenMotion::MC_MoveLinearAbsolute( INT AxesGroup, BOOL Execute, std::vector<double>Position, double Velocity, double Acceleration, double Deceleration, double Jerk, CoordSystemType CoordSystem, MC_BUFFER_MODE BufferMode, MC_TRANSITION_MODE TransitionMode, std::vector<double>TransitionParameter)
{
    int datalen=sizeof(Command) + sizeof(STRUCT_MC_MoveLinearRelative)+Position.size()*sizeof(double)+TransitionParameter.size()*sizeof(double);
    unsigned char *SendBuffer=new BYTE[datalen];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_MoveLinearAbsolute;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_MoveLinearRelative *data = (STRUCT_MC_MoveLinearRelative *)cmd->data;

    data->AxesGroup = AxesGroup;
    data->Execute = Execute;
    data->PosLength = Position.size();
    data->Velocity = Velocity;
    data->Acceleration = Acceleration;
    data->Deceleration = Deceleration;
    data->Jerk = Jerk;
    data->CoordSystem=CoordSystem;
    data->BufferMode = BufferMode;
    data->TransitionMode=TransitionMode;
    data->TranParamLength=TransitionParameter.size();
    std::copy(Position.begin(),Position.end(),data->PosAndTran);
    std::cout<<datalen<<" "<< data->PosAndTran[0]<<' '<<data->PosAndTran[1]<<" "<<data->PosAndTran[2]<<std::endl;
    std::copy(TransitionParameter.begin(),TransitionParameter.end(),data->PosAndTran+Position.size());

    
    int ret = McWrapper.Send(SendBuffer, datalen);
    delete[] SendBuffer;
    if (ret == datalen)
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }   
}
BOOL PLCOpenMotion::Mc_AddAxisToGroup( INT AxesGroup, INT Axis, BOOL Execute, INT IdentInGroup)
{
    unsigned char SendBuffer[sizeof(Command) + sizeof(STRUCT_Mc_AddAxisToGroup)];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_Mc_AddAxisToGroup;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_Mc_AddAxisToGroup *data = (STRUCT_Mc_AddAxisToGroup *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->Axis=Axis;
    data->Execute=Execute;
    data->IdentInGroup=IdentInGroup;
    int ret = McWrapper.Send(SendBuffer, sizeof(Command) + sizeof(STRUCT_Mc_AddAxisToGroup));
    if (ret == sizeof(Command) + sizeof(STRUCT_Mc_AddAxisToGroup))
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
BOOL PLCOpenMotion::MC_RemoveAxisFromGroup( INT AxesGroup, BOOL Execute, INT IdentInGroup)
{
    unsigned char SendBuffer[sizeof(Command) + sizeof(STRUCT_MC_RemoveAxisFromGroup)];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_RemoveAxisFromGroup;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_RemoveAxisFromGroup *data = (STRUCT_MC_RemoveAxisFromGroup *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->Execute=Execute;
    data->IdentInGroup=IdentInGroup;
    int ret = McWrapper.Send(SendBuffer, sizeof(Command) + sizeof(STRUCT_MC_RemoveAxisFromGroup));
    if (ret == sizeof(Command) + sizeof(STRUCT_MC_RemoveAxisFromGroup))
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
BOOL PLCOpenMotion::MC_GroupEnable( INT AxesGroup, BOOL Execute)
{
    unsigned char SendBuffer[sizeof(Command) + sizeof(STRUCT_MC_GroupEnable)];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_GroupEnable;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_GroupEnable *data = (STRUCT_MC_GroupEnable *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->Execute=Execute;
    int ret = McWrapper.Send(SendBuffer, sizeof(Command) + sizeof(STRUCT_MC_GroupEnable));
    if (ret == sizeof(Command) + sizeof(STRUCT_MC_GroupEnable))
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
BOOL PLCOpenMotion::MC_GroupDisable( INT AxesGroup, BOOL Execute)
{
    unsigned char SendBuffer[sizeof(Command) + sizeof(STRUCT_MC_GroupDisable)];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_GroupDisable;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_GroupDisable *data = (STRUCT_MC_GroupDisable *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->Execute=Execute;
    int ret = McWrapper.Send(SendBuffer, sizeof(Command) + sizeof(STRUCT_MC_GroupDisable));
    if (ret == sizeof(Command) + sizeof(STRUCT_MC_GroupDisable))
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
BOOL PLCOpenMotion::MC_UngroupAllAxes( INT AxesGroup, BOOL Execute)
{
    unsigned char SendBuffer[sizeof(Command) + sizeof(STRUCT_MC_UngroupAllAxes)];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_UngroupAllAxes;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_UngroupAllAxes *data = (STRUCT_MC_UngroupAllAxes *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->Execute=Execute;
    int ret = McWrapper.Send(SendBuffer, sizeof(Command) + sizeof(STRUCT_MC_UngroupAllAxes));
    if (ret == sizeof(Command) + sizeof(STRUCT_MC_UngroupAllAxes))
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
BOOL PLCOpenMotion::MC_SetKinTransform(int AxesGroup,MC_KIN_REF *KinTransform,MC_EXECUTION_MODE ExecutionMode,int SpaceLen)
{
    int dateLen=sizeof(Command) + sizeof(STRUCT_MC_SetKinTransform)+SpaceLen;
    unsigned char *SendBuffer=new BYTE[dateLen];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_SetKinTransform;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_SetKinTransform *data = (STRUCT_MC_SetKinTransform *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->ExecutionMode=ExecutionMode;
    memcpy(data->KinTransform,KinTransform,SpaceLen);
    int ret = McWrapper.Send(SendBuffer, dateLen);
    delete [] SendBuffer;
    if (ret == dateLen)
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
BOOL PLCOpenMotion::MC_SetCoordinateTransform(INT AxesGroup,MC_COORD_REF *CoordTransform,MC_EXECUTION_MODE ExecutionMode,int SpaceLen)
{
    int dateLen=sizeof(Command) + sizeof(STRUCT_MC_SetCoordinateTransform)+SpaceLen;
    unsigned char *SendBuffer=new BYTE[dateLen];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_SetCoordinateTransform;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_SetCoordinateTransform *data = (STRUCT_MC_SetCoordinateTransform *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->ExecutionMode=ExecutionMode;
    memcpy(data->KinTransform,CoordTransform,SpaceLen);
    int ret = McWrapper.Send(SendBuffer, dateLen);
    delete [] SendBuffer;
    if (ret == dateLen)
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
BOOL PLCOpenMotion::MC_SetCartesianTransform(INT AxesGroup,BOOL Execute,double TransX,double TransY,double TransZ,double RotAngle1,double RotAngle2,double  RotAngle3,MC_EXECUTION_MODE ExecutionMode)
{
   int dateLen=sizeof(Command) + sizeof(STRUCT_MC_SetCartesianTransform);
    unsigned char SendBuffer[dateLen];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_SetCartesianTransform;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_SetCartesianTransform *data = (STRUCT_MC_SetCartesianTransform *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->Execute=Execute;
    data->TransX=TransX;
    data->TransY=TransY;
    data->TransZ=TransZ;
    data->RotAngle1=RotAngle1;
    data->RotAngle2=RotAngle2;
    data->RotAngle3=RotAngle3;
    data->ExecutionMode=ExecutionMode;
    int ret = McWrapper.Send(SendBuffer, dateLen);
    if (ret == dateLen)
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
BOOL PLCOpenMotion::MC_GroupInterrupt(INT AxesGroup,BOOL Execute,double Deceleration,double jerk)
{
  int dateLen=sizeof(Command) + sizeof(STRUCT_MC_GroupInterrupt);
    unsigned char SendBuffer[dateLen];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_GroupInterrupt;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_GroupInterrupt *data = (STRUCT_MC_GroupInterrupt *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->Execute=Execute;
    data->Deceleration=Deceleration;
    data->Jerk=jerk;
    int ret = McWrapper.Send(SendBuffer, dateLen);
    if (ret == dateLen)
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
BOOL PLCOpenMotion::MC_GroupContinue(INT AxesGroup,BOOL Execute)
{
  int dateLen=sizeof(Command) + sizeof(STRUCT_MC_GroupContinue);
    unsigned char SendBuffer[dateLen];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_GroupContinue;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_GroupContinue *data = (STRUCT_MC_GroupContinue *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->Execute=Execute;
    int ret = McWrapper.Send(SendBuffer, dateLen);
    if (ret == dateLen)
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
BOOL PLCOpenMotion::MC_GroupStop(INT AxesGroup,BOOL Execute,double Deceleration,double Jerk,MC_BUFFER_MODE BufferMode)
{
  int dateLen=sizeof(Command) + sizeof(STRUCT_MC_GroupStop);
    unsigned char SendBuffer[dateLen];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_GroupStop;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_GroupStop *data = (STRUCT_MC_GroupStop *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->Execute=Execute;

    data->Deceleration=Deceleration;
    data->Jerk=Jerk;
    data->BufferMode=BufferMode;

    int ret = McWrapper.Send(SendBuffer, dateLen);
    if (ret == dateLen)
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
BOOL PLCOpenMotion::MC_GroupSetPosition(INT AxesGroup,std::vector<double>  Position,BOOL Relative,CoordSystemType CoordSystem,MC_BUFFER_MODE BufferMode)
{
  int dateLen=sizeof(Command) + sizeof(STRUCT_MC_GroupSetPosition)+Position.size()*sizeof(double);
    unsigned char *SendBuffer=new BYTE[dateLen];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_GroupSetPosition;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_GroupSetPosition *data = (STRUCT_MC_GroupSetPosition *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->PosLen=Position.size();
    
    data->Relative=Relative;
    data->CoordSystem=CoordSystem;
    data->BufferMode=BufferMode;
    std::copy(Position.begin(),Position.end(),data->Position);
    delete []SendBuffer;
    int ret = McWrapper.Send(SendBuffer, dateLen);
    if (ret == dateLen)
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}
INT PLCOpenMotion::MC_GroupVisualServoMove(INT AxesGroup, BOOL Execute, std::vector<double> Position, std::vector<double> Velocity, std::vector<double> EndVelocity,std::vector<double> MaxSpeed, std::vector<double> MaxAcc, std::vector<double> MaxJerk, BOOL Relative, double LoopTime)
{
    int dateLen=sizeof(Command) + sizeof(STRUCT_MC_GroupVisualServoMove)+(Position.size()+Velocity.size()+EndVelocity.size()+MaxSpeed.size()+MaxAcc.size()+MaxJerk.size())*sizeof(double);
    unsigned char *SendBuffer=new BYTE[dateLen];
    Command *cmd = (Command *)SendBuffer;
    cmd->msgID = CMD_PLCOPENPART1_MC_GroupSetPosition;
    cmd->FeedBackFunBlockId = IDManager.GetIndex();

    STRUCT_MC_GroupVisualServoMove *data = (STRUCT_MC_GroupVisualServoMove *)cmd->data;

    data->AxesGroup=AxesGroup;
    data->AxesNum=Position.size();
    
    data->Relative=Relative;
    data->LoopTime=LoopTime;
    data->Execute=Execute;
    std::copy(Position.begin(),Position.end(),data->Positon);
    std::copy(Velocity.begin(),Velocity.end(),data->Positon+Position.size());
    std::copy(EndVelocity.begin(),EndVelocity.end(),data->Positon+Position.size()*2);
    std::copy(MaxSpeed.begin(),MaxSpeed.end(),data->Positon+Position.size()*3);
    std::copy(MaxAcc.begin(),MaxAcc.end(),data->Positon+Position.size()*4);
    std::copy(MaxJerk.begin(),MaxJerk.end(),data->Positon+Position.size()*5);
    delete []SendBuffer;
    int ret = McWrapper.Send(SendBuffer, dateLen);
    if (ret == dateLen)
    {
        return cmd->FeedBackFunBlockId;
    }
    else
    {
        IDManager.FreeID(cmd->msgID);
        return ret;
    }
}


