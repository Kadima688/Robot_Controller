#pragma once
#ifndef _COMMAND_ID_H
#define _COMMAND_ID_H
#include"TypeManager.h"
struct FBStatus
{
    INT HANDLE;
    BOOL KernelRecved;
    BOOL Done;
    BOOL InVelocity;
    BOOL Busy;
    BOOL Active;
    BOOL CommandAborted;
    BOOL Error;
    INT ErrorCode;
};
struct FBAxisState
{
    BOOL Enable;
	INT32 CurrentPosition; //
	INT32 CurrentVelocity;
	INT32 CurrentTorque;
} ;

enum ErrorCode:INT32
{
    ERRORCODE_NoError=0,
    ERRORCODE_UNKNOWER,
    ERRORCODE_AXISNUMBEROUTOFRANGE,


    ERRORCODE_FBERROR_HASBEENDISABLED=100,
    ERRORCODE_FBERROR_ISINSTANDSTILL=101,


    ERRORCODE_FBEXEC_NOTASKTOSTOP=200,

    ERRORCODE_FBEXEC_DISABLEDCANTMOVE=300,

    ERRORCODE_FBEXEC_ISHOMINGCNATPOWEROFF=400,
    ERRORCODE_FBEXEC_ISHOMINGCNATMOVE,

    ERRORCODE_FBEXEC_IsErrorStop=500,

    ERRORCODE_FBEXEC_IsHomingCantMove=600,


    ERRORCODE_FBDECODE_NOSPACE=1000,
    ERRORCODE_FBDECODE_AllocANullPointer,

    ERRORCODE_FBDEXEING_IsAborted=2000,


    ERRORCODE_FBEXEC_IsDisabled=10000,
    ERRORCODE_FBEXEC_IsMoving,
    ERRORCODE_FBEXEC_IsStandby,
    ERRORCODE_FBEXEC_IsStopping,

    ErrorCode_WORKTASKRUN_OverSpeedOrpOS=20000,

};


enum COMMAND_ID:BYTE
{
	CMD_NOP = 0,

    CMD_PLCOPENPART1_MC_POWER=50,
    CMD_PLCOPENPART1_MC_MoveAbsolute,
    CMD_PLCOPENPART1_MC_MoveRelative,
    CMD_PLCOPENPART1_MC_STOP,
    CMD_PLCOPENPART1_MC_SetPostion,
    CMD_PLCOPENPART1_MC_MoveContinuousAbsolute,
    CMD_PLCOPENPART1_MC_MoveContinuousRelative,

    CMD_PLCOPENPART1_Mc_AddAxisToGroup=100,
    CMD_PLCOPENPART1_MC_RemoveAxisFromGroup,
    CMD_PLCOPENPART1_MC_GroupEnable,
    CMD_PLCOPENPART1_MC_GroupDisable,
    CMD_PLCOPENPART1_MC_UngroupAllAxes,
    CMD_PLCOPENPART1_MC_MoveLinearRelative,
    CMD_PLCOPENPART1_MC_MoveLinearAbsolute,
    CMD_PLCOPENPART1_MC_SetKinTransform,
    CMD_PLCOPENPART1_MC_SetCoordinateTransform,
    CMD_PLCOPENPART1_MC_SetCartesianTransform,
    CMD_PLCOPENPART1_MC_GroupInterrupt,
    CMD_PLCOPENPART1_MC_GroupContinue,
    CMD_PLCOPENPART1_MC_GroupStop,
    CMD_PLCOPENPART1_MC_GroupSetPosition,
    CMD_PLCOPENPART1_MC_GroupVisualServoMove,

    CMD_PLCOPENPARTREAD_MC_ReadParameter=175,
    CMD_PLCOPENPARTREAD_MC_ReadBoolParameter,
    CMD_PLCOPENPARTREAD_MC_ReadDigitalInput,
    CMD_PLCOPENPARTREAD_MC_ReadMotionState,
    CMD_PLCOPENPARTREAD_MC_ReadAxisInfo,
    CMD_PLCOPENPARTREAD_MC_ReadAxisError,
    CMD_PLCOPENPARTREAD_MC_GroupReadActualPosition,
    CMD_PLCOPENPARTREAD_MC_GroupReadActualVelocity,
    CMD_PLCOPENPARTREAD_MC_MC_GroupReadActualAcceleration,
    CMD_PLCOPENPARTREAD_MC_MC_GroupReadStatus,
    CMD_PLCOPENPARTREAD_MC_GroupReadError,

};
enum ParameterIndex:BYTE
{
    CommandedPosition=1,
    SWLimitPos,
    SWLimitNeg,
    EnableLimitPos,
    EnableLimitNeg,
    EnablePosLagMonitoring,
    MaxPositionLag,
    MaxVelocitySystem,
    MaxVelocityAppl,
    ActualVelocity,
    CommandedVelocity,
    MaxAccelerationSystem,
    MaxAccelerationAppl,
    MaxDecelerationSystem,
    MaxDecelerationAppl,
    MaxJerkSystem,
    MaxJerkAppl
};

enum MC_DIRECTION:BYTE
{
    mcPositiveDirection=1,
    mcShortestWay,
    mcNegativeDirection,
    mcCurrentDirection
};
enum MC_EXECUTION_MODE:BYTE
{
    mcImmediately,
    mcQueued,
};
enum VisualServo_TrajectoryGeneration_ControlMode:BYTE
{
    Ruckig_Velocity_Control,
    TSpeedPlan_Position_Control,
};
enum CoordSystemType:BYTE
{
    MCS = 0, // Machine related
    ACS = 1, // Axis related
    PCS = 2  // Product or Workpiece related

};
enum MC_BUFFER_MODE:BYTE
{
    mcAborting=0,
    mcBuffered,
    mcBlendingLow,
    mcBlendingPrevious,
    mcBlendingNext,
    mcBlendingHigh

};
enum MC_TRANSITION_MODE:BYTE
{
    TMNone,
    TMStartVelocity,
    TMConstantVelocity,
    TMCornerDistance,
    TMMaxCornerDeviation=4,
};

struct Command
{
    COMMAND_ID msgID;
    BYTE    FeedBackFunBlockId;
    BYTE    data[0];
};
struct STRUCT_MC_POWER
{
    INT32 Axis;
    BOOL Enable;
    BOOL EnablePositive;
    BOOL EnableNegative;
};
struct STRUCT_MC_MOVEABSOLUTE
{
    INT32 Axis;
    BOOL Execute;
    BOOL ContinuousUpdate;
    double Position;
    double Velocity;
    double Acceleration;
    double Deceleration;
    double Jerk;
    MC_DIRECTION Direction;
    MC_BUFFER_MODE BufferMode;
};
struct STRUCT_MC_STOP
{
    INT32 Axis;
    BOOL Execute;
    double Deceleration;
    double Jerk;
};

struct STRUCT_MC_SETPOSITION
{
    INT32 Axis;
    BOOL Execute;
    double Position;
    BOOL Relative;
    MC_EXECUTION_MODE ExecutionMode;
};

struct STRUCT_MC_MOVECONTINUOUS
{
    INT32 Axis;
    BOOL Execute;
    BOOL ContinuousUpdate;
    double Position;
    double EndVelocity;
    double Velocity;
    double Acceleration;
    double Decerelation;
    double jerk;
    MC_BUFFER_MODE BufferMode;
};
struct STRUCT_Mc_AddAxisToGroup
{
    INT32 AxesGroup;
    INT32 Axis;
    BOOL Execute;
    INT32 IdentInGroup;
};
struct STRUCT_MC_RemoveAxisFromGroup
{
    INT32 AxesGroup;
    BOOL Execute;
    INT32 IdentInGroup;
};
struct STRUCT_MC_GroupEnable
{
    INT32 AxesGroup;
    BOOL Execute;
};
struct STRUCT_MC_GroupDisable
{
    INT32 AxesGroup;
    BOOL Execute;
};
struct STRUCT_MC_UngroupAllAxes
{
    INT32 AxesGroup;
    BOOL Execute;
};
struct STRUCT_MC_MoveLinearRelative
{
    INT32 AxesGroup;
    BOOL Execute;
    INT32 PosLength;
    double Velocity;
    double Acceleration;
    double Deceleration;
    double Jerk;
    CoordSystemType CoordSystem;
    MC_BUFFER_MODE BufferMode;
    MC_TRANSITION_MODE TransitionMode;
    INT TranParamLength;
    double PosAndTran[0];
};

struct STRUCT_MC_GroupInterrupt
{
    INT AxesGroup;
    BOOL Execute;
    double Deceleration;
    double Jerk;


};
struct STRUCT_MC_GroupContinue
{
    INT AxesGroup;
    BOOL Execute;
};
struct STRUCT_MC_GroupStop
{
    INT AxesGroup;
    BOOL Execute;
    double Deceleration;
    double Jerk;
    MC_BUFFER_MODE BufferMode;

};
struct STRUCT_MC_GroupSetPosition
{
    INT AxesGroup;
    INT PosLen;
    BOOL Relative;
    CoordSystemType CoordSystem;
    MC_BUFFER_MODE BufferMode;
    double Position[0];
};
struct STRUCT_MC_GroupVisualServoMove
{
    INT AxesGroup;
    BOOL Execute;
    INT AxesNum;
    BOOL Relative;
    double LoopTime;
    double Positon[0];
    VisualServo_TrajectoryGeneration_ControlMode Trajectory_Mode;
};















enum DeviceType:BYTE
{
    DeviceType_AnyAxisLinear,
    DeviceType_ThreeAxisLinearRobot,
    DeviceType_Scara,
    DeviceType_ScaraJointCsp,
    DeviceType_FIveAxisMachineXYZAB
};

struct MC_KIN_REF
{
    DeviceType Device;

};
struct MC_COORD_REF
{
    DeviceType Device;

};
struct STRUCT_MC_SetKinTransform
{
    INT AxesGroup;
    MC_EXECUTION_MODE ExecutionMode;
    MC_KIN_REF KinTransform[0];
};
struct STRUCT_MC_SetCoordinateTransform
{
    INT AxesGroup;
    MC_EXECUTION_MODE ExecutionMode;
    MC_COORD_REF KinTransform[0];
};

struct STRUCT_MC_SetCartesianTransform
{
    INT AxesGroup;
    BOOL Execute;
    double TransX;
    double TransY;
    double TransZ;
    double RotAngle1;
    double RotAngle2;
    double  RotAngle3;
    MC_EXECUTION_MODE ExecutionMode;
};


struct ACS_2_MCS_ThreeAxisLinearRobot:public MC_KIN_REF
{
    double xPulseEquivalent;
    double yPulseEquivalent;
    double zPulseEquivalent;
};
struct MCS_2_PCS_ThreeAxisLinearRobot:public MC_COORD_REF
{
    double xCoordinateSystemShift;
    double yCoordinateSystemShift;
    double zCoordinateSystemShift;
};


 









#endif
