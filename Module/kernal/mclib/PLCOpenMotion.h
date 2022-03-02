#pragma once
#ifndef PLCOPENMOTION
#define PLCOPENMOTION
#include "MotionControl.h"
#include "FBIdManager.h"
#include <vector>

class PLCOpenMotion
{
private:
    PLCOpenMC McWrapper;
    FBIdManager IDManager;

public:
    PLCOpenMotion(/* args */);
    ~PLCOpenMotion();
    int MC_Power(INT Axis, BOOL Enable, BOOL EnablePositive, BOOL EnableNegative);
    int MC_Stop(INT Axis, BOOL Execute, double Deceleration, double Jerk);
    int MC_MoveAbsolute(INT Axis, BOOL Execute, BOOL ContinuousUpdate, double Position, double Velocity, double Acceleration, double Decerelation, double Jerk, MC_DIRECTION Direction, MC_BUFFER_MODE BufferMode);
    int MC_SetPostion(INT Axis, BOOL Execute, double Position, BOOL Relative, MC_EXECUTION_MODE ExecutionMode);
    int MC_MoveRelative(INT Axis, BOOL Execute, bool ContinuousUpdate, double Distance, double Velocity, double Acceleration, double Decerelation, double Jerk, MC_BUFFER_MODE BufferMode);
    int MC_MoveContinuousAbsolute(INT Axis, BOOL Execute, bool ContinuousUpdate, double Position, double EndVelocity, double Velocity, double Acceleration, double Decerelation, double Jerk, MC_BUFFER_MODE BufferMode);
    int MC_MoveContinuousRelative(INT Axis, BOOL Execute, bool ContinuousUpdate, double Distance, double EndVelocity, double Velocity, double Acceleration, double Decerelation, double Jerk, MC_BUFFER_MODE BufferMode);

    BOOL Mc_AddAxisToGroup(INT AxesGroup, INT Axis, BOOL Execute, INT IdentInGroup);
    BOOL MC_RemoveAxisFromGroup(INT AxesGroup, BOOL Execute, INT IdentInGroup);
    BOOL MC_GroupEnable(INT AxesGroup, BOOL Execute);
    BOOL MC_GroupDisable(INT AxesGroup, BOOL Execute);
    BOOL MC_UngroupAllAxes(INT AxesGroup, BOOL Execute);
    BOOL MC_SetKinTransform(int AxesGroup, MC_KIN_REF *KinTransform, MC_EXECUTION_MODE ExecutionMode, int SpaceLen);
    BOOL MC_SetCoordinateTransform(int AxesGroup, MC_COORD_REF *CoordTransform, MC_EXECUTION_MODE ExecutionMode, int SpaceLen);
    BOOL MC_SetCartesianTransform(INT AxesGroup, BOOL Execute, double TransX, double TransY, double TransZ, double RotAngle1, double RotAngle2, double RotAngle3, MC_EXECUTION_MODE ExecutionMode);
    BOOL MC_GroupInterrupt(INT AxesGroup, BOOL Execute, double Deceleration, double jerk);
    BOOL MC_GroupContinue(INT AxesGroup, BOOL Execute);
    BOOL MC_GroupStop(INT AxesGroup, BOOL Execute, double Deceleration, double Jerk, MC_BUFFER_MODE BufferMode);
    BOOL MC_GroupSetPosition(INT AxesGroup, std::vector<double> Position, BOOL Relative, CoordSystemType CoordSystem, MC_BUFFER_MODE BufferMode);

    INT MC_GroupVisualServoMove(INT AxesGroup, BOOL Execute, std::vector<double> Position, std::vector<double> Velocity, std::vector<double> EndVelocity,std::vector<double> MaxSpeed, std::vector<double> MaxAcc, std::vector<double> MaxJerk, BOOL Relative, double LoopTime);

    int MC_MoveLinearRelative(INT AxesGroup, BOOL Execute, std::vector<double> Distance, double Velocity, double Acceleration, double Deceleration, double Jerk, CoordSystemType CoordSystem, MC_BUFFER_MODE BufferMode, MC_TRANSITION_MODE TransitionMode, std::vector<double> TransitionParameter);
    int MC_MoveLinearAbsolute(INT AxesGroup, BOOL Execute, std::vector<double> Position, double Velocity, double Acceleration, double Deceleration, double Jerk, CoordSystemType CoordSystem, MC_BUFFER_MODE BufferMode, MC_TRANSITION_MODE TransitionMode, std::vector<double> TransitionParameter);

    // int MC_ReadActualPosition(int Axis,bool Enable,double *Position);
    // int MC_ReadActualVelocity(int Axis,bool Enable,double *Velocity);
    // int MC_ReadActualTorque(int Axis,bool Enable,double *Torque);

    double MC_ReadActualPosition(int Axis, bool Enable);
    int32_t MC_ReadActualVelocity(int Axis, bool Enable);
    double MC_ReadActualTorque(int Axis, bool Enable);

    FBStatus GetIdStatus(int id);
};

#endif
