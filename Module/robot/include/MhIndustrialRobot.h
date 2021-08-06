#ifndef MhIndustrialRobot_H
#define MhIndustrialRobot_H

#include<eigen3/Eigen/Dense>
#include<visp3/core/vpMatrix.h>
#include<visp3/core/vpColVector.h>
#include<vector>
#include"MhDh.h"
#include"MhHomotransform.h"
#include"MhIndustrialRobotPathPlan.h"

namespace Mh{
class MhIndustrialRobot{
public:
    MhIndustrialRobot();
    virtual ~MhIndustrialRobot();
    typedef enum{
        ROBOT_UNKNOWN,
        ROBOT_SCARA,
        ROBOT_KAWASAKI
    }MhRobotType;   
    typedef enum{
        STATE_STOP,
        STATE_VELOCITY_CONTROL,
        STATE_POSITON_CONTROL
    }MhRobotStateType;
    typedef enum{
        REFERENCE_FRAME,
        ARTICULAR_FRAME,
        JOINT_STATE = ARTICULAR_FRAME, 
        END_EFFECTOR_FRAME,
        CAMERA_FRAME,
        TOOL_FRAME=CAMERA_FRAME
    }MhControlFrameType;
    //-------------robotstate---------------------
    virtual MhRobotType getRobotType(void)const {return typeRobot;}
    virtual MhRobotType setRobotType(const MhIndustrialRobot::MhRobotType newType);
    virtual MhRobotStateType getRobotState(void)const {return stateRobot;}
    virtual MhRobotStateType setRobotState(const MhIndustrialRobot::MhRobotStateType newState);
    virtual MhControlFrameType getRobotFrame(void) const {return frameRobot;}
    virtual MhControlFrameType setRobotFrame(const MhIndustrialRobot::MhControlFrameType newFrame); 
    //-------------init---------------------------
    virtual void init()=0;
    //-------------DH-----------------------------
    virtual void set_dh_table(MhDH &_dh)=0;
    virtual void get_dh_table()=0;
    //-------------ForwardKinematics--------------
    virtual bool forwardkinematics(std::vector<double> Axis,std::vector<double>& Cartesian)=0;
    //------------InverseKinematics---------------
    virtual bool inversekinematics(std::vector<double>& Axis,std::vector<double> Cartesian,int type)=0;
    //------------Jacabian------------------------
    virtual void Jacabian(std::vector<double>Axis,std::vector<double>Cartesian)=0;
    virtual void get_fJc(vpMatrix& fJc)=0;
    virtual void get_cJc(vpMatrix& cJc)=0;
    virtual void get_fJe(vpMatrix& fJe)=0;
    virtual void get_eJe(vpMatrix& eJe)=0;
    virtual bool setVelocity(const MhIndustrialRobot::MhControlFrameType frame,const vpColVector& q)=0;
    virtual int get_nDof(){return nDof;}
    inline void set_verbose(bool verbose){verbose_=verbose;};
    bool saturateVelocities(const vpColVector& v_in,const vpColVector& v_max,vpColVector& v_sat,bool verbose=false);//调整速度，保证速度不超过最大限制，verbose(0-不打印第几个轴超速 1-打印)
protected:
    double maxTranslationVelocity;
    static const double maxTranslationVelocityDefault;//=0.2
    double maxRotationVelocity;
    static const double  maxRotationVelocityDefault;//=0.7
    vpMatrix fJc;
    vpMatrix cJc;
    vpMatrix fJe;
    vpMatrix eJe;
    MhDH dh_table;
    MhMath math;
    MhHomotransform transform;
    MhIndustrialRobotPathPlan path_plan;
    unsigned int nDof;
    bool verbose_;
private:
    MhIndustrialRobot::MhRobotType typeRobot;
    MhIndustrialRobot::MhRobotStateType stateRobot;
    MhIndustrialRobot::MhControlFrameType frameRobot;
};
}


#endif