#include"MhIndustrialRobot.h"
#include"os.h"
#include<math.h>
const double Mh::MhIndustrialRobot::maxTranslationVelocityDefault=0.2;
const double Mh::MhIndustrialRobot::maxRotationVelocityDefault=0.7;

Mh::MhIndustrialRobot::MhIndustrialRobot()
:typeRobot(MhIndustrialRobot::ROBOT_UNKNOWN),stateRobot(MhIndustrialRobot::STATE_STOP),frameRobot(MhIndustrialRobot::CAMERA_FRAME),
maxRotationVelocity(maxRotationVelocityDefault),maxTranslationVelocity(maxTranslationVelocityDefault),
fJc(),cJc(),fJe(),eJe(),
dh_table(),math(),transform(),path_plan(),
nDof(0),verbose_(true)
{ 
}

Mh::MhIndustrialRobot::~MhIndustrialRobot(){

}

Mh::MhIndustrialRobot::MhRobotType Mh::MhIndustrialRobot::setRobotType(const MhIndustrialRobot::MhRobotType newType){
    typeRobot=newType;
    return newType;
}

Mh::MhIndustrialRobot::MhRobotStateType Mh::MhIndustrialRobot::setRobotState(const MhIndustrialRobot::MhRobotStateType newState){
    stateRobot=newState;
    return stateRobot;
}

Mh::MhIndustrialRobot::MhControlFrameType Mh::MhIndustrialRobot::setRobotFrame(const MhIndustrialRobot::MhControlFrameType newFrame){
    frameRobot=newFrame;
    return frameRobot;
}

bool Mh::MhIndustrialRobot::saturateVelocities(const vpColVector& v_in,const vpColVector& v_max,vpColVector& v_sat,bool verbose){
    unsigned int size=v_in.size();
    if(size!=v_max.size()){
        EC_TRACE("Velocity vectors should have the same dimension\n");
        return false;
    }
    double scale=1;
    for(unsigned int i=0;i<size;++i){
        double v_i=fabs(v_in[i]);
        double v_max_i=fabs(v_max[i]);
        if(v_i>v_max_i){
            double scale_i=v_max_i/v_i;
            if(scale_i<scale){
                scale=scale_i;
            }
            if(verbose){
                EC_TRACE("Excess velocity %f ,axis number: %n\n",v_in[i],i);
            }
        }
    }
    v_sat.resize(size);
    v_sat=v_in*scale;
    return true;
}

