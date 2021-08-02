#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"robot-controller/Module/robot/include/MhIndustrialRobot.h"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include"MhIndustrialRobot.h"
#endif

Mh::MhIndustrialRobot::MhIndustrialRobot()
:typeRobot(MhIndustrialRobot::ROBOT_UNKNOWN),
dh_table(),math(),transform(),path_plan(),
nDof(0)
{ 
}

Mh::MhIndustrialRobot::~MhIndustrialRobot(){

}

Mh::MhIndustrialRobot::MhRobotType Mh::MhIndustrialRobot::setRobotType(const MhIndustrialRobot::MhRobotType newType){
    typeRobot=newType;
    return newType;
}

