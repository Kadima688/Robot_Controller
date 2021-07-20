#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"robot-controller/Module/robot/include/MhIndustrialRobot.h"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include"MhIndustrialRobot.h"
#endif

Mh::MhIndustrialRobot::MhIndustrialRobot(){
    typeRobot=ROBOT_UNKNOWN;
}