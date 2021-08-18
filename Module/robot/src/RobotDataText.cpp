#include"RobotDataText.h"

Mh::MhRobotDataText::~MhRobotDataText(){
    AxisPos_SCARA_out.close();
    CartPos_out.close();
}