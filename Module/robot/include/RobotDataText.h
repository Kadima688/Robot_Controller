#ifndef ROBOTDARATEXT_H
#define ROBOTDARATEXT_H

#include<iostream>
#include<fstream>

namespace Mh{
class MhRobotDataText{
public:
    ~MhRobotDataText();
    std::ofstream AxisPos_SCARA_out;
    std::ofstream CartPos_out;
    std::ofstream Looptime_out;
    std::ofstream Error_out;
};
}
#endif