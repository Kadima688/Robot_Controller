#ifndef ROBOTDARATEXT_H
#define ROBOTDARATEXT_H

#include<iostream>
#include<fstream>

namespace Mh{
class MhRobotDataText{
public:
    ~MhRobotDataText();
    std::ofstream AxisPos_SCARA_out;//记录关节角度
    std::ofstream CartPos_out;//记录笛卡尔空间位置
    std::ofstream CartVel_out;//记录笛卡尔空间速度
    std::ofstream JointVel_out;//记录关节速度
    std::ofstream Looptime_out;//记录周期时间
    std::ofstream Error_out;//记录误差
    //内核参数相关文本数据的记录
    std::ofstream JointVelPulse;//记录速度脉冲
    std::ofstream JointPulse;//记录位置脉冲
};
}
#endif