#ifndef MhIndustrialKAWASAKI_H
#define MhIndustrialKAWASAKI_H

#include"MhIndustrialRobot.h"

namespace Mh{
class MhIndustrialKAWASAKI:public MhIndustrialRobot
{
public:
    MhIndustrialKAWASAKI();
    virtual ~MhIndustrialKAWASAKI();
private:
    void init();
    std::array<double,6> m_q_min;//joint min position
    std::array<double,6> m_q_max;//joint max position 
    //-----------------robot state-------
    virtual MhRobotStateType setRobotState(const MhIndustrialRobot::MhRobotStateType newState) override;
    //-----------------dh---------------
    void set_dh_table(MhDH &_dh);
    void get_dh_table();
    void set_dh_table();
    //-----------------robotconfig------
    bool loadRobotConfigFile(const char* xmlpath);
    void loadRobotConfig(const char* robotName);
    bool getElementByName(tinyxml2::XMLElement *rootElem, const char *destElemName, tinyxml2::XMLElement **destElem);
    void getAxisAttribute(tinyxml2::XMLElement *pElem, std::vector<double>& attribute);
    void getMotionParam(tinyxml2::XMLElement *pElem, DYNAMIC& dyn);
    void getCoordinate(CARTSYS cartSys, tinyxml2::XMLElement *pElem);
//--------------------------------------------------------data

public:
    MhRobotConfigData RobotConfigData;//参数配置相关的变量
};
}


#endif