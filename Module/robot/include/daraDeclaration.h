#ifndef DARADECLARATION_H
#define DARADECLARATION_H

struct DYNAMIC
{
    double velAxis, accAxis, decAxis, jerkAxis, velPath, accPath, decPath, jerkPath, velOri, accOri, decOri, jerkOri;
};
struct CARTSYS
{
    double x, y, z, a, b, c;
};
struct COUPLING//六轴机器人后三轴耦合相关的数据类型
{
    double axis4ToAxis5;
	double axis4ToAxis6;
	double axis5ToAxis6;
};


struct MhRobotConfigData{
    protected:
            tinyxml2::XMLDocument doc;
            tinyxml2::XMLElement* rootElem;
            tinyxml2::XMLElement* robotElem;
            std::vector<const char *> robotNameList;
            std::vector<double> alpha;//连杆扭角
            std::vector<double> a_;//连杆长度
            std::vector<double> d;//连杆偏距
            std::vector<double> offset1;//关节角零点偏移
            std::vector<double> maxPos;//各轴正向最大角度
            std::vector<double> minPos;//各轴负向最大角度
            std::vector<double> maxVel;//各轴最大加速度
            std::vector<double> minVel;//各轴最大减速度
            std::vector<double> maxAcc;//各轴最大加速度
            std::vector<double> maxDec;//各轴最大减速度
            std::vector<double> maxJerk;//各轴最大加加速度
            std::vector<double> offset2;//各轴关节零点相对伺服零点偏移量
            std::vector<double> direction;//各轴的运动方向
            std::vector<double> ratio;//各轴与电机间的减速比
            std::vector<double> encoder;//各轴电机的编码器分辨率
            COUPLING coupling;
            DYNAMIC dynamic;
            DYNAMIC jogspeed;
            CARTSYS base;
            CARTSYS tool;
            std::vector<double> pulseEquivalent;//每个轴的脉冲当量
            double averagePulseEquivalent;//六轴平均脉冲当量
            double maxSyntheticVel;//轴最大合成速度
            double maxSyntheticAcc; //轴最大合成加速度
            double maxSyntheticJerk; //轴最大合成加加速度    
};

#endif
