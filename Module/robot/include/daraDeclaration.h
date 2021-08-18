#ifndef DARADECLARATION_H
#define DARADECLARATION_H

#include"tinyxml2.h"
#include<string>
#include<iostream>
#include<map>
#include<vector>
#include<mutex>
//--------------------------数据类型定义
struct AXISPOS
{
    double a1, a2, a3, a4, a5, a6, aux1, aux2, aux3, aux4, aux5;
};
struct AXISPOS_SCARA
{
public:
    AXISPOS_SCARA operator=(const std::vector<double>& inverse_ans){
        if(inverse_ans.size()==4){
            this->a1=inverse_ans[0];
            this->a2=inverse_ans[1];
            this->d=inverse_ans[2];
            this->a4=inverse_ans[3];
            return *this;
        }
        else{
            this->a1=0;
            this->a2=0;
            this->d=0;
            this->a4=0;
            return *this;
        }
    }
    double a1, a2, d, a4;//分别对应三个旋转自由度和一个直线自由度
};
struct DYNAMIC
{
    double velAxis, accAxis, decAxis, jerkAxis, velPath, accPath, decPath, jerkPath, velOri, accOri, decOri, jerkOri;
};
struct CARTSYS
{
    double x, y, z, a, b, c;
};
struct CARTPOS
{
    CARTPOS():x(0),y(0),z(0),a(0),b(0),c(0){} 
    CARTPOS operator=(const std::vector<double>& forward_ans){
        if(forward_ans.size()==6){
            this->x=forward_ans[0];
            this->y=forward_ans[1];
            this->z=forward_ans[2];
            this->a=forward_ans[3];
            this->b=forward_ans[4];
            this->c=forward_ans[5];
            return *this;
        }
        else{
            this->x=0;
            this->y=0;
            this->z=0;
            this->a=0;
            this->b=0;
            this->c=0;
            return *this;
        }       
    }
    double x, y, z, a, b, c, aux1, aux2, aux3, aux4, aux5;
    int mode;
};
struct DRIVEPOS
{
    double d1, d2, d3, d4, d5, d6;
};
struct CARTREFSYS
{
    char *baseRefSys;
    double x, y, z, a, b, c;
};


//六轴机器人耦合关系
struct COUPLING//六轴机器人后三轴耦合相关的数据类型
{
    double axis4ToAxis5;
	double axis4ToAxis6;
	double axis5ToAxis6;
};
//示教器传来的关于视觉伺服的结构体
struct  VISUALSERVO
{
    int cameraIntrinsicCalibration; //是否进行相机畸变矫正的标识符
    int cameraExtrinsicCalibration; //是否进行手眼标定的标识符
    int method; //视觉伺服方式，method为0时不进行视觉伺服，1为PBVS，2为IBVS
    CARTPOS desPos; //目标位姿距离
    double error1; //最大允许位置误差
    double error2;//最大允许姿态误差
    double error3; //最大允许图像误差
    int ifEnd; //是否结束视觉伺服的标志符
};
struct MhRobotConfigData{
// public:
//     std::vector<const char *>get_robotNameList(){return robotNameList;};
//     std::vector<double>get_alpha(){return alpha;};
//     std::vector<double>get_a_(){return a_;};
//     std::vector<double>get_d(){return d;};
//     std::vector<double>get_offset1(){return offset1;};
//     std::vector<double>get_maxPos(){return maxPos;};
//     std::vector<double>get_minPos(){return minPos;};
//     std::vector<double>get_maxVel(){return maxVel;};
//     std::vector<double>get_minVel(){return minVel;};
//     std::vector<double>get_maxAcc(){return maxAcc;};
//     std::vector<double>get_maxDec(){return maxDec;};
//     std::vector<double>get_maxJerk(){return maxJerk;};
//     std::vector<double>get_offset2(){return offset2;};
//     std::vector<double>get_direction(){return direction;};
//     std::vector<double>get_ratio(){return ratio;};
//     std::vector<double>get_encoder(){return encoder;};
//     COUPLING get_coupling(){return coupling;};
//     DYNAMIC get_dynamic(){return dynamic;};
//     DYNAMIC get_jogspeed(){return jogspeed;};
//     CARTSYS get_base(){return base;};
//     CARTSYS get_tool(){return tool;};
//     std::vector<double> get_pulseEquivalent(){return pulseEquivalent;};
//     double get_averagePulseEquivalent(){return averagePulseEquivalent;}; 
//     double get_maxSyntheticVel(){return maxSyntheticVel;};
//     double get_maxSyntheticAcc(){return maxSyntheticAcc;};
//     double get_maxSyntheticJerk(){return maxSyntheticJerk;};
// public:
//     void set_robotNameList(std::vector<const char *> x){robotNameList=x;};
//     void set_alpha(std::vector<double> x){alpha=x;};
//     void set_a_(std::vector<double> x){a_=x;};
//     void set_d(std::vector<double> x){d=x;};
//     void set_offset1(std::vector<double> x){offset1=x;};
//     void set_maxPos(std::vector<double> x){maxPos=x;};
//     void set_minPos(std::vector<double> x){ minPos=x;};
//     void set_maxVel(std::vector<double> x){ maxVel=x;};
//     void set_minVel(std::vector<double> x){ minVel=x;};
//     void set_maxAcc(std::vector<double> x){ maxAcc=x;};
//     void set_maxDec(std::vector<double> x){ maxDec=x;};
//     void set_maxJerk(std::vector<double> x){maxJerk=x;};
//     void set_offset2(std::vector<double> x){offset2=x;};
//     void set_direction(std::vector<double> x){direction=x;};
//     void set_ratio(std::vector<double> x){ratio=x;};
//     void set_encoder(std::vector<double> x){encoder=x;};
//     void set_coupling(COUPLING x){coupling=x;};
//     void set_dynamic(DYNAMIC x){dynamic=x;};
//     void set_jogspeed(DYNAMIC x){jogspeed=x;};
//     void set_base(CARTSYS x){base=x;};
//     void set_tool(CARTSYS x){tool=x;};
//     void set_pulseEquivalent(std::vector<double> x){pulseEquivalent=x;};
//     void set_averagePulseEquivalent(double x){averagePulseEquivalent=x;}; 
//     void set_maxSyntheticVel(double x){maxSyntheticVel=x;};
//     void set_maxSyntheticAcc(double x){maxSyntheticAcc=x;};
//     void set_maxSyntheticJerk(double x){maxSyntheticJerk=x;};
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
//示教器发送给控制器的数据
struct MhDem2ControlData
{
    int emergeStop=0;//机器人是否急停，0-未急停 1-急停
    int enableState=1; //机器人使能状态，0：未使能，1：使能
    int operateMode; //机器人操作模式，0：手动，1：自动
    int activeState; //程序的激活状态，0：未激活，1：激活
    int runState; //程序的运行状态，0：暂停，1：运行中
    int step; //程序的执行方式，0：连续，1：单步
    int jog; //手动示教时所选坐标系，0：轴关节坐标系，1：世界坐标系，2：工具手坐标系
    int coordinate; //手动示教时选中的坐标，0：未示教，1-6：第x个坐标
    int upOrDown; //手动示教时对选中坐标的增大或减小，0：减小，1：增大
    int ovr=5; //相对最大运动参数的百分比
    int progLine; //程序执行到的行数
    int transferZip; //是否传输文件压缩包，0：不传输，1：传输
    int zipSize; //压缩包大小
    std::string varProgName; //主程序名称
    std::map<std::string, int> chosen_Robot_name;//上位机选中的机器人名称
    VISUALSERVO visualServo; //视觉伺服参数结构体
};
//示教器发送给控制器的程序
struct MhControl2DemData
{
    struct AXISPOS axisPos; //机器人在轴关节坐标系下的位置坐标--------------六轴机器人
    struct AXISPOS_SCARA axisPos_scara;//机器人在轴关节坐标系下的位置坐标----------SCARA机器人
    struct CARTPOS cartPos; //机器人在世界坐标系下的位置坐标
    struct DRIVEPOS drivePos; //机器人伺服电机的位置坐标--------------六轴机器人，SCARA机器人由于只有4个自由度，所以d5 d6都为0
    int runProg; //是否运行程序，0：否，1：是
    int hasReachSingularity; //是否到达机器人的奇异点，0：否，1：是。用于控制示教时，机器人在直角坐标系下到达奇异点后将不再移动
    std::vector<std::pair<int, std::string>> infoReport; //信息报告的内容 0：通知，1：警告，2：错误，3：严重错误
    int Dynovr_ovr;//在所运行程序包含Dynovr(ovr1)指令的时候，ovr1将被赋值到Dynovr_ovr中，并且传输到示教器上
    int Dynovr_Judge;//判断是否接受从示教器传来的ovr参数的标志，0：接收，1：不接受
    int Tool_Judge;//传输给示教器的标志位，1:表示当前运行的程序具有Tool指令 0:当前程序不具有Tool指令
    int Dynovr_Num;//判断当前所运行的程序包含多少个Dynovr的指令
    int Tool_Num;//判断当前所运行的程序包含几个Tool的指令
    int Refsys_Judge;//传输给示教器的标志位 1:表示当前运行的程序具有Refsys指令 0：当前程序不具有Refsys指令
    int Refsys_Num;//判断当前所运行的程序包含几个Refsys指令
    std::vector<std::pair<std::string, std::string>>Robot_name_type;//存储机器人名称和类型的容器
    int cameraIntrinsicCalibrationOk; //是否完成相机畸变矫正，0：否，1：是
    int cameraExtrinsicCalibrationOk; //是否完成手眼标定，0：否，1：是
    int visualServoOk; //是否完成视觉伺服，0：否，1：是
};
//存在于控制器项目的全局变量
struct MhControlChargeData
{
    struct CARTSYS curTool; //当前的工具坐标系
    struct CARTREFSYS curRefSys; //当前的参考坐标系
    struct DYNAMIC curDynamic; //当前的机器人运动参数
    struct DYNAMIC retDynamic; //保留的运动参数
    int selectDyn=2; //选择使用的运动参数，0：手动&点位，1：手动&轨迹，2：自动&点位，3：自动&轨迹
    unsigned long contiRunState; //机器人插补运动状态，0：正在运动，1：暂停中，2：停止状态，3：未启动，4：空闲
    unsigned long allAxisState; //机器人所有轴的运动完成情况，0：正在运动，1：处于停止状态
    int startProg; //是否已进入程序编译执行阶段，0：否，1：是
    int endProg; //是否从控制器端结束程序，0：否，1：是
    int startFromActive; //判断是否从程序激活状态变为程序运行状态，0：否，1：是
    int startFromStop; //判断是否从程序停止状态变为程序运行状态，如果是则不需重新编译，只需从上次停止的位置开始继续执行程序，0：否，1：是
    int startFromEnd; //判断是否从程序结束状态变为程序运行状态，如果是则会重新编译并执行程序，0：否，1：是
    int stopFromStart; //判断是否从程序运行状态变为程序停止状态，0：否，1：是
    int endFromStart;  //判断是否从程序运行状态变为程序终止状态，0：否，1：是
    int transferZipFinished; //判断压缩包是否已经传输完成并已解压，0：否，1：是
    int startServo=0; //判断是否已经开始视觉伺服，0：否，1：是
    int endServo; //用于控制器结束视觉伺服，0：不结束，1：结束
    bool error; //判断程序运行中是否出错，false：未出错，true：出错
    int retn; //内核API返回值，为0时表示调用成功
    long targetPos[6]; //继续运动时的目标位置-------6轴机器人
    long targerPos_SCARA[4];//继续运动时的目标位置------4轴机器人
    int Robot_enable_State_Judge;//判断数据读取线程是否搜集数据的标志，0:不搜集 1:搜集
    int contiOpen; //判断是否开启了连续插补缓冲区
    int isSingular; //判断是否运动到奇异点了，0：否，1：是
    std::string robotType;//当前机器人类型
    int isEnable=0;//判断各轴是否已经使能 0：否 1：是
};
//管理变量的锁
struct MhConChargeMutex{
    std::mutex retn_mutex;
};
#endif
