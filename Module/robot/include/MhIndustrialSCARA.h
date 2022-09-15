#ifndef MhIndustrialSCARA_H
#define MhIndustrialSCARA_H

#include<eigen3/Eigen/Dense>
#include<visp3/core/vpMatrix.h>
#include"MhIndustrialRobot.h"
#include"tinyxml2.h"
#include"daraDeclaration.h"
#include"GlobalDefine.h"
#include"RobotDataText.h"
namespace Mh{
class MhIndustrialSCARA
:public MhIndustrialRobot
#ifndef USE_MCKERNEL
,public MhMotionkernel
#endif 
{
public:
    MhIndustrialSCARA();
    virtual ~MhIndustrialSCARA();
    //-----------------robot state-------
    virtual MhRobotStateType setRobotState(const MhIndustrialRobot::MhRobotStateType newState) override;
    //-----------------dh---------------
    void set_dh_table(MhDH &_dh);
    void get_dh_table();
    void set_dh_table();
    //-----------------kinematics-------
    bool forwardkinematics(std::vector<double> Axis,std::vector<double>& Cartesian);
    bool inversekinematics(std::vector<double>& Axis,std::vector<double> Cartesian,int type);
    //-----------------JACABIAN---------
    void Jacabian(std::vector<double>Axis,std::vector<double>Cartesian);
    void get_fJc(vpMatrix& fJc);
    void get_cJc(vpMatrix& cJc);
    void get_fJe(vpMatrix& fJe);
    void get_eJe(vpMatrix& eJe);
    //-----------------robotconfig------
    bool loadRobotConfigFile(const char* xmlpath);
    void loadRobotConfig(const char* robotName);
    bool getElementByName(tinyxml2::XMLElement *rootElem, const char *destElemName, tinyxml2::XMLElement **destElem);
    void getAxisAttribute(tinyxml2::XMLElement *pElem, std::vector<double>& attribute);
    void getMotionParam(tinyxml2::XMLElement *pElem, DYNAMIC& dyn);
    void getCoordinate(CARTSYS cartSys, tinyxml2::XMLElement *pElem);
    //-----------------setVelocity
    bool setVelocity(const MhIndustrialRobot::MhControlFrameType frame,const vpColVector& vel);
    void setCartVelocity(const MhIndustrialRobot::MhControlFrameType frame,const vpColVector &vel);
    void setJointVelocity(const vpColVector &qdot);
    #ifndef USE_KERNEL 
    #ifndef USE_MCKERNEL
    void setJointVelocity_virtual(const vpColVector &qdot);
    #endif
    #endif
    //----------------others
    vpMatrix get_velocityMatrix(const MhIndustrialRobot::MhControlFrameType frame,bool fixed);//获取速度转换矩阵 fixed(0-相对移动坐标系 1-相对固定坐标系)
    void set_a4_Compensation(double Compensation){a4_Compensation=Compensation;};
    inline double get_a4_Compensation(){return a4_Compensation;};
    void set_eMc(vpHomogeneousMatrix& eMc){m_eMc=eMc;}
    vpHomogeneousMatrix get_eMc()const {return m_eMc;}
private:
    void init();
    std::array<double,4> m_q_min;//joint min position
    std::array<double,4> m_q_max;//joint max position
    double z_lead;//z轴方向丝杠导程
    double a4_Compensation;//第四个轴转动带来的补偿量
protected:
    vpHomogeneousMatrix m_eMc;//相机外参矩阵
//---------------------------------------------------------motor initial
public:
    #ifndef USE_MCKERNEL
    void RobotMotorInitial();//设置轴的限位、轴的位移模式
    void RobotOpenConti();//插补运动前，开启插补缓冲区
    void RobotDynInitial();//设置轴的速度、加速度、加加速度
    void RobotInterpolationDynInitial();//设置插补速度、加速度和加加速度
    void RobotCloseConti();//关闭插补缓冲区
    #endif
//--------------------------------------------------------path planing
public:
    void FollowPathMove(std::map<int,std::vector<double>>& record,int PathType);
//--------------------------------------------------------data
public:
    MhConChargeMutex  RobotConChargeDataMutex;//管理控制器变量读写的锁
    MhRobotConfigData RobotConfigData;//参数配置相关的变量
    MhDem2ControlData Dem2ConData;//示教器传输到控制器的变量
    MhControl2DemData Con2DemData;//控制器传输到示教器的变量
    MhControlChargeData ConChargeData;//存在于控制器的全局控制变量
    //-------------------关于data的接口
    void set_retn(int r,int KERNEL_TYPE);
//--------------------------------------------------------DataText output
public:
    MhRobotDataText MhRobotText;//关于文本记录的类
    bool judge=false;
    int count=0;//记录发送的指令次数
//-------------------------------------------------------视觉伺服误差相关变量
    double threshold_t;
    double threshold_tu;
    double error_t;
    double error_tu;
    bool switch_control=false;
};
}


#endif
