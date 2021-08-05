#ifndef MhIndustrialSCARA_H
#define MhIndustrialSCARA_H

#include<eigen3/Eigen/Dense>
#include<visp3/core/vpMatrix.h>
#include"MhIndustrialRobot.h"
#include"tinyxml2.h"
#include"daraDeclaration.h"
#include"MhMotionKernel.h"

namespace Mh{
    class MhIndustrialSCARA:public MhIndustrialRobot,public MhRobotConfigData,
    public MhDem2ControlData,public MhControl2DemData,public MhControlChargeData,
    public MhMotionkernel
    {
        public:
            MhIndustrialSCARA();
            virtual ~MhIndustrialSCARA();
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
    };
}


#endif