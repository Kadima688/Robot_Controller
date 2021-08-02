#ifndef MhIndustrialSCARA_H
#define MhIndustrialSCARA_H

#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"eigen3/eigen3/Eigen/Dense"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include<eigen3/Eigen/Dense>
    #include"MhIndustrialRobot.h"
    
    #include"tinyxml2.h"
    #include"daraDeclaration.h"
#endif

namespace Mh{
    class MhIndustrialSCARA:public MhIndustrialRobot,public MhRobotConfigData{
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
            //-----------------robotconfig------
            bool loadRobotConfigFile(const char* xmlpath);
            void loadRobotConfig(const char* robotName);
            bool getElementByName(tinyxml2::XMLElement *rootElem, const char *destElemName, tinyxml2::XMLElement **destElem);
            void getAxisAttribute(tinyxml2::XMLElement *pElem, std::vector<double>& attribute);
            void getMotionParam(tinyxml2::XMLElement *pElem, DYNAMIC& dyn);
            void getCoordinate(CARTSYS cartSys, tinyxml2::XMLElement *pElem);
            
        private:
            void init();
            std::array<double,4> m_q_min;//joint min position
            std::array<double,4> m_q_max;//joint max position
            double z_lead;//z轴方向丝杠导程
    };
}


#endif