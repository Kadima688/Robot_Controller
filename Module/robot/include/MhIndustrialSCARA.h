#ifndef MhIndustrialSCARA_H
#define MhIndustrialSCARA_H

#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"eigen3/eigen3/Eigen/Dense"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include<eigen3/Eigen/Dense>
    #include"MhIndustrialRobot.h"
    #include"MhHomotransform.h"
    #include"MhIndustrialRobotPathPlan.h"
#endif

namespace Mh{
    class MhIndustrialSCARA:public MhIndustrialRobot{
        public:
            MhIndustrialSCARA();
            virtual ~MhIndustrialSCARA();
            //-----------------dh---------------
            void set_dh_table(MhDH &_dh);
            void get_dh_table();
            //-----------------kinematics-------
            bool forwardkinematics(std::vector<double> Axis,std::vector<double>& Cartesian);
            bool inversekinematics(std::vector<double>& Axis,std::vector<double> Cartesian,int type);
            //-----------------JACABIAN---------
            void Jacabian(std::vector<double>Axis,std::vector<double>Cartesian);
            MhMath math;
            MhHomotransform transform;
            MhIndustrialRobotPathPlan path_plan;
        private:
            void init();
            std::array<double,4> m_q_min;//joint min position
            std::array<double,4> m_q_max;//joint max position
    };
}


#endif