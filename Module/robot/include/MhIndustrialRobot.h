#ifndef MhIndustrialRobot_H
#define MhIndustrialRobot_H

#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"eigen3/eigen3/Eigen/Dense"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include<eigen3/Eigen/Dense>
    #include<vector>
    #include"MhDh.h"
#endif

namespace Mh{
    class MhIndustrialRobot{
        public:
            MhIndustrialRobot();
            virtual ~MhIndustrialRobot();
            typedef enum{
                ROBOT_UNKNOWN,
                ROBOT_SCARA,
                ROBOT_KAWASAKI
            }MhRobotType;   
            virtual MhRobotType getRobotState(void)const {return typeRobot;}
            virtual MhRobotType setRobotState(const MhIndustrialRobot::MhRobotType newType);
            virtual void init()=0;
            virtual void set_dh_table(MhDH &_dh)=0;
            virtual void get_dh_table()=0;
            //-------------ForwardKinematics--------------
            virtual void forwardkinematics(std::vector<double>Axis,std::vector<double>Cartesian)=0;
            //------------InverseKinematics---------------
            virtual void inversekinematics(std::vector<double>Axis,std::vector<double>Cartesian,int type)=0;
            //------------Jacabian------------------------
            virtual void Jacabian(std::vector<double>Axis,std::vector<double>Cartesian)=0;
        protected:
            MhDH dh_table;
            int nDof;
        private:
            MhIndustrialRobot::MhRobotType typeRobot;
    };
}


#endif