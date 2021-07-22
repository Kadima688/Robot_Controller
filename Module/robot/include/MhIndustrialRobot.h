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
            //-------------robotstate---------------------
            virtual MhRobotType getRobotType(void)const {return typeRobot;}
            virtual MhRobotType setRobotType(const MhIndustrialRobot::MhRobotType newType);
            //-------------init---------------------------
            virtual void init()=0;
            //-------------DH-----------------------------
            virtual void set_dh_table(MhDH &_dh)=0;
            virtual void get_dh_table()=0;
            //-------------ForwardKinematics--------------
            virtual bool forwardkinematics(std::vector<double> Axis,std::vector<double>& Cartesian)=0;
            //------------InverseKinematics---------------
            virtual bool inversekinematics(std::vector<double>& Axis,std::vector<double> Cartesian,int type)=0;
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