#ifndef MhIndustrialRobot_H
#define MhIndustrialRobot_H

#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"eigen3/eigen3/Eigen/Dense"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include<eigen3/Eigen/Dense>
#endif

namespace Mh{
    class MhIndustrialRobot{
        public:
            MhIndustrialRobot();
            typedef enum{
                ROBOT_UNKNOWN,
                ROBOT_SCARA,
                ROBOT_KAWASAKI
            }MhRobotType;
        virtual MhRobotType getRobotState(void)const {return typeRobot;}
        virtual MhRobotType setRobotState(const MhIndustrialRobot::MhRobotType newType){return typeRobot;}
        //-------------ForwardKinematics--------------
      
        //------------InverseKinematics---------------

        //------------Jacabian------------------------

        private:
            MhIndustrialRobot::MhRobotType typeRobot;
    };
}


#endif