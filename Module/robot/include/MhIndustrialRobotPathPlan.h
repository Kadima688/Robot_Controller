#ifndef MhIndustrialRobotPathPlan_H
#define MhIndustrialRobotPathPlan_H

#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"eigen3/eigen3/Eigen/Dense"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include<eigen3/Eigen/Dense>
#endif

namespace Mh{
    class MhIndustrialRobotPathPlan{
        public:
            MhIndustrialRobotPathPlan();
            double LINE_PATH_DIFF;
    };
}

#endif