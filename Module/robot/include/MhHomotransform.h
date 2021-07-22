#ifndef MhHomotransform_H
#define MhHomotransform_H

#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"eigen3/eigen3/Eigen/Dense"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include<eigen3/Eigen/Dense>
    #include<vector>
    #include"MhMath.h"
#endif

namespace Mh{
    class MhHomotransform{
        public:
            Eigen::Matrix4d rot2homomatrix(double angle,int axis);
            Eigen::Matrix4d trans2homomatrix(double distance,int axis);
            std::vector<double> homomatrix2ZYZ(Eigen::Matrix4d &T);
            Eigen::Matrix4d ZYZ2homomatrix(std::vector<double>& Cartesian);
            MhMath math;
    };
}

#endif