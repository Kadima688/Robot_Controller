#ifndef MhHomotransform_H
#define MhHomotransform_H

#include<eigen3/Eigen/Dense>
#include<vector>
#include"MhMath.h"

namespace Mh{
class MhHomotransform{
public:
    Eigen::MatrixXd rot2homomatrix(double angle,int axis);
    Eigen::MatrixXd trans2homomatrix(double distance,int axis);
    std::vector<double> homomatrix2ZYZ(Eigen::MatrixXd &T);
    Eigen::MatrixXd ZYZ2homomatrix(std::vector<double>& Cartesian);
    // Eigen::MatrixXd VelocityTwist2HomoMatrix(const std::vector<double> &v,const double &delta_t);
    MhMath math;
};
}

#endif