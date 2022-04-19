#ifndef MHPARAMETERS_H
#define MHPARAMETERS_H

class MhParameters{
    friend class MhConvertPoint;
public:
typedef enum{
    perspectiveProjWithoutDistortion,//!< Perspective projection without
    perspectiveProjWithDistortion,//!< Perspective projection with distortion
    ProjWithKannalaBrandtDistortion//!< Projection with Kannala-Brandt distortion
}MhCameraParameterProType;
    MhParameters(double cam_px,double cam_py,double cam_u0,double cam_v0);
private:
    double px,py;//像素尺寸 px=f/lx py=f/ly
    double u0,v0;//像素坐标原点
    double kud;//Radial distortion (from undistorted to distorted)
    double kdu;//Radial distortion (from distorted to undistorted)
    double inv_px,inv_py;
    MhCameraParameterProType projModel;
};
#endif 