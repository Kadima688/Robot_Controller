#ifndef MHCONVERTPOINT_H
#define MHCONVERTPOINT_H

#include"MhParameters.h"

class MhConvertPoint{
//------------------------像素坐标转换到图像平面坐标
public:
    inline static void PixelMetersconverPoint(const MhParameters &cam,
    const double &u, const double &v,double &x,double &y){
        switch (cam.projModel)
        {
        case MhParameters::perspectiveProjWithoutDistortion:
            PixelMetersconvertPointWithoutDistortion(cam, u, v, x, y);
            break;
        case MhParameters::perspectiveProjWithDistortion:

            break;
        case MhParameters::ProjWithKannalaBrandtDistortion:

            break;
        }
    }
    //不考虑相机畸变情况下，将特征点从pixel frame转换到normalized image frame上
    inline static void PixelMetersconvertPointWithoutDistortion(const MhParameters &cam,
    const double &u, const double &v, double &x, double &y)
    {
        x=(u-cam.u0)*cam.inv_px;
        y=(u-cam.v0)*cam.inv_py;
    }
//-----------------------图像平面坐标转换到像素平面坐标
public:
    inline static void MetersPixelconverPoint(const MhParameters &cam,
    double &x,double &y,double &u,double &v){
        switch (cam.projModel)
        {
        case MhParameters::perspectiveProjWithoutDistortion:
            MetersPixelconvertPointWithoutDistortion(cam, x,y,u,v);
            break;
        case MhParameters::perspectiveProjWithDistortion:

            break;
        case MhParameters::ProjWithKannalaBrandtDistortion:

            break;
        }
    }
    //不考虑相机畸变情况下，将特征点从pixel frame转换到normalized image frame上
    inline static void MetersPixelconvertPointWithoutDistortion(const MhParameters &cam,
    double &x,double &y,double &u,double &v)
    {
        u=x*cam.px+cam.u0;
        v=y*cam.py+cam.v0;
    }
};
#endif