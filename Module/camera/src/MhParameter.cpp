#include"MhParameters.h"

MhParameters::MhParameters(double cam_px,double cam_py,double cam_u0,double cam_v0){
    this->px=cam_px;
    this->py=cam_py;
    this->u0=cam_u0;
    this->v0=cam_v0;
    this->inv_px=1. /px;
    this->inv_py=1. /py;
    this->projModel=MhCameraParameterProType::perspectiveProjWithoutDistortion;
}