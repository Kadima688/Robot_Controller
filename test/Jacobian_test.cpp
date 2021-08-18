#include<iostream>
#include<unistd.h>
#include"MhIndustrialSCARA.h"
#include<visp3/core/vpConfig.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include<visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>
#include<visp3/sensor/vpRealSense2.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpPlot.h>
#include<visp3/core/vpEigenConversion.h>

// 测试雅可比得正确性，和之前在win系统得雅可比算法进行对比
void Jacabian_test(){
    Mh::MhIndustrialSCARA RobotSCARA;
    if(!RobotSCARA.loadRobotConfigFile("RobotConfig_CoolDrive.xml")){
        return ;
    }  
    vpHomogeneousMatrix eMc;
    eMc[0][0] = 1; eMc[0][1] = 0; eMc[0][2] = 0; eMc[0][3] = 0;
	eMc[1][0] = 0; eMc[1][1] = 1; eMc[1][2] = 0; eMc[1][3] = 0;
	eMc[2][0] = 0; eMc[2][1] = 0; eMc[2][2] = 1; eMc[2][3] = 0/*-183.0593653*/;
	eMc[3][0] = 0; eMc[3][1] = 0; eMc[3][2] = 0; eMc[3][3] = 1;
    RobotSCARA.set_eMc(eMc);
    RobotSCARA.Con2DemData.axisPos_scara.a1=15;
    RobotSCARA.Con2DemData.axisPos_scara.a2=16;
    RobotSCARA.Con2DemData.axisPos_scara.d=17;
    RobotSCARA.Con2DemData.axisPos_scara.a4=18;
    vpMatrix fJc;
    RobotSCARA.get_fJc(fJc);
    std::cout<<fJc<<std::endl;
}