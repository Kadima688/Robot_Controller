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

//这个是测试SCARA机器人将速度转换到世界坐标系上是否正确的

void vel_trans_test(){
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
    vpColVector vel(6);
	vel[0] = 0.01, vel[1] = 0.02, vel[2] = 0.03; vel[3] = 0.04;
	vel[4] = 0.05, vel[5] = 0.06;
    RobotSCARA.Con2DemData.axisPos_scara.a1=10;
    RobotSCARA.Con2DemData.axisPos_scara.a2=20;
    RobotSCARA.Con2DemData.axisPos_scara.d=30;
    RobotSCARA.Con2DemData.axisPos_scara.a4=40;
    vpColVector v_f(6);
    v_f=RobotSCARA.get_velocityMatrix(Mh::MhIndustrialRobot::CAMERA_FRAME,false)*vel;
    std::cout << v_f[0] << std::endl;
	std::cout << v_f[1] << std::endl;
	std::cout << v_f[2] << std::endl;
	std::cout << v_f[3] << std::endl;
	std::cout << v_f[4] << std::endl;
	std::cout << v_f[5] << std::endl;
    
}