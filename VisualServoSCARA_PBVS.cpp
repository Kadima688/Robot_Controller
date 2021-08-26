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

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && \
  (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

void MotorServoSCARA_PBVS(Mh::MhIndustrialSCARA *RobotSCARA,double opt_tagSize,bool adaptive_gain,bool opt_plot,bool opt_task_sequencing,bool opt_verbose,
                          double convergence_threshold_t,double convergence_threshold_tu);
void RealsenseServoSCARA_PBVS(Mh::MhIndustrialSCARA *RobotSCARA,double opt_tagSize,bool adaptive_gain,bool opt_plot,bool opt_task_sequencing,bool opt_verbose,
                          double convergence_threshold_t,double convergence_threshold_tu);
void VisualServoSCARA_PBVS(Mh::MhIndustrialSCARA* RobotSCARA){
  double opt_tagSzie=0.096;//二维码的尺寸大小
  bool adaptive_gain=false;//0-固定增益 1-自适应增益
  bool opt_plot=true;//是否绘制变换曲线
  bool opt_task_sequencing=false;//0-速度不连续 1-速度连续
  bool opt_verbose=false;//0-未收敛 1-收敛
  double convergence_threshold_t=0,convergence_threshold_tu=0;
  bool hasServo=0;
#ifdef USE_REALSENSE
  while(hasServo==0){
    if(RobotSCARA->ConChargeData.isEnable==1){
      sleep(1);
      RealsenseServoSCARA_PBVS(RobotSCARA,opt_tagSzie,adaptive_gain,opt_plot,opt_task_sequencing,opt_verbose,convergence_threshold_t,convergence_threshold_tu);
      hasServo=1;
    }
  }
#else
  /*具体思路步骤如下：
  1、定义相机外参信息
  2、定义相机内参信息(模拟电机的时候不需要这一步)
  3、定义cdMc、fMo，初始化相机和目标物体的位置信息
  4、根据cdMc创建位置特征和姿态特征------------------这一步的代码细节也不是很明确
  ----------5&&6是在设计控制率
  5、根据vpServo创建伺服误差特征e，同时设置eye to hand方式以及交互矩阵的类型
  6、设置自适应增益参数
  7、设置机器人的外参矩阵和速度控制模式
  8、开始获取图像并且估计目标物体相对相机的位姿--------这一步不是很清楚怎么做的
  9、更新误差特征cdMc
  10、开始计算相机速度
  11、将空间速度转换为关节速度，然后发送给内核--------这一步需要创建一个新的线程
  12、鼠标响应处理以及后续处理
  */
  while(hasServo==0){
    if(RobotSCARA->ConChargeData.isEnable==1){
      sleep(1);
      MotorServoSCARA_PBVS(RobotSCARA,opt_tagSzie,adaptive_gain,opt_plot,opt_task_sequencing,opt_verbose,convergence_threshold_t,convergence_threshold_tu);
      hasServo=1;
    }
  }
#endif
}

#endif
