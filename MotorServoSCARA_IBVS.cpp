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
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpPlot.h>
#include<visp3/core/vpEigenConversion.h>
#include<random>
void MotorServoSCARA_IBVS(Mh::MhIndustrialSCARA *RobotSCARA,double opt_tagSzie,bool adaptive_gain,bool opt_plot,bool opt_task_sequencing,bool opt_verbose,
                          double convergence_threshold_t,double convergence_threshold_tu,double convergence_threshold_image)
{ 
    int dynamic_simulation=0;//0:不开启静态模拟  1：开启静态模拟
    #ifndef USE_KERNEL
    //设置当前的轴关节位置和空间位置
    RobotSCARA->Con2DemData.axisPos_scara.a1=-39.9150;
    RobotSCARA->Con2DemData.axisPos_scara.a2=84.2464;
    RobotSCARA->Con2DemData.axisPos_scara.d=54.75512;
    RobotSCARA->Con2DemData.axisPos_scara.a4=161.4467;
    std::vector<double> cartesian;
    std::vector<double> scara_input={RobotSCARA->Con2DemData.axisPos_scara.a1,RobotSCARA->Con2DemData.axisPos_scara.a2,RobotSCARA->Con2DemData.axisPos_scara.d,RobotSCARA->Con2DemData.axisPos_scara.a4};        
    if(RobotSCARA->forwardkinematics(scara_input,cartesian)){
        RobotSCARA->Con2DemData.cartPos=cartesian;
    }
    #endif
    //1、设置相机外参信息
    vpHomogeneousMatrix eMc;
    eMc[0][0] = 0.866; eMc[0][1] = -0.5; eMc[0][2] = 0; eMc[0][3] = 0.02488820463;
	eMc[1][0] = 0.5; eMc[1][1] = 0.866; eMc[1][2] = 0; eMc[1][3] = -0.0364688832;
	eMc[2][0] = 0; eMc[2][1] = 0; eMc[2][2] = 1; eMc[2][3] = 0.001;
	eMc[3][0] = 0; eMc[3][1] = 0; eMc[3][2] = 0; eMc[3][3] = 1;
    Eigen::MatrixXd eTc;
    vp::visp2eigen(eMc,eTc);eTc(0,3)*=1000;eTc(1,3)*=1000;eTc(2,3)*=1000;
    //2、定义cdMc、fMo初始化相机和目标物体的位置信息
    vpHomogeneousMatrix cdMc,oMo,cMo;
    vpHomogeneousMatrix cdMo(vpTranslationVector(0,0,opt_tagSzie*3),vpRotationMatrix({1,0,0,0,-1,0,0,0,-1}));
    vpHomogeneousMatrix fMo(vpTranslationVector(0.487,0.041,-0.25),vpThetaUVector(vpRzyzVector(0,0,M_PI/3)));
    cdMc=cdMo*cMo.inverse();
    //3、开始创建图像误差特征-----这一步我们是选择空间位姿误作为伺服特征。在这里可以选择其他信息作为特征，如图像的点、线、甚至是深度信息。
    std::vector<vpFeaturePoint> p(4),pd(4);
    std::vector<vpPoint> point(4);
    point[0].setWorldCoordinates(-opt_tagSzie/2.,-opt_tagSzie/2.,0);
    point[1].setWorldCoordinates(opt_tagSzie/2.,-opt_tagSzie/2.,0);
    point[2].setWorldCoordinates(opt_tagSzie/2.,opt_tagSzie/2.,0);
    point[3].setWorldCoordinates(-opt_tagSzie/2.,opt_tagSzie/2.,0);
    //4、开始建立视觉伺服任务
    vpServo task;
    for(size_t i=0;i<p.size();i++){
        task.addFeature(p[i],pd[i]);
    }
    task.setServo(vpServo::EYEINHAND_CAMERA);//手眼系统，这里还可以选择其他的
    task.setInteractionMatrixType(vpServo::CURRENT);//每次迭代都更新交互矩阵
    //5、设置自适应参数
    if(adaptive_gain){//跟伺服收敛速度、误差变化的速度都有关系
        vpAdaptiveGain lambda(1.5,0.4,30);//lambda(0)=4,lambda(oo)=0.4 and lambda'(0)=30;
        task.setLambda(lambda);
    }
    else{
        task.setLambda(0.5);
    }
    //6、绘图对象初始化
    vpPlot *plotter=nullptr;
    int iter_plot=0;
    if(opt_plot){
        plotter=new vpPlot(2,250*2,500,100,200,"Realtime curves plotter");
        plotter->setTitle(0,"Visual features error");
        plotter->setTitle(1,"Camera velocity");
        plotter->initGraph(0,8);
        plotter->initGraph(1,6);
        plotter->setLegend(0, 0, "error_feat_p1_x");
        plotter->setLegend(0, 1, "error_feat_p2_x");
        plotter->setLegend(0, 2, "error_feat_p3_x");
        plotter->setLegend(0, 3, "error_feat_p4_x");
        plotter->setLegend(0, 4, "error_feat_p5_x");
        plotter->setLegend(0, 5, "error_feat_p6_x");
        plotter->setLegend(0, 6, "error_feat_p7_x");
        plotter->setLegend(0, 7, "error_feat_p8_x");
        plotter->setLegend(1, 0, "vc_x");
        plotter->setLegend(1, 1, "vc_y");
        plotter->setLegend(1, 2, "vc_z");
        plotter->setLegend(1, 3, "wc_x");
        plotter->setLegend(1, 4, "wc_y");
        plotter->setLegend(1, 5, "wc_z");
    }
    bool final_quit=false;
    bool has_converge=false;
    bool send_velocitys=false;
    bool servo_started=false;

    static double t_init_servo=vpTime::measureTimeMs();

    RobotSCARA->set_eMc(eMc);//设置机器人的外参矩阵
    RobotSCARA->setRobotState(Mh::MhIndustrialRobot::STATE_VELOCITY_CONTROL);//设置为速度控制模式

    //-------动态模拟相关参数
    int dynamic_n=1;//运动段数 
    int segment=100;
    double des_x=0.450,des_y=0.045,des_z=-0.2;//fMo目标位置
    double ini_x=fMo[0][3],ini_y=fMo[1][3],ini_z=fMo[2][3];

    while(!has_converge &&!final_quit){
        double t_start=vpTime::measureTimeMicros();
        //7、开始更新cMo=eMc.inverse()*fMe.inverse()*fMo;---模拟detect函数的作用
        vpHomogeneousMatrix fMe;
        std::vector<double> catesian;catesian.clear();
        catesian.push_back(RobotSCARA->Con2DemData.cartPos.x);catesian.push_back(RobotSCARA->Con2DemData.cartPos.y);
        catesian.push_back(RobotSCARA->Con2DemData.cartPos.z);catesian.push_back(RobotSCARA->Con2DemData.cartPos.a);
        catesian.push_back(RobotSCARA->Con2DemData.cartPos.b);catesian.push_back(RobotSCARA->Con2DemData.cartPos.c);
        Eigen::MatrixXd fTe=RobotSCARA->transform.ZYZ2homomatrix(catesian);
        vp::eigen2visp(fTe,fMe);fMe[0][3]/=1000;fMe[1][3]/=1000;fMe[2][3]/=1000;
        cMo=eMc.inverse()*fMe.inverse()*fMo;

        //---------------------------------------动态模拟生成器
        if(dynamic_simulation && send_velocitys){
            unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::default_random_engine gen(seed);
            if(dynamic_n<segment+1){
                fMo[0][3]+=(des_x-ini_x)/segment;
                fMo[1][3]+=(des_y-ini_y)/segment;
                fMo[2][3]+=(des_z-ini_z)/segment;
                dynamic_n++;
            }
            else{
                std::normal_distribution<double> dis_x(des_x,0.01);
                std::normal_distribution<double> dis_y(des_y,0.01);
                std::normal_distribution<double> dis_z(des_z,0.01);
                fMo[0][3]=dis_x(gen);
                fMo[1][3]=dis_y(gen);
                fMo[2][3]=dis_z(gen);
            }
        }
        //---------------------------------------动态模拟生成器

        vpColVector v_c(6);
        bool first_time=true;
        if(first_time){
            //9、处于安全，避免PI旋转
            std::vector<vpHomogeneousMatrix>v_oMo(2),v_cdMc(2);
            v_oMo[1].buildFrom(0,0,0,0,0,M_PI);
            for(size_t i=0;i<2;i++){
                v_cdMc[i]=cdMo*v_oMo[i]*cMo.inverse();
            }
            if(std::fabs(v_cdMc[0].getThetaUVector().getTheta())<
            std::fabs(v_cdMc[1].getThetaUVector().getTheta())){
                oMo=v_oMo[0];
            }
            else{
                oMo=v_oMo[1];std::cout<<"Desired frame modified to avoid PI rotation of the camera"<<std::endl;
            }
            for(size_t i=0;i<point.size();i++){
                vpColVector cP,p_;
                point[i].changeFrame(cdMo*oMo,cP);
                point[i].projection(cP,p_);
                pd[i].set_x(p_[0]);
                pd[i].set_y(p_[1]);
                pd[i].set_Z(cP[2]);
            }
        }
        //10、开始更新伺服特征误差
        // cdMc=cdMo*oMo*cMo.inverse();
        for(size_t i=0;i<point.size();i++){
            vpColVector cP,tP;
            point[i].changeFrame(cMo,cP);
            point[i].projection(cP,tP);
            p[i].set_x(tP[0]);
            p[i].set_y(tP[1]);
            p[i].set_Z(cP[2]);
        }
        //11、根据速度是否连续计算控制率得到相机速度    
        if(opt_task_sequencing){
            if(!servo_started){
                if(send_velocitys){
                    servo_started=true;
                }
                t_init_servo=vpTime::measureTimeMs();
            }
            v_c=task.computeControlLaw((vpTime::measureTimeMs()-t_init_servo)/1000.);
        }
        else{
            v_c=task.computeControlLaw();
        }
        //12、开始绘图
        #ifdef VISP_HAVE_DISPLAY
        if(opt_plot){
            plotter->plot(0,iter_plot,task.getError());
            plotter->plot(1,iter_plot,v_c);
            iter_plot++;
        } 
        #endif     
        if(opt_verbose){
            std::cout<<"v_c"<<v_c.t()<<std::endl;
        }    
        double error=task.getError().sumSquare();
        if(error < convergence_threshold_image){
            has_converge=true;
            std::cout<<"Servo task has Converge"<<std::endl;
        }
        if(first_time){
            first_time=false;
        }   
        if(!send_velocitys && opt_plot==1){
            v_c=0;
        }    
        //13、开始将速度发送给内核
        // std::thread SetVelocityThread(&Mh::MhIndustrialSCARA::setVelocity,RobotSCARA,Mh::MhIndustrialRobot::CAMERA_FRAME,v_c);
        // SetVelocityThread.detach();
        RobotSCARA->setVelocity(Mh::MhIndustrialRobot::CAMERA_FRAME,v_c);
        //14、处理鼠标事件
        vpMouseButton::vpMouseButtonType button;
        if(opt_plot){
            if(vpDisplay::getClick(plotter->I,button,false)){
            switch (button)
            {
            case vpMouseButton::button1:
                send_velocitys=!send_velocitys;
                break;
            case vpMouseButton::button3:
                final_quit=true;
                v_c=0;
                break;
            default:
                break;
            }
            }   
        }
        //视觉伺服过程点击结束伺服按钮，会导致伺服的结束
        if(RobotSCARA->ConChargeData.startServo==0){
            final_quit=true;
        }
        #ifdef VISP_HAVE_DISPLAY
        //位姿误差也写在图上
        if(opt_plot){
            std::stringstream ss;
            ss.str("");
            ss<<"error_t:"<<error;
            vpDisplay::displayText(plotter->I,20,20,ss.str(),vpColor::red);
            vpDisplay::flush(plotter->I);
        }
        #endif
    }
    std::cout<<"Stop the robot"<<std::endl;
    RobotSCARA->setRobotState(Mh::MhIndustrialRobot::STATE_STOP); 
    //删除图像
    if(opt_plot && plotter!=nullptr){
        delete plotter;
        plotter=nullptr;
    }
    //处理自然收敛时候后续图片显示的问题
    if(opt_plot){
        if(!final_quit){
        plotter=new vpPlot(2,250*2,500,100,200,"Realtime curves plotter");
        while(!final_quit){
            vpDisplay::displayText(plotter->I,20,20,"click to quit the program.",vpColor::red);
            vpDisplay::displayText(plotter->I,40,20,"Visual servo converged.",vpColor::red);
            if(vpDisplay::getClick(plotter->I,false)){
                final_quit=true;
            }
            vpDisplay::flush(plotter->I);
        }
        delete plotter;
        plotter=nullptr;
        }
    }
    //配合control thread处理相应的控制变量
    RobotSCARA->ConChargeData.startServo=0;//视觉伺服结束
    RobotSCARA->ConChargeData.hasServo=0;
}