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
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/gui/vpPlot.h>
#include<visp3/core/vpEigenConversion.h>
#include<thread>

void display_point_trajectory(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &vip,
                              std::vector<vpImagePoint> *traj_vip)
{
  for (size_t i = 0; i < vip.size(); i++) {
    if (traj_vip[i].size()) {
      // Add the point only if distance with the previous > 1 pixel
      if (vpImagePoint::distance(vip[i], traj_vip[i].back()) > 1.) {
        traj_vip[i].push_back(vip[i]);
      }
    }
    else {
      traj_vip[i].push_back(vip[i]);
    }
  }
  for (size_t i = 0; i < vip.size(); i++) {
    for (size_t j = 1; j < traj_vip[i].size(); j++) {
      vpDisplay::displayLine(I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2);
    }
  }
}

void RealsenseServoSCARA_IBVS(Mh::MhIndustrialSCARA *RobotSCARA,double opt_tagSzie,bool adaptive_gain,bool opt_plot,bool opt_task_sequencing,bool opt_verbose,
                          double convergence_threshold_t,double convergence_threshold_tu,double convergence_threshold_image)
{ 
    bool display_tag = true;
    int opt_quad_decimate = 2;
    vpRealSense2 rs;
    rs2::config config;
    unsigned int width = 640, height = 480;
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    config.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    rs.open(config);

    //1、设置相机外参信息
    vpHomogeneousMatrix eMc;
    eMc[0][0] = 0.866; eMc[0][1] = -0.5; eMc[0][2] = 0; eMc[0][3] = 0.02488820463;
	eMc[1][0] = 0.5; eMc[1][1] = 0.866; eMc[1][2] = 0; eMc[1][3] = -0.0364688832;
	eMc[2][0] = 0; eMc[2][1] = 0; eMc[2][2] = 1; eMc[2][3] = 0.001;
	eMc[3][0] = 0; eMc[3][1] = 0; eMc[3][2] = 0; eMc[3][3] = 1;
    Eigen::MatrixXd eTc;
    vp::visp2eigen(eMc,eTc);eTc(0,3)*=1000;eTc(1,3)*=1000;eTc(2,3)*=1000;
    vpCameraParameters cam(612.5037923270, 614.2081420775, 319.6421537812, 241.6648888147, 0.1116749369209, 0.1080431375330);
    std::cout << "cam:\n" << cam << std::endl;

    vpImage<unsigned char> I(height, width);
#if defined(VISP_HAVE_X11)
    vpDisplayX dc(I, 10, 10, "Color image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(I, 10, 10, "Color image");
#endif
    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    //vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
    vpDetectorAprilTag detector(tagFamily);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setDisplayTag(display_tag);
    detector.setAprilTagQuadDecimate(opt_quad_decimate);

    //2、定义cdMc初始化相机和目标物体的位置信息
    vpHomogeneousMatrix cdMc,oMo,cMo;
    vpHomogeneousMatrix cdMo(vpTranslationVector(0,0,opt_tagSzie*3),vpRotationMatrix({1,0,0,0,-1,0,0,0,-1}));
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
        plotter=new vpPlot(2, static_cast<int>(250 * 2), 500, static_cast<int>(I.getWidth()) + 80, 10, "Real time curves plotter");
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

    std::vector<vpImagePoint> *traj_corners = nullptr; // To memorize point trajectory

    static double t_init_servo=vpTime::measureTimeMs();

    RobotSCARA->set_eMc(eMc);//设置机器人的外参矩阵
    RobotSCARA->setRobotState(Mh::MhIndustrialRobot::STATE_VELOCITY_CONTROL);//设置为速度控制模式
    while(!has_converge &&!final_quit){
        double t_start=vpTime::measureTimeMs();
        rs.acquire(I);

        vpDisplay::display(I);
        std::vector<vpHomogeneousMatrix> cMo_vec;
        detector.detect(I, opt_tagSzie, cam, cMo_vec);
        {
            std::stringstream ss;
            ss << "Left click to " << (send_velocitys ? "stop the robot" : "servo the robot") << ", right click to quit.";
            vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
        }

        //7、开始更新cMo=eMc.inverse()*fMe.inverse()*fMo;---模拟detect函数的作用
        vpColVector v_c(6);
        if(cMo_vec.size()==1){
            cMo=cMo_vec[0];
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
            // Get tag corners
            std::vector<vpImagePoint> corners = detector.getPolygon(0);
            //10、开始更新伺服特征误差
            // cdMc=cdMo*oMo*cMo.inverse();
            for(size_t i=0;i<corners.size();i++){
                // Update the point feature from the tag corners location
                vpFeatureBuilder::create(p[i], cam, corners[i]);
                // Set the feature Z coordinate from the pose
                vpColVector cP;
                point[i].changeFrame(cMo, cP);

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
            // Display the current and desired feature points in the image display
            vpServoDisplay::display(task, cam, I);
            for (size_t i = 0; i < corners.size(); i++) {
              std::stringstream ss;
              ss << i;
              // Display current point indexes
              vpDisplay::displayText(I, corners[i]+vpImagePoint(15, 15), ss.str(), vpColor::red);
              // Display desired point indexes
              vpImagePoint ip;
              vpMeterPixelConversion::convertPoint(cam, pd[i].get_x(), pd[i].get_y(), ip);
              vpDisplay::displayText(I, ip+vpImagePoint(15, 15), ss.str(), vpColor::red);
            }
            if (first_time) {
               traj_corners = new std::vector<vpImagePoint> [corners.size()];
            }
            // Display the trajectory of the points used as features
            display_point_trajectory(I, corners, traj_corners);

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
            std::stringstream ss;
            ss << "error: " << error;
            vpDisplay::displayText(I, 20, static_cast<int>(I.getWidth()) - 150, ss.str(), vpColor::red);

            if (opt_verbose){
                std::cout << "error: " << error << std::endl;
            }
          
            if(error < convergence_threshold_image){
                has_converge=true;
                std::cout<<"Servo task has Converge"<<std::endl;
                vpDisplay::displayText(I, 100, 20, "Servo task has converged", vpColor::red);
            }
            if(first_time){
                first_time=false;
            }   
        }
        else{
            v_c=0;
        }
        if(!send_velocitys){
            v_c=0;
        }    
        //13、开始将速度发送给内核
        std::thread SetVelocityThread(&Mh::MhIndustrialSCARA::setVelocity,RobotSCARA,Mh::MhIndustrialRobot::CAMERA_FRAME,v_c);
        SetVelocityThread.detach();
        // RobotSCARA->setVelocity(Mh::MhIndustrialRobot::CAMERA_FRAME,v_c);
        {
            std::stringstream ss;
            ss << "Loop time: " << vpTime::measureTimeMs() - t_start << " ms";
            vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
        }
        vpDisplay::flush(I);

        //14、处理鼠标事件
        vpMouseButton::vpMouseButtonType button;
        if(opt_plot){
            if(vpDisplay::getClick(I,button,false)){
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
    }
    std::cout<<"Stop the robot"<<std::endl;
    RobotSCARA->setRobotState(Mh::MhIndustrialRobot::STATE_STOP); 
    //删除图像
    if(opt_plot && plotter!=nullptr){
        delete plotter;
        plotter=nullptr;
    }
    //处理自然收敛时候后续图片显示的问题
    if (!final_quit) {
        while (!final_quit) {
            rs.acquire(I);
            vpDisplay::display(I);    
            vpDisplay::displayText(I, 20, 20, "Click to quit the program.", vpColor::red);
            vpDisplay::displayText(I, 40, 20, "Visual servo converged.", vpColor::red);   
            if (vpDisplay::getClick(I, false)) {
                final_quit = true;
            } 
            vpDisplay::flush(I);
        }
    }
    if (traj_corners) {
      delete [] traj_corners;
    }
    //配合control thread处理相应的控制变量
    RobotSCARA->ConChargeData.startServo=0;//视觉伺服结束
    RobotSCARA->ConChargeData.hasServo=0;
}