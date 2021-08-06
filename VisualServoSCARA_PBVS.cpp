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

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && \
  (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI))

void VisualServoSCARA_PBVS(void *scara){
    // Mh::MhIndustrialSCARA* RobotSCARA=static_cast<Mh::MhIndustrialSCARA*>(scara);
}

#endif
