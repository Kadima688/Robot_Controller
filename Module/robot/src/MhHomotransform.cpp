#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"robot-controller/Module/robot/include/MhHomotransform.h"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include"MhHomotransform.h"
#endif

//angle:the unit is deg
//axis:0,1,2:rotate about x,y,z
Eigen::Matrix4d Mh::MhHomotransform::rot2homomatrix(double angle,int axis){
    Eigen::Matrix4d rotation=Eigen::Matrix4d::Identity();
    if(axis==0){
        rotation(1,1)=math.cosd(angle);
        rotation(1,2)=-math.sind(angle);
        rotation(2,1)=math.sind(angle);
        rotation(2,2)=math.cosd(angle);
    }
    else if(axis==1){
        rotation(0,0)=math.cosd(angle);
        rotation(0,2)=math.sind(angle);
        rotation(2,0)=-math.sind(angle);
        rotation(2,1)=math.cosd(angle);
    }
    else if(axis==2){
        rotation(0,0)=math.cosd(angle);
        rotation(0,1)=-math.sind(angle);
        rotation(1,0)=math.sind(angle);
        rotation(1,1)=math.cosd(angle);
    }
    return rotation;
}

Eigen::Matrix4d Mh::MhHomotransform::trans2homomatrix(double distance,int axis){
    Eigen::Matrix4d translation=Eigen::Matrix4d::Identity();
    if(axis==0){
        translation(0,3)=distance;
    }
    else if(axis==1){
        translation(1,3)=distance;
    }
    else if(axis==2){
        translation(2,3)=distance;
    }
    return translation;
}

