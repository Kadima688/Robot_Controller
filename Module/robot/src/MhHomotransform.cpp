#include"MhHomotransform.h"
#include<iostream>
//angle:the unit is deg
//axis:0,1,2:rotate about x,y,z
Eigen::MatrixXd Mh::MhHomotransform::rot2homomatrix(double angle,int axis){
    Eigen::MatrixXd rotation=Eigen::MatrixXd::Identity(4,4);
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
        rotation(2,2)=math.cosd(angle);
    }
    else if(axis==2){
        rotation(0,0)=math.cosd(angle);
        rotation(0,1)=-math.sind(angle);
        rotation(1,0)=math.sind(angle);
        rotation(1,1)=math.cosd(angle);
    }
    return rotation;
}

Eigen::MatrixXd Mh::MhHomotransform::trans2homomatrix(double distance,int axis){
    Eigen::MatrixXd translation=Eigen::MatrixXd::Identity(4,4);
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

std::vector<double> Mh::MhHomotransform::homomatrix2ZYZ(Eigen::MatrixXd &T){
    std::vector<double> cartpos(6);
    cartpos[0]=T(0,3);
    cartpos[1]=T(1,3);
    cartpos[2]=T(2,3);
    cartpos[3]=0;
    cartpos[4]=math.atan2d(sqrt(pow(T(2,0),2)+pow(T(2,1),2)),T(2,2));
    if(fabs(cartpos[4]-0)<1e-8){
        cartpos[3]=0;
        cartpos[5]=math.atan2d(-T(0,1),T(0,0));
        return cartpos;
    }
    else if(fabs(cartpos[4]-180)<1e-8){
        cartpos[3]=0;
        cartpos[5]=math.atan2d(T(0,1),-T(0,0));
        return cartpos;
    }
    else{
        cartpos[3]=math.atan2d(T(1,2)/math.sind(cartpos[4]),T(0,2)/math.sind(cartpos[4]));
        cartpos[5]=math.atan2d(T(2,1)/math.sind(cartpos[4]),-T(2,0)/math.sind(cartpos[4]));
        return cartpos;
    }  
}

Eigen::MatrixXd Mh::MhHomotransform::ZYZ2homomatrix(std::vector<double>& Cartesian){
    return trans2homomatrix(Cartesian[0],0)*trans2homomatrix(Cartesian[1],1)*trans2homomatrix(Cartesian[2],2)*rot2homomatrix(Cartesian[3],2)*rot2homomatrix(Cartesian[4],1)*rot2homomatrix(Cartesian[5],2);
}


