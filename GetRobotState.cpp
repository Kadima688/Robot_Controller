
#include<iostream>
#include<unistd.h>
#include"MhIndustrialSCARA.h"
void GetRobotState(void *scara){
    Mh::MhIndustrialSCARA* RobotSCARA=static_cast<Mh::MhIndustrialSCARA*>(scara);
    std::cout<<"getrobotstate"<<std::endl;
    // while(true){
        
    // }
}