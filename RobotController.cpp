#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"robot-controller/Module/core/include/MhMath.h"
    #include"robot-controller/Module/core/include/MhDh.h"
    
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include"Module/core/include/MhMath.h"
    #include"Module/robot/include/MhDh.h"
    #include"Module/robot/include/MhHomotransform.h"
#endif
#include<iostream>


int main(){
    #if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
        //添加windows平台相关得代码
    #endif 
    #if defined(linux) || defined(_linux) || defined(_linux_)
        //添加linux平台相关的代码
 
        return 0;
    #endif
}