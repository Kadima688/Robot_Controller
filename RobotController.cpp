#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"robot-controller/Module/core/include/MhMath.h"
    
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include"Module/core/include/MhMath.h"
    #include"Module/robot/include/MhDh.h"
#endif
#include<iostream>


int main(){
    #if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
        //添加windows平台相关得代码
    #endif 
    #if defined(linux) || defined(_linux) || defined(_linux_)
        //添加linux平台相关的代码
        Mh::DH dh;
        int n=4;
        dh.set_link_number(n);
        std::cout<<dh.get_link_number()<<std::endl;
        return 0;
    #endif
}