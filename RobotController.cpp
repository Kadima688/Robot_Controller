#include<iostream>
#include"Module/core/MhMath.h"

int main(){
    #if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
        //添加windows平台相关得代码
    #endif 
    #if defined(linux) || defined(_linux) || defined(_linux_)
        //添加linux平台相关的代码
    #endif
    
}