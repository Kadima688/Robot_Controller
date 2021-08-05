#include"MhDh.h"
#include<iostream>
#include<eigen3/Eigen/Dense>

void Mh::MhDH::set_link_number(int n){
    link_number=n;
    alapha.resize(link_number);
    a.resize(link_number);
    theta.resize(link_number);
    d.resize(link_number);
}
void Mh::MhDH::set_alapha(const std::vector<double>& value){
    for(int i=0;i<link_number;++i){
        alapha[i]=value[i];
    }
}
void Mh::MhDH::set_a(const std::vector<double>& value){
    for(int i=0;i<link_number;++i){
        a[i]=value[i];
    }
}
void Mh::MhDH::set_theta(const std::vector<double>& value){
    for(int i=0;i<link_number;++i){
        theta[i]=value[i];
    }
}
void Mh::MhDH::set_d(const std::vector<double>& value){
    for(int i=0;i<link_number;++i){
        d[i]=value[i];
    }
}

void Mh::MhDH::show_dh_table(){
    for(int i=0;i<link_number;++i){
        std::cout<<"连杆转角"<<i<<"= "<<alapha[i]<<"   ";
        if(i==link_number-1){
            std::cout<<std::endl;
        }
    }
    for(int i=0;i<link_number;++i){
        std::cout<<"连杆长度"<<i<<"= "<<a[i]<<"   ";
        if(i==link_number-1){
            std::cout<<std::endl;
        }
    }
    for(int i=0;i<link_number;++i){
        std::cout<<"连杆偏距"<<i<<"= "<<d[i]<<"   ";
        if(i==link_number-1){
            std::cout<<std::endl;
        }
    }
    for(int i=0;i<link_number;++i){
        std::cout<<"连杆扭角"<<i<<"= "<<theta[i]<<"   ";
        if(i==link_number-1){
            std::cout<<std::endl;
        }
    }

}