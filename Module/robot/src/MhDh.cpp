#include"MhDh.h"


void Mh::DH::set_link_number(int n){
    link_number=n;
    alapha.resize(link_number);
    a.resize(link_number);
    theta.resize(link_number);
    d.resize(link_number);
}
void Mh::DH::set_alapha(const std::vector<double>& value){
    for(int i=0;i<link_number;++i){
        alapha[i]=value[i];
    }
}
void Mh::DH::set_a(const std::vector<double>& value){
    for(int i=0;i<link_number;++i){
        a[i]=value[i];
    }
}
void Mh::DH::set_theta(const std::vector<double>& value){
    for(int i=0;i<link_number;++i){
        theta[i]=value[i];
    }
}
void Mh::DH::set_d(const std::vector<double>& value){
    for(int i=0;i<link_number;++i){
        d[i]=value[i];
    }
}