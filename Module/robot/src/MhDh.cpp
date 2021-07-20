#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"robot-controller/Module/robot/include/MhDh.h"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include"MhDh.h"
#endif

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