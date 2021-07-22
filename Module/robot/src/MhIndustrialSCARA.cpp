#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"robot-controller/Module/robot/include/MhIndustrialSCARA.h"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include"MhIndustrialSCARA.h"
#endif

Mh::MhIndustrialSCARA::MhIndustrialSCARA()
:math(),transform(),path_plan(),m_q_min(),m_q_max()
{
    setRobotType(MhIndustrialRobot::ROBOT_SCARA);
    init();
}

void Mh::MhIndustrialSCARA::init(){
    nDof=4;

    m_q_min=std::array<double,4>{-127,-145,-20,-360};
    m_q_max=std::array<double,4>{127,145,100,360};
}

Mh::MhIndustrialSCARA::~MhIndustrialSCARA(){
    setRobotType(MhIndustrialRobot::ROBOT_UNKNOWN);
}

void Mh::MhIndustrialSCARA::set_dh_table(MhDH &_dh){
    dh_table.set_link_number(_dh.get_link_number());
    dh_table.set_a(_dh.get_a());
    dh_table.set_alapha(_dh.get_alapha());
    dh_table.set_d(_dh.get_d());
    dh_table.set_theta(_dh.get_theta());
}

void Mh::MhIndustrialSCARA::get_dh_table(){
    for(int i=0;i<dh_table.get_link_number();++i){
        std::cout<<"连杆转角"<<i<<"= "<<dh_table.get_alapha()[i]<<"   ";
        if(i==dh_table.get_link_number()-1){
            std::cout<<std::endl;
        }
    }
    for(int i=0;i<dh_table.get_link_number();++i){
        std::cout<<"连杆长度"<<i<<"= "<<dh_table.get_a()[i]<<"   ";
        if(i==dh_table.get_link_number()-1){
            std::cout<<std::endl;
        }
    }
    for(int i=0;i<dh_table.get_link_number();++i){
        std::cout<<"连杆偏距"<<i<<"= "<<dh_table.get_d()[i]<<"   ";
        if(i==dh_table.get_link_number()-1){
            std::cout<<std::endl;
        }
    }
    for(int i=0;i<dh_table.get_link_number();++i){
        std::cout<<"连杆扭角"<<i<<"= "<<dh_table.get_theta()[i]<<"   ";
        if(i==dh_table.get_link_number()-1){
            std::cout<<std::endl;
        }
    }
}


bool Mh::MhIndustrialSCARA::forwardkinematics(std::vector<double> Axis,std::vector<double>& Cartesian){
    if(Axis.size()!=4){
        std::cout<<"the input axis number is not right"<<std::endl;
        return false;
    }
    else{
        
        Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
        T(0,0)=math.cosd(Axis[0]+Axis[1]-Axis[3]);
        T(0,1)=math.sind(Axis[0]+Axis[1]-Axis[3]);
        T(0,3)=dh_table.get_a()[2]*math.cosd(Axis[0]+Axis[1])+dh_table.get_a()[1]*math.cosd(Axis[0]);
        T(1,0)=math.sind(Axis[0]+Axis[1]-Axis[3]);
        T(1,1)=-math.cosd(Axis[0]+Axis[1]-Axis[3]);
        T(1,3)=dh_table.get_a()[2]*math.sind(Axis[0]+Axis[1])+dh_table.get_a()[1]*math.sind(Axis[0]);
        T(2,2)=-1;
        T(2,3)=dh_table.get_d()[0]-Axis[2];
        Cartesian=transform.homomatrix2ZYZ(T);
        return true;
    }   
}   

bool Mh::MhIndustrialSCARA::inversekinematics(std::vector<double>& Axis,std::vector<double> Cartesian,int type){
    if(fabs(fabs(Cartesian[4])-180)>0.1){
        return false;
    }
    Eigen::Matrix4d T=transform.ZYZ2homomatrix(Cartesian);
    double temp = (pow(T(0, 3), 2) + pow(T(1, 3), 2) - pow(dh_table.get_a()[1], 2) - pow(dh_table.get_a()[2], 2)) / (2 * dh_table.get_a()[1] * dh_table.get_a()[2]);
    double theta2[2];
    if (1 - pow(temp, 2) < 0) {
		return false;
	}
	else
	{
		theta2[0] = math.atan2d(sqrt(1 - pow(temp, 2)), temp);
		double num = 0 - sqrt(1 - pow(temp, 2));
		theta2[1] = math.atan2d(num, temp);
	}
    double theta1[2];
	theta1[0]= math.atan2d(T(1, 3), T(0, 3)) - math.atan2d(dh_table.get_a()[2] * math.sind(theta2[0]), dh_table.get_a()[1] + dh_table.get_a()[2] * math.cosd(theta2[0]));
	theta1[1] = math.atan2d(T(1, 3), T(0, 3)) - math.atan2d(dh_table.get_a()[2] * math.sind(theta2[1]), dh_table.get_a()[1] + dh_table.get_a()[2] * math.cosd(theta2[1]));
    for (int i = 0; i < 2; i++) {
		if (theta1[i] > 0 && theta1[i] > 180) {
			theta1[i] -= 360;
		}
		else if (theta1[i] < 0 && theta1[i] < -180) {
			theta1[i] += 360;
		}
	}
    double theta4[2];
	theta4[0]= theta1[0]+theta2[0]-math.atan2d(T(1, 0), T(0, 0));
	theta4[1] = theta1[1] + theta2[1]-math.atan2d(T(1, 0), T(0, 0));
	for (int i = 0; i < 2; i++) {
		if (theta4[i] > 0 && theta4[i] > 180) {
			theta4[i] -= 360;
		}
		else if (theta4[i] < 0 && theta4[i] < -180) {
			theta4[i] += 360;
		}
	}
    double d3 =dh_table.get_d()[0]-T(2,3);
    //minimum distance to select the ans
    double curDistance[2] = { 0 };
    int index = 0;
    for (int i = 0; i < 2; i++) {
		curDistance[i] = fabs(theta1[i] - Axis[0]) + fabs(theta2[i] - Axis[1]) + fabs(theta4[i] - Axis[3]);
	}
	if (curDistance[0] >=curDistance[1]) {
		index = 1;
	}
	if (type == 1)
	{
		if (curDistance[index] > 10 * path_plan.LINE_PATH_DIFF)
		{
			return false;
		}
	}
    Axis[0]=theta1[index];
    Axis[1]=theta2[index];
    Axis[3]=theta4[index];
    Axis[2]=d3;
    //judge if axis offlimit
    for(int i=0;i<4;++i){
        if(Axis[i]<m_q_min[i]||Axis[i]>m_q_max[i]){
            std::cout<<"Axis"<<i<<"is offlimit"<<std::endl;
            return false;
        }
    }
    return true;
}

void Mh::MhIndustrialSCARA::Jacabian(std::vector<double>Axis,std::vector<double>Cartesian){
    std::cout<<"jacabian"<<std::endl;
}
