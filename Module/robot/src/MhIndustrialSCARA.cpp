#if defined(WIN32) || defined(_WIN32) || defined(_WIN32) 
    #include"robot-controller/Module/robot/include/MhIndustrialSCARA.h"
#endif 
#if defined(linux) || defined(_linux) || defined(_linux_)
    #include"MhIndustrialSCARA.h"
#endif

Mh::MhIndustrialSCARA::MhIndustrialSCARA()
:m_q_min(),m_q_max()
{
    setRobotType(MhIndustrialRobot::ROBOT_SCARA);
    init();
}

void Mh::MhIndustrialSCARA::init(){
    nDof=4;
    z_lead=20;
    
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

void Mh::MhIndustrialSCARA::set_dh_table(){
    dh_table.set_link_number(nDof);
    dh_table.set_a(a_);
    dh_table.set_alapha(alpha);
    dh_table.set_d(d);
    dh_table.set_theta(offset1);
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

bool Mh::MhIndustrialSCARA::loadRobotConfigFile(const char* xmlpath){
    if(doc.LoadFile(xmlpath)!=0){
        return false;
    }
    rootElem=doc.RootElement();
    robotElem=nullptr;
    for(robotElem=rootElem->FirstChildElement();robotElem;robotElem=robotElem->NextSiblingElement()){
        tinyxml2::XMLElement *baseInfo=robotElem->FirstChildElement();
        if(!strcmp(baseInfo->Attribute("available"),"true")){
            robotNameList.push_back(robotElem->Value());
        }
    }
    if(robotNameList.empty()){
        std::cout<<"no available robot"<<std::endl;
        return false;
    }
    const char* robotName=robotNameList[1];
    loadRobotConfig(robotName);
    doc.SaveFile(xmlpath);
    return true;
}

void Mh::MhIndustrialSCARA::loadRobotConfig(const char* robotName){
        getElementByName(rootElem, robotName, &robotElem);
		tinyxml2::XMLElement* pElem = nullptr;
		getElementByName(robotElem, "alpha", &pElem);
		getAxisAttribute(pElem, alpha);
        getElementByName(robotElem, "a", &pElem);
		getAxisAttribute(pElem, a_);
		getElementByName(robotElem, "d", &pElem);
		getAxisAttribute(pElem, d);
		getElementByName(robotElem, "offset1", &pElem);
		getAxisAttribute(pElem, offset1);
		getElementByName(robotElem, "maxPos", &pElem);
		getAxisAttribute(pElem, maxPos);
		getElementByName(robotElem, "minPos", &pElem);
		getAxisAttribute(pElem, minPos);
		getElementByName(robotElem, "maxVel", &pElem);
		getAxisAttribute(pElem, maxVel);
		getElementByName(robotElem, "maxAcc", &pElem);
		getAxisAttribute(pElem, maxAcc);
		getElementByName(robotElem, "maxDec", &pElem);
		getAxisAttribute(pElem, maxDec);
		getElementByName(robotElem, "maxJerk", &pElem);
		getAxisAttribute(pElem, maxJerk);
		getElementByName(robotElem, "offset2", &pElem);
		getAxisAttribute(pElem, offset2);
		getElementByName(robotElem, "direction", &pElem);
		getAxisAttribute(pElem, direction);
		getElementByName(robotElem, "ratio", &pElem);
		getAxisAttribute(pElem, ratio);
		getElementByName(robotElem, "encoder", &pElem);
		getAxisAttribute(pElem, encoder);
        getElementByName(robotElem, "dynamic", &pElem);
		getMotionParam(pElem, dynamic);
		getElementByName(robotElem, "jogSpeed", &pElem);
		getMotionParam(pElem, jogspeed);
		getElementByName(robotElem, "base", &pElem);
		getCoordinate(base, pElem);
		getElementByName(robotElem, "tool", &pElem);
        getCoordinate(tool, pElem);
        averagePulseEquivalent=0;
        pulseEquivalent.resize(nDof);
        for (int i = 0; i < nDof; ++i)
		    {
			    pulseEquivalent[i] = 360 * direction[i] / ratio[i] / encoder[i];	
			    if (i == 2) {
				    pulseEquivalent[2] = pulseEquivalent[2] * z_lead / 360;//20代表Z轴方向丝杠导程
			    }
			    averagePulseEquivalent += fabs(pulseEquivalent[i]);
		    }
		    averagePulseEquivalent = averagePulseEquivalent / nDof;
        //采用保守方法计算最大合成速度和最大合成加速度
		maxSyntheticVel = maxVel[0];
		maxSyntheticAcc = maxAcc[0];
		maxSyntheticJerk = maxJerk[0];
		for (int i = 1; i < nDof; i++)
		{
			if (maxSyntheticVel > maxVel[i]) maxSyntheticVel = maxVel[i];
			if (maxSyntheticAcc > maxAcc[i]) maxSyntheticAcc = maxAcc[i];
			if (maxSyntheticJerk > maxJerk[i]) maxSyntheticJerk = maxJerk[i];
		}
        //初始化最小关节和最大关节m_q_min和m_q_max
        for(int i=0;i<nDof;++i){
            m_q_max[i]=maxPos[i];
            m_q_min[i]=minPos[i];
        }
}

bool Mh::MhIndustrialSCARA::getElementByName(tinyxml2::XMLElement *rootElem, const char *destElemName, tinyxml2::XMLElement **destElem){
    if(0==strcmp(destElemName,rootElem->Value())){
        *destElem=rootElem;
        return true;
    }
    tinyxml2::XMLElement *plem=nullptr;
    for(plem=rootElem->FirstChildElement();plem;plem=plem->NextSiblingElement()){
        if(0!=strcmp(destElemName,plem->Value())){
            getElementByName(plem,destElemName,destElem);
        }
        else{
            *destElem=plem;
            return true;
        }
    }
    return false;
}
void Mh::MhIndustrialSCARA::getAxisAttribute(tinyxml2::XMLElement *pElem, std::vector<double>& attribute){
    attribute.resize(nDof);
    for(int i=0;i<nDof;++i){
        char axisName[nDof];
        sprintf(axisName,"%s%d", "axis", i + 1);
        attribute[i] = atof(pElem->Attribute(axisName));
    }
}

void Mh::MhIndustrialSCARA::getMotionParam(tinyxml2::XMLElement *pElem, DYNAMIC& dyn){
    dyn.velAxis = atof(pElem->Attribute("velAxis"));
	dyn.accAxis = atof(pElem->Attribute("accAxis"));
	dyn.decAxis = -atof(pElem->Attribute("accAxis"));
	dyn.jerkAxis = atof(pElem->Attribute("jerkAxis"));
	dyn.velPath = atof(pElem->Attribute("velPath"));
	dyn.accPath = atof(pElem->Attribute("accPath"));
	dyn.decPath = -atof(pElem->Attribute("accPath"));
	dyn.jerkPath = atof(pElem->Attribute("jerkPath"));
	dyn.velOri = atof(pElem->Attribute("velOri"));
	dyn.accOri = atof(pElem->Attribute("accOri"));
	dyn.decOri = -atof(pElem->Attribute("accOri"));
	dyn.jerkOri = atof(pElem->Attribute("jerkOri"));
}

void Mh::MhIndustrialSCARA::getCoordinate(CARTSYS cartSys, tinyxml2::XMLElement *pElem){
    cartSys.x = atof(pElem->Attribute("x"));
	cartSys.y = atof(pElem->Attribute("y"));
	cartSys.z = atof(pElem->Attribute("z"));
	cartSys.a = atof(pElem->Attribute("a"));
	cartSys.b = atof(pElem->Attribute("b"));
	cartSys.c = atof(pElem->Attribute("c"));
}
