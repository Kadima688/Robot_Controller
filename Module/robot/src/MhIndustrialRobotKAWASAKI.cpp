#include"MhIndustrialRobotKAWASAKI.h"

Mh::MhIndustrialKAWASAKI::MhIndustrialKAWASAKI()
:m_q_min(),m_q_max()
{
    setRobotType(MhIndustrialRobot::ROBOT_KAWASAKI);
    init();
}

Mh::MhIndustrialKAWASAKI::~MhIndustrialKAWASAKI(){
    setRobotType(MhIndustrialRobot::ROBOT_UNKNOWN);
}

void Mh::MhIndustrialKAWASAKI::init(){
    nDof=6;
    m_q_max={180,135,155,200,125,360};
    m_q_min={-180,-135,-155,-200,-125,-360};
}

Mh::MhIndustrialRobot::MhRobotStateType Mh::MhIndustrialKAWASAKI::setRobotState(const Mh::MhIndustrialRobot::MhRobotStateType newState){
    Mh::MhIndustrialRobot::setRobotState(newState);
    return newState;
}

void Mh::MhIndustrialKAWASAKI::set_dh_table(MhDH &_dh){
    dh_table.set_link_number(_dh.get_link_number());
    dh_table.set_a(_dh.get_a());
    dh_table.set_alapha(_dh.get_alapha());
    dh_table.set_d(_dh.get_d());
    dh_table.set_theta(_dh.get_theta());
}

void Mh::MhIndustrialKAWASAKI::set_dh_table(){
    dh_table.set_link_number(nDof);
    dh_table.set_a(RobotConfigData.a_);
    dh_table.set_alapha(RobotConfigData.alpha);
    dh_table.set_d(RobotConfigData.d);
    dh_table.set_theta(RobotConfigData.offset1);
}

void Mh::MhIndustrialKAWASAKI::get_dh_table(){
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

bool Mh::MhIndustrialKAWASAKI::loadRobotConfigFile(const char* xmlpath){
    if(RobotConfigData.doc.LoadFile(xmlpath)!=0){
        return false;
    }
    RobotConfigData.rootElem=RobotConfigData.doc.RootElement();
    RobotConfigData.robotElem=nullptr;
    for(RobotConfigData.robotElem=RobotConfigData.rootElem->FirstChildElement();RobotConfigData.robotElem;RobotConfigData.robotElem=RobotConfigData.robotElem->NextSiblingElement()){
        tinyxml2::XMLElement *baseInfo=RobotConfigData.robotElem->FirstChildElement();
        if(!strcmp(baseInfo->Attribute("available"),"true")){
            RobotConfigData.robotNameList.push_back(RobotConfigData.robotElem->Value());
        }
    }
    if(RobotConfigData.robotNameList.empty()){
        std::cout<<"no available robot"<<std::endl;
        return false;
    }
    const char* robotName=RobotConfigData.robotNameList[0];//使用KAWASAKI参数
    loadRobotConfig(robotName);
    RobotConfigData.doc.SaveFile(xmlpath);
    return true;
}

void Mh::MhIndustrialKAWASAKI::loadRobotConfig(const char* robotName){
    getElementByName(RobotConfigData.rootElem, robotName, &RobotConfigData.robotElem);
		tinyxml2::XMLElement* pElem = nullptr;
		getElementByName(RobotConfigData.robotElem, "alpha", &pElem);
		getAxisAttribute(pElem,RobotConfigData.alpha);
        getElementByName(RobotConfigData.robotElem, "a", &pElem);
		getAxisAttribute(pElem, RobotConfigData.a_);
		getElementByName(RobotConfigData.robotElem, "d", &pElem);
		getAxisAttribute(pElem, RobotConfigData.d);
		getElementByName(RobotConfigData.robotElem, "offset1", &pElem);
		getAxisAttribute(pElem, RobotConfigData.offset1);
		getElementByName(RobotConfigData.robotElem, "maxPos", &pElem);
		getAxisAttribute(pElem, RobotConfigData.maxPos);
		getElementByName(RobotConfigData.robotElem, "minPos", &pElem);
		getAxisAttribute(pElem, RobotConfigData.minPos);
		getElementByName(RobotConfigData.robotElem, "maxVel", &pElem);
		getAxisAttribute(pElem, RobotConfigData.maxVel);
		getElementByName(RobotConfigData.robotElem, "maxAcc", &pElem);
		getAxisAttribute(pElem, RobotConfigData.maxAcc);
		getElementByName(RobotConfigData.robotElem, "maxDec", &pElem);
		getAxisAttribute(pElem, RobotConfigData.maxDec);
		getElementByName(RobotConfigData.robotElem, "maxJerk", &pElem);
		getAxisAttribute(pElem, RobotConfigData.maxJerk);
		getElementByName(RobotConfigData.robotElem, "offset2", &pElem);
		getAxisAttribute(pElem, RobotConfigData.offset2);
		getElementByName(RobotConfigData.robotElem, "direction", &pElem);
		getAxisAttribute(pElem, RobotConfigData.direction);
		getElementByName(RobotConfigData.robotElem, "ratio", &pElem);
		getAxisAttribute(pElem, RobotConfigData.ratio);
		getElementByName(RobotConfigData.robotElem, "encoder", &pElem);
		getAxisAttribute(pElem, RobotConfigData.encoder);
        getElementByName(RobotConfigData.robotElem, "coupling", &pElem);
		RobotConfigData.coupling.axis4ToAxis5 = atof(pElem->Attribute("axis4ToAxis5"));
		RobotConfigData.coupling.axis4ToAxis6 = atof(pElem->Attribute("axis4ToAxis6"));
		RobotConfigData.coupling.axis5ToAxis6 = atof(pElem->Attribute("axis5ToAxis6"));
        getElementByName(RobotConfigData.robotElem, "dynamic", &pElem);
		getMotionParam(pElem, RobotConfigData.dynamic);
		getElementByName(RobotConfigData.robotElem, "jogSpeed", &pElem);
		getMotionParam(pElem, RobotConfigData.jogspeed);
		getElementByName(RobotConfigData.robotElem, "base", &pElem);
		getCoordinate(RobotConfigData.base, pElem);
		getElementByName(RobotConfigData.robotElem, "tool", &pElem);
        getCoordinate(RobotConfigData.tool, pElem);
        RobotConfigData.averagePulseEquivalent=0;
        RobotConfigData.pulseEquivalent.resize(nDof);
        //计算各轴的脉冲当量
        for(unsigned int i=0;i<nDof;++i){
            RobotConfigData.pulseEquivalent[i] = 360 * RobotConfigData.direction[i] / RobotConfigData.ratio[i] / RobotConfigData.encoder[i];
            RobotConfigData.averagePulseEquivalent += fabs(RobotConfigData.pulseEquivalent[i]);
        }
        RobotConfigData.averagePulseEquivalent=RobotConfigData.averagePulseEquivalent/nDof;
        //采用保守估计法计算最大合成速度和恶最大合成加速度
        RobotConfigData.maxSyntheticVel=RobotConfigData.maxVel[0];
        RobotConfigData.maxSyntheticAcc=RobotConfigData.maxAcc[0];
        RobotConfigData.maxSyntheticJerk=RobotConfigData.maxJerk[0];
        for(unsigned int i=1;i<nDof;i++){
            if (RobotConfigData.maxSyntheticVel > RobotConfigData.maxVel[i]) RobotConfigData.maxSyntheticVel = RobotConfigData.maxVel[i];
			if (RobotConfigData.maxSyntheticAcc > RobotConfigData.maxAcc[i]) RobotConfigData.maxSyntheticAcc = RobotConfigData.maxAcc[i];
			if (RobotConfigData.maxSyntheticJerk > RobotConfigData.maxJerk[i]) RobotConfigData.maxSyntheticJerk = RobotConfigData.maxJerk[i];
        }
        //初始化最小关节和最大关节m_q_min和m_q_max
        for(unsigned int i=0;i<nDof;++i){
            m_q_max[i]=RobotConfigData.maxPos[i];
            m_q_min[i]=RobotConfigData.minPos[i];
        }
}

bool  Mh::MhIndustrialKAWASAKI::getElementByName(tinyxml2::XMLElement *rootElem, const char *destElemName, tinyxml2::XMLElement **destElem){
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

void Mh::MhIndustrialKAWASAKI::getAxisAttribute(tinyxml2::XMLElement *pElem, std::vector<double>& attribute){
    attribute.resize(nDof);
    for(unsigned int i=0;i<nDof;++i){
        char axisName[nDof];
        sprintf(axisName,"%s%d", "axis", i + 1);
        attribute[i] = atof(pElem->Attribute(axisName));
    }
}

void Mh::MhIndustrialKAWASAKI::getMotionParam(tinyxml2::XMLElement *pElem, DYNAMIC& dyn){
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

void Mh::MhIndustrialKAWASAKI::getCoordinate(CARTSYS cartSys, tinyxml2::XMLElement *pElem){
    cartSys.x = atof(pElem->Attribute("x"));
	cartSys.y = atof(pElem->Attribute("y"));
	cartSys.z = atof(pElem->Attribute("z"));
	cartSys.a = atof(pElem->Attribute("a"));
	cartSys.b = atof(pElem->Attribute("b"));
	cartSys.c = atof(pElem->Attribute("c"));
}
    




