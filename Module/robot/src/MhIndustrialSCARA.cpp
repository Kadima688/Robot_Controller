#include"MhIndustrialSCARA.h"
#include <visp3/robot/vpRobotException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include<visp3/core/vpEigenConversion.h>
#include<visp3/core/vpExponentialMap.h>
#include"os.h"
#include<unistd.h>
Mh::MhIndustrialSCARA::MhIndustrialSCARA()
:m_q_min(),m_q_max()
{
    setRobotType(MhIndustrialRobot::ROBOT_SCARA);
    init();
}

void Mh::MhIndustrialSCARA::init(){
    nDof=4;
    z_lead=20;
    a4_Compensation=35;
    m_q_max={127,145,100,360};
    m_q_min={-127,-145,0,-360};
}

Mh::MhIndustrialSCARA::~MhIndustrialSCARA(){
    setRobotType(MhIndustrialRobot::ROBOT_UNKNOWN);
}

Mh::MhIndustrialRobot::MhRobotStateType Mh::MhIndustrialSCARA::setRobotState(const Mh::MhIndustrialRobot::MhRobotStateType newState){
    int retn;
    switch(newState){
    case Mh::MhIndustrialRobot::STATE_STOP:
        if(Mh::MhIndustrialRobot::STATE_VELOCITY_CONTROL==Mh::MhIndustrialRobot::getRobotState()){
            std::cout<<"Stop robot from velocity control"<<std::endl;
            for(unsigned int i=0;i<nDof;++i){
                //首先速度设置为0
                retn=SetVelCommand(i,0);
                set_retn(retn,SETVELCOMMAND);
                sleep(0.1);
                //转换为位置控制模式
                retn=SetAxisCommandMode(i,0);
                set_retn(retn,SETAXISCOMMANDMODE);
            }
        }
        Mh::MhIndustrialRobot::setRobotState(newState);
    break;
    case MhIndustrialRobot::STATE_VELOCITY_CONTROL:
        if(Mh::MhIndustrialRobot::STATE_STOP==Mh::MhIndustrialRobot::getRobotState()){
            std::cout<<"Start Velocity Control robot from stop"<<std::endl;
            //首先退出插补缓冲
            RobotCloseConti();
            //停止所有轴运动
            retn=StopAllAxis(1);
            set_retn(retn,STOPALLAXIS);
            for(unsigned int i=0;i<nDof;++i){
                //从位置控制模式转换为速度控制模式
                retn=SetAxisCommandMode(i,1);
                set_retn(retn,SETAXISCOMMANDMODE);  
            }
        }
        Mh::MhIndustrialRobot::setRobotState(newState);
    break;
    }
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
    dh_table.set_a(RobotConfigData.a_);
    dh_table.set_alapha(RobotConfigData.alpha);
    dh_table.set_d(RobotConfigData.d);
    dh_table.set_theta(RobotConfigData.offset1);
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
        Eigen::MatrixXd T=Eigen::MatrixXd::Identity(4,4);
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
    Eigen::MatrixXd T=transform.ZYZ2homomatrix(Cartesian);
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
		if (curDistance[index] > 10 * path_plan.get_LINE_PATH_DIFF())
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

void Mh::MhIndustrialSCARA::get_fJc(vpMatrix& fJc){
    vpMatrix fJc_temp(4,4);
    fJc_temp[0][0] = m_eMc[1][3] * math.cosd(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2 - Con2DemData.axisPos_scara.a4) - m_eMc[0][3] * math.sind(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2 - Con2DemData.axisPos_scara.a4) - (RobotConfigData.a_[2] / 1000) * math.sind(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2) - (RobotConfigData.a_[1] / 1000) * math.sind(Con2DemData.axisPos_scara.a1);
    fJc_temp[0][1] = m_eMc[1][3] * math.cosd(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2 - Con2DemData.axisPos_scara.a4) - m_eMc[0][3] * math.sind(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2 - Con2DemData.axisPos_scara.a4) - (RobotConfigData.a_[2] / 1000) * math.sind(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2);
    fJc_temp[0][2] = 0;
    fJc_temp[0][3] = m_eMc[0][3] * math.sind(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2 - Con2DemData.axisPos_scara.a4) - m_eMc[1][3] * math.cosd(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2 - Con2DemData.axisPos_scara.a4);
	fJc_temp[1][0] = m_eMc[0][3] * math.cosd(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2 - Con2DemData.axisPos_scara.a4) + m_eMc[1][3] * math.sind(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2 - Con2DemData.axisPos_scara.a4) + (RobotConfigData.a_[2] / 1000) * math.cosd(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2) + (RobotConfigData.a_[1] / 1000) * math.cosd(Con2DemData.axisPos_scara.a1);
	fJc_temp[1][1] = m_eMc[0][3] * math.cosd(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2 - Con2DemData.axisPos_scara.a4) + m_eMc[1][3] * math.sind(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2 - Con2DemData.axisPos_scara.a4) + (RobotConfigData.a_[2] / 1000) * math.cosd(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2);
	fJc_temp[1][2] = 0;
	fJc_temp[1][3] = -m_eMc[0][3] * math.cosd(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2 - Con2DemData.axisPos_scara.a4) - m_eMc[1][3] * math.sind(Con2DemData.axisPos_scara.a1 + Con2DemData.axisPos_scara.a2 - Con2DemData.axisPos_scara.a4);
	fJc_temp[2][0] = 0;
	fJc_temp[2][1] = 0;
	fJc_temp[2][2] = -z_lead / 1000 / (2 * PI); 
	fJc_temp[2][3] = 0;
	fJc_temp[3][0] = 1;
	fJc_temp[3][1] = 1;
	fJc_temp[3][2] = 0;
	fJc_temp[3][3] = -1;
	fJc = fJc_temp;
}

void Mh::MhIndustrialSCARA::get_cJc(vpMatrix& cJc){
    std::cout<<"get cJc"<<std::endl;
}

void Mh::MhIndustrialSCARA::get_fJe(vpMatrix& fJe){
    std::cout<<"get fJe"<<std::endl;
}

void Mh::MhIndustrialSCARA::get_eJe(vpMatrix& eJe){
    std::cout<<"get eJe"<<std::endl;
}

bool Mh::MhIndustrialSCARA::loadRobotConfigFile(const char* xmlpath){
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
    const char* robotName=RobotConfigData.robotNameList[1];//默认用SCARA机器人的配置
    loadRobotConfig(robotName);
    RobotConfigData.doc.SaveFile(xmlpath);
    return true;
}

void Mh::MhIndustrialSCARA::loadRobotConfig(const char* robotName){
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
        for (unsigned int i = 0; i < nDof; ++i)
		    {
			    RobotConfigData.pulseEquivalent[i] = 360 * RobotConfigData.direction[i] / RobotConfigData.ratio[i] / RobotConfigData.encoder[i];	
			    if (i == 2) {
				    RobotConfigData.pulseEquivalent[2] = RobotConfigData.pulseEquivalent[2] * z_lead / 360;//20代表Z轴方向丝杠导程
			    }
			    RobotConfigData.averagePulseEquivalent += fabs(RobotConfigData.pulseEquivalent[i]);
		    }
		    RobotConfigData.averagePulseEquivalent = RobotConfigData.averagePulseEquivalent / nDof;
        //采用保守方法计算最大合成速度和最大合成加速度
		RobotConfigData.maxSyntheticVel = RobotConfigData.maxVel[0];
		RobotConfigData.maxSyntheticAcc = RobotConfigData.maxAcc[0];
		RobotConfigData.maxSyntheticJerk = RobotConfigData.maxJerk[0];
		for (unsigned int i = 1; i < nDof; i++)
		{
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
    for(unsigned int i=0;i<nDof;++i){
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

bool Mh::MhIndustrialSCARA::setVelocity(const MhIndustrialRobot::MhControlFrameType frame,const vpColVector& vel){
    if(MhIndustrialRobot::STATE_VELOCITY_CONTROL!=getRobotState()){
        EC_TRACE("can not send velocity,the robotstate is not right");
        return false;
    }
    vpColVector vel_sat(6);
    switch (frame)
    {
    case MhIndustrialRobot::TOOL_FRAME:
    case MhIndustrialRobot::REFERENCE_FRAME:
    case MhIndustrialRobot::END_EFFECTOR_FRAME:{
        //设置空间速度
        if(vel.size()!=6){
            EC_TRACE("can not apply a Cartesian velocity that is not a 6-dim vector(%d)",vel.size());
            return false;
        }
        vpColVector vel_max(6);
        for(unsigned int i=0;i<3;++i){
            vel_max[i]=maxTranslationVelocity;
        }
        for(unsigned int i=3;i<6;++i){
            vel_max[i]=maxRotationVelocity;
        }
        if(!MhIndustrialRobot::saturateVelocities(vel,vel_max,vel_sat,true)){
            return false;
        }
        setCartVelocity(frame,vel_sat);
        break;
    }
    case MhIndustrialRobot::JOINT_STATE:{
        //设置关节速度
        if(vel.size()!=nDof){
            EC_TRACE("can not apply a Joint velocity that is not a %d-dim vector(%d)",nDof,vel.size());
            return false;
        }
        vpColVector vel_max(vel.size());
        vel_max=maxRotationVelocity;
        if(!MhIndustrialRobot::saturateVelocities(vel,vel_max,vel_sat,true)){
            return false;
        }
        setJointVelocity(vel_sat);
    }
    }
    return true;
}

void Mh::MhIndustrialSCARA::setCartVelocity(const MhIndustrialRobot::MhControlFrameType frame,const vpColVector &vel){
    if(vel.size()!=6){
        EC_TRACE("can not send a velocity twist vector in tool frame that is not 6-dim(%d)",vel.size());
    }
    vpColVector v_c;
    switch (frame)
    {
    case MhIndustrialRobot::TOOL_FRAME:{
        v_c=vel;
        break;
    }
    case MhIndustrialRobot::END_EFFECTOR_FRAME:
    case MhIndustrialRobot::REFERENCE_FRAME:{

        break;
    case MhIndustrialRobot::JOINT_STATE:

        break;
    }
    }
    //----------相机雅可比方案，考虑姿态
    vpMatrix fJc;
    get_fJc(fJc);
    fJc=fJc.inverseByLUEigen3();//camera&&fixed frame的逆矩阵
    vpColVector v_f(6);
    v_f=get_velocityMatrix(frame,false)*v_c;//获取相机速度相对fixed frame的表达
    vpColVector v_c_real(4);
    v_c_real[0]=v_f[0];v_c_real[1]=v_f[1];v_c_real[2]=v_f[2];v_c_real[3]=v_f[5];
    vpColVector qdot=fJc*v_c_real;
    //如果接近奇异点，则停止运动
    if(math.sind(Con2DemData.axisPos_scara.a2)==0){
        for(unsigned int i=0;i<qdot.size();++i){
            qdot[i]=0;
        }
    }
    //如果接近限位点，则停止运动
    double delta_t = 0.1;
	vpHomogeneousMatrix eMed = vpExponentialMap::direct(v_c, delta_t);
	Eigen::Matrix4d eTed;
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			eTed(i, j) = eMed[i][j];
		}
	}
	eTed(0, 3) *= 1000;
	eTed(1, 3) *= 1000;
	eTed(2, 3) *= 1000;
    std::vector<double> Cartesian;Cartesian.clear();
    Cartesian.push_back(Con2DemData.cartPos.x);Cartesian.push_back(Con2DemData.cartPos.y);Cartesian.push_back(Con2DemData.cartPos.z);
    Cartesian.push_back(Con2DemData.cartPos.a);Cartesian.push_back(Con2DemData.cartPos.b);Cartesian.push_back(Con2DemData.cartPos.c);
    Eigen::MatrixXd fTed=transform.ZYZ2homomatrix(Cartesian)*eTed;
    std::vector<double> des_Cartesian(6);
    des_Cartesian=transform.homomatrix2ZYZ(fTed);//预期到达的空间位置
    std::vector<double> axis_pos;axis_pos.clear();
    axis_pos.push_back(Con2DemData.axisPos_scara.a1);axis_pos.push_back(Con2DemData.axisPos_scara.a2);
    axis_pos.push_back(Con2DemData.axisPos_scara.d);axis_pos.push_back(Con2DemData.axisPos_scara.a4);//当前关节位姿
    if(inversekinematics(axis_pos,des_Cartesian,0)){
        //逆解，此时axis_pos为将要到达的关节位置
        for(unsigned int i=0;i<nDof;++i){
            if(axis_pos[i]<RobotConfigData.minPos[i]+10||axis_pos[i]>RobotConfigData.maxPos[i]-10){
                std::cout<<"第"<<i<<"个轴将要接近限位点"<<std::endl;
                for(unsigned int i=0;i<qdot.size();++i){
                    qdot[i]=0;
                }
                break;
            }
        }
        #ifdef USE_KERNEL
            setJointVelocity(qdot);
        #else
            setJointVelocity_virtual(qdot);
        #endif
    }
}

vpMatrix Mh::MhIndustrialSCARA::get_velocityMatrix(const MhIndustrialRobot::MhControlFrameType frame,bool fixed){
    Eigen::MatrixXd  T=Eigen::MatrixXd::Identity(4,4);
    vpMatrix TwistMatrix;TwistMatrix.eye(4,4);
    double theta[4]={Con2DemData.axisPos_scara.a1,Con2DemData.axisPos_scara.a2,Con2DemData.axisPos_scara.d,Con2DemData.axisPos_scara.a4};
    switch (frame)
    {
    case MhIndustrialRobot::TOOL_FRAME:{
        if(fixed==0){
            //求解camera相对固定世界坐标系的速度转换矩阵
            for(int i=0;i<4;++i){
                if(i==2){
                    T = T * transform.rot2homomatrix(RobotConfigData.alpha[i], 0) * transform.trans2homomatrix(RobotConfigData.a_[i], 0) * transform.trans2homomatrix(RobotConfigData.d[i] + theta[i], 2) * transform.rot2homomatrix(RobotConfigData.offset1[i], 2);
                }
                else{
                    T = T * transform.rot2homomatrix(RobotConfigData.alpha[i], 0) * transform.trans2homomatrix(RobotConfigData.a_[i], 0) * transform.trans2homomatrix(RobotConfigData.d[i], 2) * transform.rot2homomatrix((theta[i] + RobotConfigData.offset1[i]), 2);
                }
            }
            //获取旋转矩阵
            Eigen::MatrixXd rotation_matrix=T.block<3,3>(0,0);
            Eigen::MatrixXd zero_matrix=Eigen::MatrixXd::Zero(3,3);
            Eigen::MatrixXd VelocityTwistMatrix(6,6);
            VelocityTwistMatrix<<rotation_matrix,zero_matrix,zero_matrix,rotation_matrix;
            vp::eigen2visp(VelocityTwistMatrix,TwistMatrix);
            return TwistMatrix;
        }
        break;
    }
    case MhIndustrialRobot::END_EFFECTOR_FRAME:
    case MhIndustrialRobot::REFERENCE_FRAME:{
        break;
    }
    case MhIndustrialRobot::JOINT_STATE:{
        break;
    }
    }
}

void Mh::MhIndustrialSCARA::setJointVelocity(const vpColVector &qdot){
    for(unsigned i=0;i<nDof;++i){
        int velocity2pulse;
        if(i!=2){
            velocity2pulse = (int)((qdot[i] * RobotConfigData.direction[i] * RobotConfigData.ratio[i] * RobotConfigData.encoder[i]) / (2 * PI));
        }
        else{
            velocity2pulse = (int)((qdot[i] * RobotConfigData.direction[i] * RobotConfigData.ratio[i] * RobotConfigData.encoder[i]) / (2 * PI));
			//velocity2pulse += (int)((qdot[3] * 180 / PI) / 360 *a4_Compensation) / RobotConfigData.pulseEquivalent[2];
        }
        int retn=SetVelCommand(i,velocity2pulse);
        set_retn(retn,SETVELCOMMAND);
    }
}

void Mh::MhIndustrialSCARA::setJointVelocity_virtual(const vpColVector &qdot){
    Con2DemData.axisPos_scara.a1+=qdot[0];
    Con2DemData.axisPos_scara.a2+=qdot[1];
    Con2DemData.axisPos_scara.d+=qdot[2];
    Con2DemData.axisPos_scara.a4+=qdot[3];
    std::vector<double> cartesian;
    std::vector<double> scara_input={Con2DemData.axisPos_scara.a1,Con2DemData.axisPos_scara.a2,Con2DemData.axisPos_scara.d,Con2DemData.axisPos_scara.a4};        
    if(forwardkinematics(scara_input,cartesian)){
        Con2DemData.cartPos=cartesian;
    }
    sleep(0.02);
}

void Mh::MhIndustrialSCARA::RobotMotorInitial(){
    for(unsigned int i=0;i<nDof;++i){
        int temp1,temp2;
        if(i!=2){
            temp1=(RobotConfigData.maxPos[i]+RobotConfigData.offset2[i])/RobotConfigData.pulseEquivalent[i];
            temp2=(RobotConfigData.minPos[i]+RobotConfigData.offset2[i])/RobotConfigData.pulseEquivalent[i];
        }
        else{
            temp1=(RobotConfigData.maxPos[i]+RobotConfigData.offset2[i]+RobotConfigData.maxPos[3]/360*a4_Compensation)/RobotConfigData.pulseEquivalent[i];
            temp2=(RobotConfigData.minPos[i]+RobotConfigData.offset2[i]+RobotConfigData.minPos[3]/360*a4_Compensation)/RobotConfigData.pulseEquivalent[i];
        }
        int limp,limn;
        if(temp1>temp2){
            limp=temp1;
            limn=temp2;
        }
        else{
            limp=temp2;
            limn=temp1;
        }
        int retn=SetAxisSoftLimit(i,0,limp,limn);
        set_retn(retn,SETAXISSOFTLIMIT);
        retn=EnableSoftLimit(i,1);
        set_retn(retn,ENABLESOFTLIMIT);
        retn=SetAxisPositionMode(i,1);
        set_retn(retn,SETAXISPOSITONMODE);
    }  
}
void Mh::MhIndustrialSCARA::RobotOpenConti(){
    unsigned int ulAxisList[nDof]={0,1,2,3};
    unsigned int ulMaxAcc[nDof];
    for(unsigned int i=0;i<nDof;++i){
        ulMaxAcc[i]=fabs(RobotConfigData.maxAcc[i]/RobotConfigData.pulseEquivalent[i]/pow(1000,2));
        int retn=ContiOpenList(0,nDof,ulAxisList,ulMaxAcc);
        set_retn(retn,CONTIOPENLIST);
        retn=ContiSetLookheadMode(0,1,100,50);
        set_retn(retn,CONTISETLOOKHEADMODE);
    }
}
void Mh::MhIndustrialSCARA::RobotDynInitial(){
    if(ConChargeData.selectDyn==0 || ConChargeData.selectDyn==1){//手动点位或者手动轨迹
        ConChargeData.curDynamic=RobotConfigData.jogspeed;
    }
    else if(ConChargeData.selectDyn==2 || ConChargeData.selectDyn==3){//自动点位或者自动轨迹
        ConChargeData.curDynamic=RobotConfigData.dynamic;
    }
    double targetVel,acc,dec,jerk;
    for(unsigned int i=0;i<nDof;++i){
        //设置轴的速度
        targetVel=RobotConfigData.maxVel[i]*ConChargeData.curDynamic.velAxis/100*Dem2ConData.ovr/100/RobotConfigData.pulseEquivalent[i]/1000;
        int retn =SetAxisVel(i,0,fabs(targetVel),0);
        set_retn(retn,SETAXISVEL);
        //设置轴的加速度
        acc=RobotConfigData.maxAcc[i]*ConChargeData.curDynamic.accAxis/100*Dem2ConData.ovr/100/RobotConfigData.pulseEquivalent[i]/pow(1000,2);
        dec=RobotConfigData.maxDec[i]*ConChargeData.curDynamic.decAxis/100*Dem2ConData.ovr/100/RobotConfigData.pulseEquivalent[i]/pow(1000,2);
        retn=SetAxisAcc(i,fabs(acc),fabs(dec));
        set_retn(retn,SETAXISACC);
        //设置轴的加加速度
        jerk=RobotConfigData.maxJerk[i]*ConChargeData.curDynamic.jerkAxis/100*Dem2ConData.ovr/100/RobotConfigData.pulseEquivalent[i]/pow(1000,3);
        SetAxisJerk(i,fabs(jerk),fabs(jerk));
        set_retn(retn,SETAXISJERK);
    }
}

void Mh::MhIndustrialSCARA::RobotInterpolationDynInitial(){
    if(ConChargeData.selectDyn==0 || ConChargeData.selectDyn==1){//手动点位或者手动轨迹
        ConChargeData.curDynamic=RobotConfigData.jogspeed;
    }
    else if(ConChargeData.selectDyn==2 || ConChargeData.selectDyn==3){//自动点位或者自动轨迹
        ConChargeData.curDynamic=RobotConfigData.dynamic;
    }
    double targetVel,acc,dec,jerk;
    if(ConChargeData.selectDyn==0 || ConChargeData.selectDyn==2){//手动点位&&自动点位运动
        //设置轴的插补速度
        targetVel=RobotConfigData.maxSyntheticVel*ConChargeData.curDynamic.velAxis/100*Dem2ConData.ovr/100/RobotConfigData.averagePulseEquivalent/1000;
        acc=RobotConfigData.maxSyntheticAcc*ConChargeData.curDynamic.accAxis/100*Dem2ConData.ovr/100/RobotConfigData.averagePulseEquivalent/pow(1000,2);
        dec=RobotConfigData.maxSyntheticAcc*ConChargeData.curDynamic.decAxis/100*Dem2ConData.ovr/100/RobotConfigData.averagePulseEquivalent/pow(1000,2);
        jerk=RobotConfigData.maxSyntheticJerk*ConChargeData.curDynamic.jerkAxis/100*Dem2ConData.ovr/100/RobotConfigData.averagePulseEquivalent/pow(1000,3);
    }
    else if(ConChargeData.selectDyn==1 || ConChargeData.selectDyn==3){
        targetVel=ConChargeData.curDynamic.velPath*Dem2ConData.ovr/100/RobotConfigData.averagePulseEquivalent/1000;
        acc=ConChargeData.curDynamic.accPath*Dem2ConData.ovr/100/RobotConfigData.averagePulseEquivalent/pow(1000,2);
        dec=ConChargeData.curDynamic.decPath*Dem2ConData.ovr/100/RobotConfigData.averagePulseEquivalent/pow(1000,2);
        jerk=ConChargeData.curDynamic.jerkPath*Dem2ConData.ovr/100/RobotConfigData.averagePulseEquivalent/pow(1000,3);
    }
    //开始设置插补段连续速度
    int retn=SetInterpolationVel(0,fabs(targetVel),0);
    set_retn(retn,SETINTERPOLATIONVEL);
    //设置插补段加速度和减速度
    retn=SetInterpolationAcc(fabs(acc),fabs(dec));
    set_retn(retn,SETINTERPOLATIONACC);
    //设置插补段加加速度和减加速度
    retn=SetInterpolationJerk(fabs(jerk),fabs(jerk));
    set_retn(retn,SETINTERPOLATIONJERK);
}

void Mh::MhIndustrialSCARA::RobotCloseConti(){
    //停止插补运动
    int retn=ContiStopList(0,1);
    set_retn(retn,CONTISTOPLIST);
    //关闭插补缓冲区
    retn=ContiCloseList(0);
    set_retn(retn,CONTICLOSELIST);
}
void Mh::MhIndustrialSCARA::FollowPathMove(std::map<int,std::vector<double>>& record,int PathType){
    switch (PathType)
    {
    int retn;
    case PTP:
        unsigned int ulAxisList[4]={0,1,2,3};
        int lTargetPos[4]={(record[0][0]+RobotConfigData.offset2[0])/RobotConfigData.pulseEquivalent[0],\
                             (record[0][1]+RobotConfigData.offset2[1])/RobotConfigData.pulseEquivalent[1],\
                             (record[0][2]+RobotConfigData.offset2[2]/*-record[0][3]/360*a4_Compensation*/)/RobotConfigData.pulseEquivalent[2],
                             record[0][3]+RobotConfigData.offset2[3]/RobotConfigData.pulseEquivalent[3]};
        //设置单轴运动
        for(unsigned int i=0;i<nDof;++i){
            retn=PositionDrive(i,lTargetPos[i]);
            set_retn(retn,POSITIONDRIVE);
        }
        //插补运动得指令
        // retn=ContiLineUnit(0,nDof,ulAxisList,lTargetPos,1,0);
        // set_retn(retn,CONTILINEUNIT);
        // retn=ConttiStartList(0);
        // set_retn(retn,CONTISTARTLIST);
        break;
    };
}
void Mh::MhIndustrialSCARA::set_retn(int r,int KERNEL_TYPE){
    std::lock_guard<std::mutex> lock(RobotConChargeDataMutex.retn_mutex);
    switch (KERNEL_TYPE)
    {
    case OPENDEVICE:
        if(r!=0){std::cout<<"打开设备失败,并即将退出程序！"<<std::endl;exit(1);}
        break;
    case OPENMCKERNEL:
        if(r!=0){std::cout<<"初始化内核失败！"<<std::endl;}
        break;
    case SETAXISSOFTLIMIT:
        if(r!=0){std::cout<<"设置软限位失败！"<<std::endl;}
        break;
    case ENABLESOFTLIMIT:
        if(r!=0){std::cout<<"使能软限位失败！"<<std::endl;}
        break;
    case SETAXISPOSITONMODE:
        if(r!=0){std::cout<<"设置轴的位移模式失败"<<std::endl;}
        break;
    case SETAXISVEL:
        if(r!=0){std::cout<<"设置轴的速度失败！"<<std::endl;}
        break;
    case SETAXISACC:
        if(r!=0){std::cout<<"设置轴的加速度失败！"<<std::endl;}
        break;
    case SETAXISJERK:
        if(r!=0){std::cout<<"设置轴的加加速度失败！"<<std::endl;}
        break;
    case SETINTERPOLATIONVEL:
        if(r!=0){std::cout<<"设置插补速度失败！"<<std::endl;}
        break;  
    case SETINTERPOLATIONACC:
        if(r!=0){std::cout<<"设置插补加速度失败！"<<std::endl;}
        break;
    case SETINTERPOLATIONJERK:
        if(r!=0){std::cout<<"设置插补加加速度失败！"<<std::endl;}
        break;
    case SETVELCOMMAND:
        if(r!=0){std::cout<<"发送速度失败！"<<std::endl;}
        break;
    case GETDRIVERSTATE:
        if(r!=0){std::cout<<"获取从站状态失败！"<<std::endl;}
        break;
    case GETAXISPOSITION:
        if(r!=0){std::cout<<"获取轴的逻辑位置失败!"<<std::endl;}
        break;
    case GETDRIVERPOS:
        if(r!=0){std::cout<<"获取从站位置失败！"<<std::endl;}
        break;
    case CONTIOPENLIST:
        if(r!=0){std::cout<<"打开插补缓冲区去失败!"<<std::endl;}
        break;
    case CONTISETLOOKHEADMODE:
        if(r!=0){std::cout<<"设置速度前瞻失败!"<<std::endl;}
        break;
    case CONTILINEUNIT:
        if(r!=0){std::cout<<"直线插补指令调用！"<<std::endl;}
        break;
    case CONTISTARTLIST:
        if(r!=0){std::cout<<"开启连续插补失败！"<<std::endl;}
        break;
    case POSITIONDRIVE:
        if(r!=0){std::cout<<"单轴定长运动指令调用失败！"<<std::endl;}
        break;
    case SETAXISCOMMANDMODE:
        if(r!=0){std::cout<<"设置轴得控制模式失败!"<<std::endl;}
        break;
    case CONTISTOPLIST:
        if(r!=0){std::cout<<"停止插补运动失败！"<<std::endl;}
        break;
    case STOPALLAXIS:
        if(r!=0){std::cout<<"停止所有轴运动失败！"<<std::endl;}
        break;
    case CLOSEDEVICE:
        if(r!=0){std::cout<<"关闭设备失败！"<<std::endl;}
        break;
    }
    ConChargeData.retn=r;
}


