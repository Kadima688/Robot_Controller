#include"MhIndustrialSCARA.h"
#include <visp3/robot/vpRobotException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include<visp3/core/vpEigenConversion.h>
#include<visp3/core/vpExponentialMap.h>
#include"os.h"
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

void Mh::MhIndustrialSCARA::get_fJc(vpMatrix& fJc){
    vpMatrix fJc_temp(4,4);
    fJc_temp[0][0] = m_eMc[1][3] * math.cosd(axisPos_scara.a1 + axisPos_scara.a2 - axisPos_scara.a4) - m_eMc[0][3] * math.sind(axisPos_scara.a1 + axisPos_scara.a2 - axisPos_scara.a4) - (a_[2] / 1000) * math.sind(axisPos_scara.a1 + axisPos_scara.a2) - (a_[1] / 1000) * math.sind(axisPos_scara.a1);
    fJc_temp[0][1] = m_eMc[1][3] * math.cosd(axisPos_scara.a1 + axisPos_scara.a2 - axisPos_scara.a4) - m_eMc[0][3] * math.sind(axisPos_scara.a1 + axisPos_scara.a2 - axisPos_scara.a4) - (a_[2] / 1000) * math.sind(axisPos_scara.a1 + axisPos_scara.a2);
    fJc_temp[0][2] = 0;
    fJc_temp[0][3] = m_eMc[0][3] * math.sind(axisPos_scara.a1 + axisPos_scara.a2 - axisPos_scara.a4) - m_eMc[1][3] * math.cosd(axisPos_scara.a1 + axisPos_scara.a2 - axisPos_scara.a4);
	fJc_temp[1][0] = m_eMc[0][3] * math.cosd(axisPos_scara.a1 + axisPos_scara.a2 - axisPos_scara.a4) + m_eMc[1][3] * math.sind(axisPos_scara.a1 + axisPos_scara.a2 - axisPos_scara.a4) + (a_[2] / 1000) * math.cosd(axisPos_scara.a1 + axisPos_scara.a2) + (a_[1] / 1000) * math.cosd(axisPos_scara.a1);
	fJc_temp[1][1] = m_eMc[0][3] * math.cosd(axisPos_scara.a1 + axisPos_scara.a2 - axisPos_scara.a4) + m_eMc[1][3] * math.sind(axisPos_scara.a1 + axisPos_scara.a2 - axisPos_scara.a4) + (a_[2] / 1000) * math.cosd(axisPos_scara.a1 + axisPos_scara.a2);
	fJc_temp[1][2] = 0;
	fJc_temp[1][3] = -m_eMc[0][3] * math.cosd(axisPos_scara.a1 + axisPos_scara.a2 - axisPos_scara.a4) - m_eMc[1][3] * math.sind(axisPos_scara.a1 + axisPos_scara.a2 - axisPos_scara.a4);
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
    }
    }
    //----------相机雅可比方案，考虑姿态
    vpMatrix fJc;
    get_fJc(fJc);
    fJc=fJc.inverseByLUEigen3();//camera&&fixed frame的逆矩阵
    vpColVector v_f(6);
    v_f=get_velocityMatrix(frame,true)*v_c;//获取相机速度相对fixed frame的表达
    vpColVector v_c_real(4);
    v_c_real[0]=v_f[0];v_c_real[1]=v_f[1];v_c_real[2]=v_f[2];v_c_real[3]=v_f[5];
    vpColVector qdot=fJc*v_c_real;
    //如果接近奇异点，则停止运动
    if(math.sind(axisPos_scara.a2)==0){
        for(int i=0;i<qdot.size();++i){
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
    std::vector<double> Cartesian(6);Cartesian[0]=cartPos.x;Cartesian[1]=cartPos.y;Cartesian[2]=cartPos.z;Cartesian[3]=cartPos.a;Cartesian[4]=cartPos.b;Cartesian[5]=cartPos.c;
    Eigen::MatrixXd fTed=transform.ZYZ2homomatrix(Cartesian)*eTed;
    std::vector<double> des_Cartesian=transform.homomatrix2ZYZ(fTed);//预期到达的空间位置
    std::vector<double> axis_pos(4);axis_pos[0]=axisPos_scara.a1;axis_pos[1]=axisPos_scara.a2;axis_pos[2]=axisPos_scara.d;axis_pos[3]=axisPos_scara.a4;//当前关节位姿
    if(inversekinematics(axis_pos,des_Cartesian,0)){
        //逆解，此时axis_pos为将要到达的关节位置
        for(int i=0;i<nDof;++i){
            if(axis_pos[i]<minPos[i]+10||axis_pos[i]>maxPos[i]-10){
                EC_TRACE("第%d个轴将要接近限位点!",i);
                for(int i=0;i<qdot.size();++i){
                    qdot[i]=0;
                }
                break;
            }
        }
        setJointVelocity(qdot);
    }

}

vpMatrix Mh::MhIndustrialSCARA::get_velocityMatrix(const MhIndustrialRobot::MhControlFrameType frame,bool fixed){
    Eigen::MatrixXd  T=Eigen::MatrixXd::Identity(4,4);
    double theta[4]={axisPos_scara.a1,axisPos_scara.a2,axisPos_scara.d,axisPos_scara.a4};
    switch (frame)
    {
    case MhIndustrialRobot::TOOL_FRAME:{
        if(fixed==0){
            //求解camera相对固定世界坐标系的速度转换矩阵
            for(int i=0;i<4;++i){
                if(i==2){
                    T = T * transform.rot2homomatrix(alpha[i], 0) * transform.trans2homomatrix(a_[i], 0) * transform.trans2homomatrix(d[i] + theta[i], 2) * transform.rot2homomatrix(offset1[i], 2);
                }
                else{
                    T = T * transform.rot2homomatrix(alpha[i], 0) * transform.trans2homomatrix(a_[i], 0) * transform.trans2homomatrix(d[i], 2) * transform.rot2homomatrix((theta[i] + offset1[i]), 2);
                }
            }
            //获取旋转矩阵
            Eigen::MatrixXd rotation_matrix=T.block<3,3>(0,0);
            Eigen::MatrixXd zero_matrix=Eigen::MatrixXd::Zero(3,3);
            Eigen::MatrixXd VelocityTwistMatrix(6,6);
            VelocityTwistMatrix<<rotation_matrix,zero_matrix,zero_matrix,rotation_matrix;
            vpMatrix TwistMatrix;
            vp::eigen2visp(VelocityTwistMatrix,TwistMatrix);
            return TwistMatrix;
        }
        break;
    }
    case MhIndustrialRobot::END_EFFECTOR_FRAME:
    case MhIndustrialRobot::REFERENCE_FRAME:{

        break;
    }
    }
}

void Mh::MhIndustrialSCARA::setJointVelocity(const vpColVector &qdot){

}
