#include"MhConfig.h"
#include<iostream>
#include<math.h>
Mh::MhConfig::MhConfig():
axisnum(6)
{  
    coupling.axis4ToAxis5=0;
    coupling.axis4ToAxis6=0;
    coupling.axis5ToAxis6=0; 
    robotType="General Robot(six axis)";
}

int Mh::MhConfig::createxml(const char* xmlpath){
    const char* declaration ="<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>";
    tinyxml2::XMLDocument doc;
    doc.Parse(declaration);
    return doc.SaveFile(xmlpath);
}

//0:success 1:false
bool Mh::MhConfig::loadRobotConfig(const char* xmlpath){
    if(doc.LoadFile(xmlpath)!=0){
        return false;
    }
    // doc.Print();
    root=doc.RootElement();
    robot=nullptr;
    for(robot=root->FirstChildElement();robot;robot=robot->NextSiblingElement()){
        tinyxml2::XMLElement *baseInfo=robot->FirstChildElement();
        if(!strcmp(baseInfo->Attribute("available"),"true")){
            robotNameList.push_back(robot->Value());
        }
    }
    if(robotNameList.empty()){
        std::cout<<"no available robot"<<std::endl;
        return false;
    }
    const char* robotName=robotNameList[0];
    loadAvailableRoobt(robotName);
    doc.SaveFile(xmlpath);
    return true;
}
void Mh::MhConfig::loadAvailableRoobt(const char* robotname){
        getElementByName(root, robotname, &robot);
		tinyxml2::XMLElement* pElem = nullptr;
		getElementByName(robot, "alpha", &pElem);
		getAxisAttribute(pElem, alpha);
        getElementByName(robot, "a", &pElem);
		getAxisAttribute(pElem, a_);
		getElementByName(robot, "d", &pElem);
		getAxisAttribute(pElem, d);
		getElementByName(robot, "offset1", &pElem);
		getAxisAttribute(pElem, offset1);
		getElementByName(robot, "maxPos", &pElem);
		getAxisAttribute(pElem, maxPos);
		getElementByName(robot, "minPos", &pElem);
		getAxisAttribute(pElem, minPos);
		getElementByName(robot, "maxVel", &pElem);
		getAxisAttribute(pElem, maxVel);
		getElementByName(robot, "maxAcc", &pElem);
		getAxisAttribute(pElem, maxAcc);
		getElementByName(robot, "maxDec", &pElem);
		getAxisAttribute(pElem, maxDec);
		getElementByName(robot, "maxJerk", &pElem);
		getAxisAttribute(pElem, maxJerk);
		getElementByName(robot, "offset2", &pElem);
		getAxisAttribute(pElem, offset2);
		getElementByName(robot, "direction", &pElem);
		getAxisAttribute(pElem, direction);
		getElementByName(robot, "ratio", &pElem);
		getAxisAttribute(pElem, ratio);
		getElementByName(robot, "encoder", &pElem);
		getAxisAttribute(pElem, encoder);
        getElementByName(robot, "dynamic", &pElem);
		getMotionParam(pElem, dynamic);
		getElementByName(robot, "jogSpeed", &pElem);
		getMotionParam(pElem, jogspeed);
		getElementByName(robot, "base", &pElem);
		getCoordinate(base, pElem);
		getElementByName(robot, "tool", &pElem);
        getCoordinate(tool, pElem);
        averagePulseEquivalent=0;
        if(robotType=="General Robot(six axis)"){
            pulseEquivalent.resize(axisnum);
            for (int i = 0; i < 6; ++i)
		    {
			    pulseEquivalent[i] = 360 * direction[i] / ratio[i] / encoder[i];
			    averagePulseEquivalent += fabs(pulseEquivalent[i]);
		    }
		    averagePulseEquivalent = averagePulseEquivalent / 6;
        }
        else if(robotType=="SCARA Robot(four axis)"){
            for (int i = 0; i < 4; ++i)
		    {
			    pulseEquivalent[i] = 360 * direction[i] / ratio[i] / encoder[i];	
			    if (i == 2) {
				    pulseEquivalent[2] = pulseEquivalent[2] * 20 / 360;
			    }
			    averagePulseEquivalent += fabs(pulseEquivalent[i]);
		    }
		    averagePulseEquivalent = averagePulseEquivalent / 4;
        }
        //采用保守方法计算最大合成速度和最大合成加速度
		maxSyntheticVel = maxVel[0];
		maxSyntheticAcc = maxAcc[0];
		maxSyntheticJerk = maxJerk[0];
		for (int i = 1; i < axisnum; i++)
		{
			if (maxSyntheticVel > maxVel[i]) maxSyntheticVel = maxVel[i];
			if (maxSyntheticAcc > maxAcc[i]) maxSyntheticAcc = maxAcc[i];
			if (maxSyntheticJerk > maxJerk[i]) maxSyntheticJerk = maxJerk[i];
		}
}

bool Mh::MhConfig::getElementByName(tinyxml2::XMLElement *rootElem, const char *destElemName, tinyxml2::XMLElement **destElem){
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

void Mh::MhConfig::getAxisAttribute(tinyxml2::XMLElement *pElem, std::vector<double>& attribute){
    attribute.resize(axisnum);
    for(int i=0;i<axisnum;++i){
        char axisName[axisnum];
        sprintf(axisName,"%s%d", "axis", i + 1);
        attribute[i] = atof(pElem->Attribute(axisName));
    }
}

void Mh::MhConfig::getMotionParam(tinyxml2::XMLElement *pElem, DYNAMIC& dyn){
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

void Mh::MhConfig::getCoordinate(CARTSYS cartSys, tinyxml2::XMLElement *pElem){
    cartSys.x = atof(pElem->Attribute("x"));
	cartSys.y = atof(pElem->Attribute("y"));
	cartSys.z = atof(pElem->Attribute("z"));
	cartSys.a = atof(pElem->Attribute("a"));
	cartSys.b = atof(pElem->Attribute("b"));
	cartSys.c = atof(pElem->Attribute("c"));
}