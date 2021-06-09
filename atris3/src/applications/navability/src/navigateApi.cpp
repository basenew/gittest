/******************************************************************************

  Copyright (C), 2001-2011,UBT.CO.

 ******************************************************************************
 File Name     : navigateApi.cpp
Version       : Initial Draft
Author        : marty.gong@ubtrobot.com
Created       : 2020/2/25
Last Modified :
Description   : navigateApi.cpp

 ******************************************************************************/

/*----------------------------------------------*
 * include files                           *
 *----------------------------------------------*/
//#include "ros/ros.h"
#include "navigateApi.h"
#include <json/json.h>
#include <string>
#include <algorithm>


/*----------------------------------------------*
 * external variables                           *
 *----------------------------------------------*/

/*----------------------------------------------*
 * external routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * internal routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * project-wide global variables                *
 *----------------------------------------------*/

/*----------------------------------------------*
 * module-wide global variables                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * constants                                    *
 *----------------------------------------------*/

/*----------------------------------------------*
 * macros                                       *
 *----------------------------------------------*/

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nva_api]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nva_api]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nva_api]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nva_api]"#format, ##args)

NavigateApi::NavigateApi()
{

}

NavigateApi::~NavigateApi()
{

}

int NavigateApi::init()
{
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	Navi_.Init();
#endif
	return 0;
}

#if 0
int NavigateApi::startMapping(const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.StartMapping(request, rep);
#endif
	return ret;
}

int  NavigateApi::pauseMapping(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.PauseMapping(request,rep);
#endif
	return ret;
}

int NavigateApi::resumeMapping(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.ResumeMapping(request,rep);
#endif
	return ret;

}

int NavigateApi::stopMapping(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.StopMapping(request,rep);
#endif
	return ret;
}
#endif

int NavigateApi::getMapLists(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.GetMapLists(request,rep);
#endif
	return ret;
}

int NavigateApi::setMap(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.SetMap(request,rep);
#endif
	return ret;
}

int NavigateApi::cancelMap(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.CancelMap(request,rep);
#endif   
	return ret;
}

int NavigateApi::renameMap(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.RenameMap(request,rep);
#endif
	return ret;
}
int NavigateApi::delMap(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.DelMap(request,rep);
#endif
	return ret;
}

int NavigateApi::delAllMap(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.DelAllMap(request,rep);
#endif     
	return ret;
}

int NavigateApi::upLoadMap(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.UploadMap(request,rep);
#endif
	return ret;
}

int NavigateApi::downloadMap(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.DownloadMap(request,rep);
#endif
	return ret;
}

int NavigateApi::getTransferMapStatus(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.GetTransferMapStatus(request,rep);
#endif     
	return ret;
}

int NavigateApi::getMapStatus(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.getMapStatus(request,rep);
#endif
	return ret;
}

int NavigateApi::getUsingMap(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.getUsingMap(request,rep);
#endif
	return ret;
}

//relocate   modified by marty.gong@ubtrobot.com:2020-2-28-10:4:46 

int NavigateApi::startRelocating(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.StartRelocate(request,rep);
#endif 
	return ret;
}

int NavigateApi::pauseReLocating(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.PauseReLocate(request,rep);
#endif
	return ret;
}

int NavigateApi::resumeReLocating(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.ResumeReLocate(request,rep);
#endif
	return ret;
}

int NavigateApi::stopReLocating(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.StopReLocate(request,rep);
#endif
	return ret;
}

int NavigateApi::getRelocatingStatus(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.GetRelocateStatus(request,rep);
#endif
	return ret;
}

int NavigateApi::getRelocatingResult(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.GetRelocateResult(request,rep);
#endif
	return ret;
}

int NavigateApi::getRobotPosition(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.GetRobotPosition(request,rep);
#endif
	return ret;
}

//navigate  modified by marty.gong@ubtrobot.com:2020-2-28-10:6:2 

int NavigateApi::startNavigation(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.StartNavigate(request,rep);
#endif
	return ret;
}

int NavigateApi::pauseNavigating(IN const string request, OUT string &rep){
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.PauseNavigate(request,rep);
#endif
	return ret;
}

int NavigateApi::resumeNavigating(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.ResumeNavigate(request,rep);
#endif    
	return ret;
}

int NavigateApi::stopNavigating(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.StopNavigate(request,rep);
#endif   
	return ret;
}

int NavigateApi::getNavistatus(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.GetNavistatus(request,rep);
#endif    
	return ret;
}

int NavigateApi::getTspNavigating(IN const string request, OUT string &rep)
{
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.GetTspNavigate(request,rep);
#endif    
	return ret;
}

int NavigateApi::queryRoute(IN const string request, OUT string &rep)
{
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.QueryRoute(request,rep);
#endif    
	return ret;
}

int  NavigateApi::getTspNavigatingStatus(IN const string request, OUT string &rep)
{
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.GetTspNavigateStatus(request,rep);
#endif    
	return ret;
}

//speed   modified by marty.gong@ubtrobot.com:2020-2-28-10:7:17 

int NavigateApi::getSpeed(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.GetSpeed(request,rep);
#endif      
	return ret;
}

int NavigateApi::setSpeed(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.SetSpeed(request,rep);
#endif      
	return ret;
}

int NavigateApi::applySite(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.ApplySite(request,rep);
#endif  
	return ret;
}

int NavigateApi::getVersion(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.GetVersion(request,rep);
#endif
	return ret;
}

int NavigateApi::getPointLists(IN const string request, OUT string &rep) {
	int ret = 0;
#if (_XINGSHEN_SDK_ == _NAVIGATION_USED_)
	ret = Navi_.GetPointLists(request,rep);
#endif
	return ret;
}


/*************************private*****************************/   


//type 0 string 1:json   modified by marty.gong@ubtrobot.com:2020-5-09-19:39:48 

string NavigateApi::writeJsonWithNormPara(const string title,const int numPara,int type,const string data)
{
		int64_t now = (uint64_t)(ros::Time::now().toSec() * 1000);
		std::stringstream id; id << now;

		Json::FastWriter fw;
		Json::Value strJson,contentJson;

		contentJson["id"]=id.str();
		contentJson["timestamp"]=now;


		if(numPara != 0)
		{
				if(0 == type)
				{
						contentJson["data"]= data;
				}
				else
				{
						Json::Reader reader;
						Json::Value jsonData;
						if(!reader.parse(data,jsonData)) return "";
						contentJson["data"] = jsonData;
				}
		}
		strJson["content"]=contentJson;
		strJson["title"]=title;


		return fw.write(strJson);
}

string NavigateApi::writeJsonWithRenameMapPara(const string title,const string origin_name, const string new_name)
{
		int64_t now = (uint64_t)(ros::Time::now().toSec() * 1000);
		std::stringstream id; id << now;

		Json::FastWriter fw;
		Json::Value strJson,contentJson;

		contentJson["id"]=id.str();
		contentJson["timestamp"]=now;

		contentJson["origin_name"]=origin_name;
		contentJson["new_name"]=new_name;

		strJson["content"]=contentJson;
		strJson["title"]=title;

		return fw.write(strJson);

}

string NavigateApi::writeJsonWithSpeedPara(const string title,const int mode,const int numPara, const string data)
{
		int64_t now = (uint64_t)(ros::Time::now().toSec() * 1000);
		std::stringstream id; id << now;

		Json::FastWriter fw;
		Json::Reader reader;
		Json::Value strJson,contentJson;

		contentJson["id"]=id.str();
		contentJson["timestamp"]=now;
		contentJson["mode"]= mode;
		if(0 != numPara)
		{
				Json::Value jsonData;
				if(!reader.parse(data,jsonData)) return "";
				contentJson["data"]= jsonData;
		}

		strJson["content"]=contentJson;
		strJson["title"]=title;


		return fw.write(strJson);
}

bool NavigateApi::parseResponse(string resp,naviRsp &parseRsp)
{
		Json::Reader reader;
		Json::Value val;

		//从字符串中读取数据
		if(reader.parse(resp,val))
		{
				//data 的值是字符串
				parseRsp.title = (val["title"].isNull())?(""):(val["title"].asString());
				parseRsp.id = (val["content"]["id"].isNull())?(""):(val["content"]["id"].asString());
				parseRsp.timestamp =(val["content"]["timestamp"].isNull())? 0:(val["content"]["timestamp"].asLargestInt());
				parseRsp.msg = (val["content"]["msg"].isNull())?(""): (val["content"]["msg"].asString());
				parseRsp.code = (val["content"]["code"].isNull())?(-1):(val["content"]["code"].asInt());
				parseRsp.result = (val["content"]["result"].isNull())?(""):(val["content"]["result"].asString());
				parseRsp.data = (val["content"]["data"].isNull())?(""):(val["content"]["data"].asString());

		}
		else
		{
				return false;
		}

		return true;  
}



void NavigateApi::EraseCharactorFromString(string &str_,char char_)
{
		string::size_type pos_;
		while(true)
		{

				pos_ = str_.find(char_);
				if(str_.npos == pos_ )
				{
						return ;
				}
				if('\\' == char_ && str_.length() > pos_+1)
				{
						if(str_[pos_+1] == 'n')///n
						{
								str_ = str_.erase(pos_,2);

						}
						else
						{
								str_ = str_.erase(pos_,1);
						}
				}
				str_ = str_.erase(pos_,1);
		}
}
