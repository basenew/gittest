#ifndef _NAVIGATEAPI_H_H
#define _NAVIGATEAPI_H_H


#include <string>
#include <iostream>
#include <stdio.h>
#include <json/json.h>
#include "xs/xingshenApi.h"

using namespace std;

#define IN  
#define OUT 

#define _XINGSHEN_SDK_	(0)
#define _UBT_NAVI_		(1)
#define _GX_NAVI_		(2)


#if defined NAVIGATE_UBT_
	#define _NAVIGATION_USED_	_UBT_NAVI_
#elif defined NAVI_GX_
	#define _NAVIGATION_USED_	_GX_NAVI_
#else 
	#define _NAVIGATION_USED_	_XINGSHEN_SDK_
#endif



typedef enum{
     normal_mode = 0,
     speed_mode ,   
}jsonMode;

typedef struct{
    string title;
    string  id;
    long timestamp;
    string data;
    string msg;
    int code;
    string result;
}naviRsp;

class NavigateApi
{
    public:

     NavigateApi();
     ~NavigateApi();
     int init();
    //map   modified by marty.gong@ubtrobot.com:2020-2-25-15:8:58 
#if 0
     int startMapping(IN const string request, OUT string &rep);
     int pauseMapping(IN const string request, OUT string &rep);
     int resumeMapping(IN const string request, OUT string &rep);
     int stopMapping(IN const string request, OUT string &rep);
#endif
     int getMapLists(IN const string request, OUT string &rep);
     int setMap(IN const string request, OUT string &rep);
     int downloadMap(IN const string request, OUT string &rep);
     int upLoadMap(IN const string request, OUT string &rep);
     int cancelMap(IN const string request, OUT string &rep);
     int renameMap(IN const string request, OUT string &rep);
     int delMap(IN const string request, OUT string &rep);
     int delAllMap(IN const string request, OUT string &rep);
     int getMapStatus(IN const string request, OUT string &rep);
     int getTransferMapStatus(IN const string request, OUT string &rep);
     int getUsingMap(IN const string request, OUT string &rsp);

     int getPointLists(IN const string request, OUT string &rep);
     
    //relocate   modified by marty.gong@ubtrobot.com:2020-2-25-15:9:10 
     int startRelocating(IN const string request, OUT string &rep);
     int pauseReLocating(IN const string request, OUT string &rep);
     int resumeReLocating(IN const string request, OUT string &rep);
     int stopReLocating(IN const string request, OUT string &rep);
     int getRelocatingStatus(IN const string request, OUT string &rep);
     int getRobotPosition(IN const string request, OUT string &rep);
    //navigate   modified by marty.gong@ubtrobot.com:2020-2-25-15:9:16 
     int startNavigation(IN const string request, OUT string &rep);
     int pauseNavigating(IN const string request, OUT string &rep);
     int resumeNavigating(IN const string request, OUT string &rep);
     int stopNavigating(IN const string request, OUT string &rep);
     int getNavistatus(IN const string request, OUT string &rep);
     int getTspNavigating(IN const string request, OUT string &rep);
     int queryRoute(IN const string request, OUT string &rep);
     int getTspNavigatingStatus(IN const string request, OUT string &rep);
     int getRelocatingResult(IN const string request, OUT string &rep);

    //speed   modified by marty.gong@ubtrobot.com:2020-2-26-8:52:47 
     int getSpeed(IN const string request, OUT string &rep);
     int setSpeed(IN const string request, OUT string &rep);
     int getVersion(IN const string request, OUT string &rep);
     int applySite(IN const string request, OUT string &rep);
     
    private:

     string writeJsonWithNormPara(const string title,const int numPara,int type,const string data);
     string writeJsonWithSpeedPara(const string title,const int mode,const int numPara, const string data);
     string writeJsonWithRenameMapPara(const string title,const string origin_name, const string new_name);
     bool parseResponse(string resp,naviRsp &parseRsp);
     void EraseCharactorFromString(string &str_,char char_);
    #if( _NAVIGATION_USED_ == _XINGSHEN_SDK_)
     NaviMove Navi_;
    #endif

};

#endif

