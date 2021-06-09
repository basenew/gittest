/******************************************************************************

  Copyright (C), 2001-2011,UBT.CO.

 ******************************************************************************
  File Name     : gs_upgrade.cpp
  Version       : Initial Draft
  Author        : marty.gong@ubtrobot.com
  Created       : 2018/7/26
  Last Modified :
  Description   : 高仙升级

******************************************************************************/

/*----------------------------------------------*
 * include files                           *
 *----------------------------------------------*/
#include "gs_upgrade.h"
#include <json/json.h>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include "tiny_ros/ros.h"
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
 #define GS_FILE_CFG_PAHT "/home/atris/atris_app/firmware/gsnav/version_info.yaml"
 #define GS_FILE_IMAGE_PATH "/home/atris/atris_app/firmware/gsnav/"
 #define THREE_MINITE 3*60
 #define GS_MAX_PRIT_SIZE 512
 #define GS_REBOOT_TIMEOUT 3*60 
 #define GS_CONNECT_TIMEOUT 30
 #define GS_DEFAULT_IP "192.168.8.4"

//最新的yaml-cpp 0.5取消了运算符">>"，但是还是会有好多的旧代码  
//依旧在使用，所以重载下">>"运算符  
template<typename T>  
void operator >> (const YAML::Node& node, T& i)  
{  
  i = node.as<T>();  
}

//gs-robot/system/update_system_form
/**/


static size_t httpRecvDataCB(void *data, size_t size, size_t nmemb,void *userdata)
{

    HttpResCod *r = (HttpResCod*)userdata;
    if(NULL == r || NULL == data)
    {
        return -1;
    }
    (r->body).append((char*)data, size*nmemb);
    /*
    char str[512];
    snprintf(str, sizeof(str), "body:%s",r->body.c_str());
    log_info(str);
    */
    return (size * nmemb);


}

static size_t httpRecvDataHeaderCB(void *data, size_t size, size_t nmemb,void *userdata)
{

    HttpResCod* r = (HttpResCod*)userdata;
    if(NULL == r || NULL == data)
    {
        return -1;
    }
    r->header.append((char*)data, size*nmemb);
    /*
    char str[512];
    snprintf(str, sizeof(str), "header:%s",r->header.c_str());
    log_info(str);
    */
    return (size * nmemb);

}
static int on_debug_file_(CURL *, curl_infotype itype, char * pData, size_t size, void *)
{
    if(itype == CURLINFO_TEXT)
        log_debug("[TEXT]\n%s", pData);
    else if(itype == CURLINFO_HEADER_IN)
    	log_debug("[HEADER_IN]\n%s", pData);
    else if(itype == CURLINFO_HEADER_OUT)
        log_debug("[HEADER_OUT]%s", pData);
    else if(itype == CURLINFO_DATA_IN)
        log_debug("[DATA_IN]\n%s", pData);
    else if(itype == CURLINFO_DATA_OUT)
        log_debug("[DATA_OUT]\n%02x", *pData);

    return 0;
}

GSupgrade::GSupgrade()
{
}

GSupgrade::~GSupgrade()
{

}

bool GSupgrade::init()
{

    char ch[256]={0};
    cfg = Config::get_instance();
    if(NULL == cfg)
    {
        log_error("get gaussian gs_web_port false.");
        return false;
    }
    snprintf(ch, sizeof(ch), "%s:%d",cfg->gs_ip.c_str(),cfg->gs_web_port);
    gs_web_port_cfg = ch;

    snprintf(ch, sizeof(ch), ":%d",cfg->gs_web_port);
    gs_web_port_bkup = ch; 
    gs_web_port_bkup = GS_DEFAULT_IP+gs_web_port_bkup;


    gs_web_port = gs_web_port_cfg;//实际使用的ip地址初始化为配置ip地址。
    is_systemp_ip_changed = false;
  
    snprintf(ch, sizeof(ch), "Gaussian web_port:%s,cfg:%s,backup:%s",gs_web_port.c_str(),gs_web_port_cfg.c_str(),gs_web_port_bkup.c_str());
    log_info("gaussian upgrade init ok: %s", ch);
    return true;
    
}

bool GSupgrade::getVersion(string & ver_)
{
    string ver_now="";
    GSCfg gs_cfg;

    if(false == verGet(ver_now,gs_cfg))
    {
        log_error("get ver false");
        return false;
    }
    
    char ch[GS_MAX_PRIT_SIZE]={0};
    char needUp = (ver_now ==gs_cfg.ver_cfg)?'N':'Y';
    sprintf(ch,"version_now:%s,version_cfg:%s,needUp:%c",ver_now.c_str(),gs_cfg.ver_cfg.c_str(),needUp);
    ver_ = ch;
    return true;
}


bool GSupgrade::upgrade()
{
    int nmax_count = 3;
    int ncount = 0;
    bool ret_flag = false;

    //升级前要适配好ip地址
   if(false == gsConnectIpSelect())
    {
        log_error("gs:errorfor ip select false.");
        return false;
    }  

    while(ncount++ < nmax_count)
    {
        if(true == tryUpgrade())
        {
            ret_flag = true;
            break;
        }
        log_warn("try upgrade again.");
        sleep(5);
    }
    
    if(true == is_systemp_ip_changed)
    {
        log_info("check need recovery system ip.");

        if(false == gsSetSystemIp("10.20.18.2","10.20.18.1"))
        {
            log_error("recover system ip  failed.");
            return false;
        }
        is_systemp_ip_changed = false;
        gs_web_port = gs_web_port_cfg;
        
    }
    
    if(false == ret_flag)
    {
        log_warn("upgrade still false after 3 times trying.");
    }
    
    return ret_flag;
}
bool GSupgrade::tryUpgrade()
{
    log_info("start upgrade gaussian=====>start.");
    bool isNeedUp = true;
    string ver_now="";
    GSCfg gs_cfg;
    if(false == verCompare(ver_now,gs_cfg,isNeedUp))
    {
        return false;        
    }
    //check is needup   modified by marty.gong@ubtrobot.com:2018-8-02-21:11:22 
    if(false == isNeedUp) return true;
    if(false == startUpgrade(gs_cfg.bin_path))
    {
        return false;
    }
    if(false == rebootGSsys())
    {
        log_error("reboot GSsystem error.");
        return false;
    }
    
    if(false == checkGSDisconnect())
    {
        log_error("check disconncet error.");
        return false;
    }
   
    // 解决高仙私自自启动导致逻辑错误的bug。   modified by marty.gong@ubtrobot.com:2019-12-23-15:18:51 
    sleep(20);
    
    if(gs_web_port != gs_web_port_cfg)
    {
        if(false == gsSetSystemIp("10.20.18.2","10.20.18.1"))
        {
            log_error("change system ip to cfg ip faild.");
            return false;
        }
        is_systemp_ip_changed = false;
        gs_web_port = gs_web_port_cfg;
    }
    
    if(false == versionCheck(gs_cfg.ver_cfg))
    {
        log_error("version check error after upgrading.");
        return false;
    }
    log_info("upgrade gaussian ========>successful.");
    return true;
}


bool GSupgrade::rebootGSsys()
{
    //log_info("reboot the GS systme.please waite===>");
    HttpResCod rep;
    HttpRsp parseRsp;

    if(false == getHttpRequest(&rep,gs_web_port+"/gs-robot/cmd/reboot"))
    {
        log_error("GSupgrade::rebootGSsys  disconnect."); 
        return false;
    }

    if(false == parseHttpResponse(&rep,&parseRsp))
    {
        log_error("GSupgrade::rebootGSsys parse error.");
        return false;
    }
    if(false == parseRsp.successed)
    {
        log_error("GSupgrade::rebootGSsys reboot error.");
        return false;
    }
    
    log_info("reboot GSsystem OK,after upgrading.");
    return true; 
}

bool GSupgrade::checkGSDisconnect()
{
    int timeout=0;

    while(true)
    {
        log_warn("waite for disconnet.");
        if(false  == checkGSconnect())
        {
            return true;
        }
        sleep(1);
        if(++timeout > GS_CONNECT_TIMEOUT)
        {
            log_error("check disconnect timeout.");
            return false;
        }
    }
    return true;
}


bool GSupgrade::gsSetSystemIp(string ip_addr_,string defaut_gw)
{
    string ch = "";
    ch = "ifconfig eth0 " + ip_addr_ + " netmask 255.255.255.0 ; route add default gw " +defaut_gw ;

    log_info(ch.c_str());

    FILE *fp = NULL;
    if((fp = popen(ch.c_str(), "r"))) 
    {
        pclose(fp);
        sleep(5);
        log_info("set systemp ip success.");
        return true;
    }  
    log_error("set systemp ip failed: cannot open the pipe");
    return false;
}
bool GSupgrade::gsConnectIpSelect()
{
   string ch = "";
   gs_web_port = gs_web_port_cfg;

  // log_info(cfg->gs_ip.c_str());
   
    for(int i = 0; i < 40; i++)
    {
        if(true == checkGSconnect())
        {
            log_info("select gs_cfg_ip_port");
            return true;
        }
        sleep(3);
    }

    gs_web_port = gs_web_port_bkup;

    //set system ip 192.168.8.2

    if(false == gsSetSystemIp("192.168.8.2","192.168.8.1"))
    {
        log_error("set sysip failed.");
        return false;
    }
    is_systemp_ip_changed = true;//need recovery.
    
    for(int i = 0; i < 60; i++)
    {
        if(true == checkGSconnect())
        {
            log_info("select gs_bkup_ip_port");
            return true;
        }
        sleep(1);
    }

    //recovery system ip 
   
   if(false == gsSetSystemIp("10.20.18.2","10.20.18.1")) 
   {
        log_error("recover system ip failed.");        
   }
    is_systemp_ip_changed = false;
    log_error("all the ips are not suitable.");
    return false;
}
bool GSupgrade::checkGSconnect()
{
    
    HttpResCod rep;
    HttpRsp parseRsp;

    if(false == getHttpRequest(&rep,gs_web_port+"/gs-robot/cmd/ping"))
    {
        log_error("GSupgrade::checkGSconnect  disconnect."); 
        return false;
    }

    if(false == parseHttpResponse(&rep,&parseRsp))
    {
        log_error("GSupgrade::checkGSconnect parse error.");
        return false;
    }
    if(false == parseRsp.successed)
    {
        log_error("GSupgrade::checkGSconnect error.");
        return false;
    }
    return true;
}
bool GSupgrade::versionCheck( string &ver_cfg)
{
    int time_count_ = 0;
    string ver_now_="";
    while(true)
    {
        
        if(true == verRunRead(ver_now_)) break;
        if(++time_count_ > GS_REBOOT_TIMEOUT)
        {
            log_warn(" GSupgrade::versionCheck waite timeout.");
            return false;
        }
        sleep(1);
    }

    if(ver_now_ != ver_cfg)
    {
        log_warn("the version after upgrading:%s,and cfg:%s",ver_now_.c_str(),ver_cfg.c_str());
        return false;
    }
    return true;

    
}

bool GSupgrade::verGet(string &ver_now,GSCfg &cfg)
{
     if(false == verRunRead(ver_now))
    {
        log_error("read the running software version error.");
        return false;
    }

    if(false == verCfgRead(cfg))
    {
        log_error("read the cfg software version error.");
        return false;
    }
    return true;
}

bool GSupgrade::verCompare(string &ver_now,GSCfg &gs_cfg,bool &isNeedUp)
{
    
    char ch[GS_MAX_PRIT_SIZE]={0};
    if(false == verGet(ver_now,gs_cfg))
    {
        log_warn("get gaussian version false.");
        return false;
    }

    if(gs_cfg.is_force)
    {
        snprintf(ch, sizeof(ch), "ver_now is %s,cfg version is :%s,is_forced:true,bin_path:%s",ver_now.c_str(),gs_cfg.ver_cfg.c_str(),gs_cfg.bin_path.c_str());
    }
    else
    {
        snprintf(ch, sizeof(ch), "ver_now is %s,cfg version is :%s,is_forced:false,bin_path:%s",ver_now.c_str(),gs_cfg.ver_cfg.c_str(),gs_cfg.bin_path.c_str());
    }
    log_info(ch);
    
    if((ver_now == gs_cfg.ver_cfg)&&(false == gs_cfg.is_force))
    {
        log_warn("the software need not upgrade.");
        isNeedUp = false;
        return true;
    }
    return true;
}


bool GSupgrade::startUpgrade(string& bin_path)
{
    HttpResCod rep;
    HttpRsp parseRsp;
    //string filedata;
    char ch[256]={0};
    /*
    if(false == readUpgradeFile(filedata,bin_path))
    {
        log_error("read the image file false.please check the image file path ====> ");
        log_error(bin_path);
        log_error("<===");
        return false;
    }
   */
    if(false == postFileStreamHttpRequest(&rep,gs_web_port+"/gs-robot/system/update_system_form",bin_path))
    {
        log_error("transform binfile error.check the network.");
        return false;
    }

          
    if(false == parseHttpResponse(&rep, &parseRsp) )
    {
         log_error("transform image response parse error. ");
         return false;
    }

    if(true != parseRsp.successed)
    {
        snprintf(ch, sizeof(ch), "tranfor binfile failed. %s",parseRsp.msg.c_str());
        log_error(ch);
        return false;
    }

    log_error("transform binfile successful.and wait for upgrading.");
    sleep(THREE_MINITE);
    return true;
}


bool GSupgrade::verRunRead(string &ver_now)
{
    HttpResCod rep;
    char ch[256]={0};

    if(false == getHttpRequest(&rep,gs_web_port+"/gs-robot/info"))
    {
        log_error("GSupgrade::verRunRead  disconnect."); 
        return false;
    }
    Json::Reader reader;
    Json::Value val;

 //   log_info(rep.body.c_str());
    if(reader.parse(rep.body,val))
    {
       bool isSuccess = val["successed"].asBool();
       if(true == isSuccess)
       {
         ver_now = val["data"]["version"].asString();
       }
    }
    else
    {
          snprintf(ch, sizeof(ch), "get gaussin version false.%s====%d",__FUNCTION__,__LINE__);
          log_error(ch);
          return false;
    }
    return true; 
}
bool GSupgrade::verCfgRead(GSCfg &cfg)
{
    boost::unique_lock<boost::mutex> lock(mutex);
    ifstream fin(GS_FILE_CFG_PAHT);
    if(!fin.is_open())
    {
        log_error("can not open cfg file.");
        return false;
    }
	YAML::Node doc;
    doc = YAML::Load(fin);

    if(doc[0]["gaussioncfg"])
    {
        if(doc[0]["forceUp"])
        {
            string temp_force;
            doc[0]["forceUp"] >> temp_force;
            cfg.is_force = (("yes" == temp_force) || ("true" == temp_force))?true:false;
        }
        else
        {
            cfg.is_force = false;
        }

        if(doc[0]["version"])
        {
            doc[0]["version"] >> cfg.ver_cfg;
        }
        else
        {
            cfg.ver_cfg = "";
        }

        if(doc[0]["bin_file"])
        {
            doc[0]["bin_file"] >> bin_file_name;
            cfg.bin_path = GS_FILE_IMAGE_PATH + bin_file_name;
        }
        else
        {
            cfg.bin_path = "/home/atris/atris_app/firmware/gsnav/gs_image.gspkg";
        }
    }
    fin.close();
    return true;
}

bool GSupgrade::readUpgradeFile(std::string& file_data,string &bin_path)
{
   std::streampos size;
   std::ifstream file(bin_path,std::ios::in | std::ios::binary | std::ios::ate);
  if (file.is_open()) 
  {
        file.seekg(0, std::ios::end);
        size = file.tellg();
        file.seekg(0,std::ios::beg);
        file_data.resize(size);
        file.read((char*)file_data.data(),size);
        file.close();
        return true; 
  } 
 else 
 {
        log_error("open file error.");
        return false;
 }

}

bool GSupgrade::getHttpRequest(HttpResCod *rep,string req_link)
{

    if(NULL == rep) return false;
    *rep = httpGet(req_link); 

    if(200 != rep->code)
    {
        log_error("get httprequest error.code is =====>%d",rep->code);
        return false;
    }
    return true;

}

bool GSupgrade::postFileStreamHttpRequest(HttpResCod *res,string req_link,string &filePath)
{
    if(NULL == res)return false;

    *res = httpPost(req_link,"multipart/form-data",filePath);

    if(200 != res->code)
    {
        log_error("get postrequest error.code is =====>%d",res->code);
        return false;
    }
    return true;    
}



bool GSupgrade::parseHttpResponse(HttpResCod *rep,HttpRsp *parseRsp)
{


    if(NULL == rep || NULL == parseRsp) return false;
    Json::Reader reader;
    Json::Value val;

    //从字符串中读取数据
    if(reader.parse(rep->body,val))
    {
        parseRsp->data = val["data"].asString();
        parseRsp->errCode = val["errCode"].asString();
        parseRsp->msg = val["msg"].asString();
        parseRsp->successed = val["successed"].asBool();
    }
    return true;
}

HttpResCod GSupgrade::httpGet(string &url)
{
     HttpResCod rep={};
     CURL* curl = curl_easy_init();
     if(NULL == curl)
     {
         log_error("could not inti curl.");
         rep.code = -1;
         return rep;
     }
      //set header
     string header = "";
     curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
     curl_easy_setopt(curl, CURLOPT_TIMEOUT, 3);
     rep = sendHttpProtocol(curl,url,header);
     
     //free curlhandl   modified by marty.gong@ubtrobot.com:2018-9-03-10:55:45 
     if(curl)curl_easy_cleanup(curl);
     return rep; 
    
}

HttpResCod GSupgrade::httpPost(string &url,string type_,string &content_)
{
    HttpResCod rep;
    struct curl_httppost *formpost = NULL;
    struct curl_httppost *lastptr = NULL;
    
    CURL* curl= curl_easy_init();
    if(NULL == curl)
    {
        log_error("could not inti curl.");
        rep.code = -1;
        return rep;
    }
     //set header
    string header;
     header = "Content-Type:"+type_; 
    /** Now specify we want to POST data */
    if(type_ == "multipart/form-data")
    {
     
            curl_slist* headerList = NULL;
            #if 0
            curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
            curl_easy_setopt(curl, CURLOPT_DEBUGFUNCTION, on_debug_file_);
            #endif
            //form-data
           curl_formadd(&formpost, &lastptr,CURLFORM_COPYNAME, "file",CURLFORM_FILE,content_.c_str(), CURLFORM_END);
           curl_easy_setopt(curl,CURLOPT_HTTPPOST,formpost);
           rep = sendHttpProtocol(curl,url,header);
           //last free post
           curl_formfree(formpost);

           //free curlhandl   modified by marty.gong@ubtrobot.com:2018-9-03-10:55:45 
           if(curl)curl_easy_cleanup(curl);
           return rep;
    }
    else
    {
            curl_easy_setopt(curl, CURLOPT_POST, 1L);
            /** set post fields */
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, content_.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, content_.size());
            rep = sendHttpProtocol(curl,url,header);
            //free curlhandl   modified by marty.gong@ubtrobot.com:2018-9-03-10:55:45 
            if(curl)curl_easy_cleanup(curl); 
    }

    return rep;
  
}
HttpResCod GSupgrade::sendHttpProtocol(CURL*& curlHandl,string &url,string &headerstring )
{
     // init return type
     HttpResCod ret = {};
     CURLcode res = CURLE_OK;
     curl_slist* headerList = NULL;
    if(NULL == curlHandl)
    {
        ret.code = -1;
        return ret;
    }
    /** set query URL */
    curl_easy_setopt(curlHandl, CURLOPT_URL, url.c_str());

     /** set callback function */
    curl_easy_setopt(curlHandl, CURLOPT_WRITEFUNCTION,httpRecvDataCB);
    curl_easy_setopt(curlHandl, CURLOPT_WRITEDATA, (void*)&ret);

     /** set the header callback function */
    curl_easy_setopt(curlHandl, CURLOPT_HEADERFUNCTION,httpRecvDataHeaderCB);
    /** callback object for headers */
    curl_easy_setopt(curlHandl, CURLOPT_HEADERDATA, (void*)&ret);

    //ignore signal   modified by marty.gong@ubtrobot.com:2018-9-03-15:2:43 
    curl_easy_setopt(curlHandl, CURLOPT_NOSIGNAL, 1L);
    //当进程处理完毕后强制关闭会话，不再缓存供重用
    curl_easy_setopt(curlHandl,CURLOPT_FORBID_REUSE,1L);

    if(!headerstring.empty())
    {
        headerList = curl_slist_append(headerList, headerstring.c_str());
        curl_easy_setopt(curlHandl, CURLOPT_HTTPHEADER,headerList);
    }
    res = curl_easy_perform(curlHandl);
    if (res != CURLE_OK)
    {
        log_error("%s error(%d): %s", __FUNCTION__, res, curl_easy_strerror(static_cast<CURLcode>(res)));
        switch (res) 
        {
          case CURLE_OPERATION_TIMEDOUT:
            ret.code = res;
            ret.body = "Operation Timeout.";
            break;
          case CURLE_SSL_CERTPROBLEM:
            ret.code = res;
            ret.body = curl_easy_strerror(res);
            break;
          default:
            ret.body = "Failed to query.";
            ret.code = -1;
       }
    } 
   else 
   {
     int64_t http_code = 0;
     curl_easy_getinfo(curlHandl, CURLINFO_RESPONSE_CODE, &http_code);
     ret.code = static_cast<int>(http_code);
   }

  // free header list
  if(NULL != headerList )curl_slist_free_all(headerList);
  // reset curl handle
  curl_easy_reset(curlHandl);
  return ret; 
}



