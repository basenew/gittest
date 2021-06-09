#ifndef GS_UPGRADE_H_H
#define GS_UPGRADE_H_H


#include <string.h>
#include <json/json.h>

#include "curl/curl.h"
#include "config/config.h"
#include "log/log.h"
using namespace std;


typedef struct{
    string ver_cfg;
    bool is_force;
    string bin_path;
}GSCfg;

typedef struct{
    string data;
    string  errCode;
    string msg;
    bool successed;
}HttpRsp;



typedef struct {
  int code;
  std::string body;
  std::string header;
} HttpResCod;

class GSupgrade
{
    public:
        GSupgrade();
        ~GSupgrade();

        bool init();
        bool upgrade();
        bool getVersion(string & ver_);

    private:

        bool tryUpgrade();
        bool rebootGSsys();
        bool versionCheck(string &ver_cfg);
        bool checkGSDisconnect();
        bool checkGSconnect();
        bool verGet(string &ver_now,GSCfg &cfg);
        bool verCfgRead(GSCfg &cfg);
        bool verRunRead(string &ver_now);
        bool verCompare(string &ver_now,GSCfg &gs_cfg,bool &isNeedUp);
        bool startUpgrade(string& bin_path);
        bool readUpgradeFile(std::string& file_data,string &bin_path);
        HttpResCod httpGet(string &url);
        HttpResCod httpPost(string &url, string type_, string &context_);
        bool getHttpRequest(HttpResCod *rep,string req_link);
        bool postFileStreamHttpRequest(HttpResCod *res,string req_link,string &fileContext);
        bool parseHttpResponse( HttpResCod *rep,HttpRsp *parseRsp);
    //    size_t httpRecvDataCB(void *data, size_t size, size_t nmemb,void *userdata);
  //      size_t httpRecvDataHeaderCB(void *data, size_t size, size_t nmemb,void *userdata);
        HttpResCod sendHttpProtocol(CURL*& crulHandl,string &url,string &headerstring );
        bool gsConnectIpSelect();
        bool gsSetSystemIp(string ip_addr_,string defaut_gw);
    private:
        string bin_file_name;
        string gs_web_port;
        string gs_web_port_cfg;
        string gs_web_port_bkup;
        Config *cfg;
        boost::mutex mutex;

        bool is_systemp_ip_changed;
    };



#endif
