#include "voip.h"

#include "libxml/xml.h"
#include "config/config.h"
#include "utils/utils.h"
#include "imemory/atris_imemory_api.h"

Voip::Voip()
    : calling_now_(false)
{
    if (Config::get_instance()->voip_auto_setting) {
        InitSetting();
    }
}

Voip::~Voip()
{
    
}

void Voip::InitSetting()
{
    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);

    std::vector<std::string> AttrText;
    std::string quickKey1Type = "dss.page.1.key.1.Type = 1";
    std::string quickKey1Value = "dss.page.1.key.1.Value = " + std::string(shmrbt.appdata.sip_num) + "@1/f";
    std::string quickKey1Title = "dss.page.1.key.1.Title =";
    std::string callOutUrl = "cti.OutgoingCallUrl = http://" + Config::get_instance()->local_ip + "/voip/outgoing_call";
    std::string call_start_url = "cti.CallActiveUrl = http://" + Config::get_instance()->local_ip + "/voip/call_established";
    std::string call_stop_url = "cti.CallStopUrl = http://" + Config::get_instance()->local_ip + "/voip/call_terminated";

    std::string robotSn = std::string(shmrbt.robot.sn);
    char letterToNum[4] = "0";
    if (robotSn.length() >= 9) {
        for (unsigned int i = 0; i < 3; i++) {
            letterToNum[i] = robotSn[i] - 16;
        }
        robotSn.replace(0, 3, letterToNum);
        robotSn.erase(6,3);
    }
    std::string line1PhoneNum = "sip.line.1.PhoneNumber=" + robotSn;
    std::string line1DispName = "sip.line.1.DisplayName=";
    std::string line1RegPort = "sip.line.1.RegPort=5060";
    std::string line1RegUser = "sip.line.1.RegUser=" + robotSn;
    std::string line1RegPswd= "sip.line.1.RegPswd=123456";
    std::string line1RegEnabled = "sip.line.1.RegEnabled=1";
    std::string line1RegAddr = "sip.line.1.RegAddr=" + Config::get_instance()->voip_sip_server;

    std::string keyHangUp = "call.basic.UseSDialHandDown = 0";

    AttrText.push_back(quickKey1Type);
    AttrText.push_back(quickKey1Value);
    AttrText.push_back(quickKey1Title);
    AttrText.push_back(callOutUrl);
    AttrText.push_back(call_start_url);
    AttrText.push_back(call_stop_url);
    AttrText.push_back(line1PhoneNum);
    AttrText.push_back(line1DispName);
    AttrText.push_back(line1RegPort);
    AttrText.push_back(line1RegUser);
    AttrText.push_back(line1RegPswd);
    AttrText.push_back(line1RegEnabled);
    AttrText.push_back(line1RegAddr);
    AttrText.push_back(keyHangUp);

    std::string httpResp;
    handleVoipAttributes(HANDLEVOIPTYPE_SET, AttrText, httpResp);
}

int Voip::CancelCall()
{
    std::string httpFlagString = "http://";
    std::string authString = "admin:admin@";
    std::string voipDeviceIp = Config::get_instance()->voip_ip;
    std::string httpPathString = "/cgi-bin/ConfigManApp.com?key=RELEASE";
    std::string httpUrl = httpFlagString + authString + voipDeviceIp + httpPathString;

    std::string resp;
    int ret = Utils::get_instance()->http_get(httpUrl, resp);

    return ret;
}

int Voip::GetVolume(int *volume)
{
    if (volume == nullptr) {
        return -1;
    }

    int result = -1;
    std::vector<std::string> itemNodeText;
    std::string getVolCmd = "phone.volume.HandFreeVol";
    itemNodeText.push_back(getVolCmd);

    std::string httpResp;
    std::string retCode = "";
    int ret = handleVoipAttributes(HANDLEVOIPTYPE_GET, itemNodeText, httpResp);
    Xml::queryNodeText(httpResp, "RetCode", retCode);
    if ((ret == 0) && (retCode == "0")) {
        std::vector<std::string> allItemText;
        Xml::queryAllChildNodeText(httpResp, "info", allItemText);
        int i = 0;
        int num = allItemText.size();
        std::string::size_type position;
        std::string volumeInfo = "phone.volume.HandFreeVol=";
        int volumeInfoLen = volumeInfo.size();
        for (i = 0; i < num; i++) {
            position = allItemText[i].find(volumeInfo);
            if (position != std::string::npos) {
                std::string volString(allItemText[i], position+volumeInfoLen);
                *volume = atoi(volString.c_str());
                result = 0;
            }
        }
    }

    return result;
}

int Voip::SetVolume(int volume)
{
    std::vector<std::string> itemNodeText;
    std::string setVolCmd = "phone.volume.HandFreeVol =" + std::to_string(volume);
    itemNodeText.push_back(setVolCmd);
    std::string httpResp = "";
    std::string retCode = "";
    int result = -1;

    int ret = handleVoipAttributes(HANDLEVOIPTYPE_SET, itemNodeText, httpResp);
    Xml::queryNodeText(httpResp, "RetCode", retCode);
    if ((ret == 0) && (retCode == "0")) {
        result = 0;
    }

    return result;
}

int Voip::SetSipNum(const std::string &num)
{
    int ret = -1;

    std::vector<std::string> itemNodeText;
    std::string quickKey1Value = "dss.page.1.key.1.Value = " + num + "@1/f";
    itemNodeText.push_back(quickKey1Value);
    std::string httpResp;
    if (Config::get_instance()->voip_auto_setting) {
        ret = handleVoipAttributes(HANDLEVOIPTYPE_SET, itemNodeText, httpResp);
    }

    return ret;
}

int Voip::handleVoipAttributes(HandleType handleType, const std::vector<std::string> &AttrText, std::string &httpResp)
{
    int ret = 0;

    std::string httpFlagString = "http://";
    std::string authString = "admin:admin@";
    std::string voipDeviceIp = Config::get_instance()->voip_ip;
    std::string httpPathString = "/xmlservice";
    std::string url = httpFlagString + authString + voipDeviceIp + httpPathString;

    std::string xmlString;
    std::string strRootNodeName = "FanvilConfiguration";
    TiXmlDocument *pDoc = new TiXmlDocument;
    if (NULL == pDoc) {
        return -1;
    }
    TiXmlDeclaration *pDeclaration = new TiXmlDeclaration("1.0", "UTF-8", "");
    if (NULL == pDeclaration) {
        return -1;
    }
    pDoc->LinkEndChild(pDeclaration);
    TiXmlElement *pRootEle = new TiXmlElement(strRootNodeName.c_str());
    if (NULL == pRootEle) {
        return -1;
    }
    pDoc->LinkEndChild(pRootEle);
    pRootEle->SetAttribute("Beep", "");

    std::string handleString = "";
    if (handleType == HANDLEVOIPTYPE_GET) {
        handleString = "get";
    } else if (handleType == HANDLEVOIPTYPE_SET) {
        handleString = "set";
    } else {
        return -1;
    }
    pRootEle->SetAttribute("cmd", handleString.c_str());

    int itemNum = AttrText.size();
    if (itemNum == 0) {
        return -1;
    }

    int i = 0;
    TiXmlElement *pItemEle = NULL;
    TiXmlText *setItemText = NULL;
    for (i = 0; i < itemNum; i++) {
        pItemEle = new TiXmlElement("Item");
        if (NULL == pItemEle) {
            ret =  -1;
            break;
        }
        pRootEle->LinkEndChild(pItemEle);
        setItemText = new TiXmlText(AttrText[i].c_str());
        if (NULL == setItemText) {
            ret =  -1;
            break;
        }
        pItemEle->LinkEndChild(setItemText);
    }

    TiXmlPrinter printer;
    pDoc->Accept(&printer);
    xmlString = printer.CStr();
    pDoc->Clear();
    delete pDoc;
    pDoc = NULL;

    std::vector<std::string> httpHeader;
    httpHeader.clear();
    httpHeader.push_back("Content-Type:text/xml");

    ret = Utils::get_instance()->http_post(url, xmlString, httpResp, httpHeader);
    return ret;
}
