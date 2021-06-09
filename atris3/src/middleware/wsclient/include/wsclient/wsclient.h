#ifndef WSCLIENT_H_
#define WSCLIENT_H_

#include "libwebsockets.h"
#include <boost/thread.hpp>
#include <functional>


struct wsClientParam {
    std::string path;
    std::string protocolsName;
    std::string addr;
    int port;
};

class WsClient
{
public:
    typedef std::function <void (const std::string &string)> ReadCallback;
    WsClient();
    ~WsClient();

    void setParams(wsClientParam param);
    void setSsl(bool isSupportSsl, const char* caFilepath = NULL,
                const char* serverCertFilepath = NULL, 
                const char* serverPrivateKeyFilepath = NULL);
    inline bool isConnected(){return !wsServiceThreadExit_ && wsStateConnected_;};
    void start();
    void restart();
    void setReadCallback(const ReadCallback& cb)
    { 
        readCallback_ = cb; 
    }
    void sendMessageToServer(const std::string &string);

private:
    void wsConnectThread();
    void wsServiceThread();
    static int wsClientCallback(struct lws *wsi, enum lws_callback_reasons reason, 
                                void *user, void *in, size_t  len);

private:
    std::string serverPath_;
    std::string protocolsName_;
    std::string serverAddr_;
    int serverPort_;
   
    bool isSupportSsl_;
    struct lws_context *wsCtx_;
    struct lws_context_creation_info ctxInfo_;
    struct lws_client_connect_info connInfo_;
    struct lws *connLws_;

    boost::thread* wsServiceThread_;
    bool wsServiceThreadExit_;
    bool wsStateConnected_;

    ReadCallback readCallback_;
};

#endif
