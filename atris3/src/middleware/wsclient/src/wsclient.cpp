#include "wsclient.h"
#include "log/log.h"

#define MAX_PAYLOAD_SIZE  (10 * 1024)

struct session_data {
    unsigned char buf[LWS_PRE + MAX_PAYLOAD_SIZE];
    int len;
};

WsClient::WsClient()
  : serverPath_(""),
    protocolsName_(""),
    serverAddr_(""),
    serverPort_(0),
    isSupportSsl_(nullptr),
    wsCtx_(nullptr),
    connLws_(nullptr),
    wsServiceThread_(nullptr),
    wsServiceThreadExit_(false),
    wsStateConnected_(false)
{
    memset(&ctxInfo_, 0, sizeof(ctxInfo_));
    memset(&connInfo_, 0, sizeof connInfo_);
}

WsClient::~WsClient()
{
    
    wsServiceThreadExit_ = true;
    if (wsServiceThread_) {
        wsServiceThread_->interrupt();
        wsServiceThread_->join();
        delete wsServiceThread_;
        wsServiceThread_ = nullptr;
    }

    if (wsCtx_) {
        lws_context_destroy(wsCtx_);
        wsCtx_ = nullptr;
    }
}

void WsClient::setParams(wsClientParam param)
{
    serverPath_ = param.path;
    protocolsName_ = param.protocolsName;
    serverAddr_ = param.addr;
    serverPort_ = param.port;
}

void WsClient::setSsl(bool isSupportSsl, const char* caFilepath,
                      const char* serverCertFilepath, 
                      const char* serverPrivateKeyFilepath)
{
    isSupportSsl_ = isSupportSsl;
    if (isSupportSsl) {
        ctxInfo_.options |= LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;
    }
    ctxInfo_.ssl_ca_filepath = caFilepath;
    ctxInfo_.ssl_cert_filepath = serverCertFilepath;
    ctxInfo_.ssl_private_key_filepath = serverPrivateKeyFilepath;
}

void  WsClient::restart()
{
    if (isConnected()){
        log_info("ws already connected");
        wsStateConnected_ = false;
    }
    else
        start();
}

void  WsClient::start()
{
    static const struct lws_protocols lwsProtocols[] = {
        {
            protocolsName_.c_str(),
            wsClientCallback,
            sizeof(struct session_data), 
            MAX_PAYLOAD_SIZE,
        },
        { NULL, NULL, 0, 0}
    };

    int n = 0;
    ctxInfo_.options = LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT;
    ctxInfo_.port = CONTEXT_PORT_NO_LISTEN;
    ctxInfo_.protocols = lwsProtocols;

    wsCtx_ = lws_create_context(&ctxInfo_);
    if (!wsCtx_) {
        lwsl_err("lws init failed\n");
        return;
    }

    memset(&connInfo_, 0, sizeof connInfo_); 
    connInfo_.context = wsCtx_;
    connInfo_.port = serverPort_;
    connInfo_.address = serverAddr_.c_str();
    connInfo_.path = serverPath_.c_str();
    connInfo_.host = connInfo_.address;
    connInfo_.origin = connInfo_.address;
    connInfo_.protocol = lwsProtocols[0].name;
    if (isSupportSsl_) {
        connInfo_.ssl_connection = 1;
    } else {
        connInfo_.ssl_connection = 0;
    }

    wsServiceThread_ = new boost::thread(boost::bind(&WsClient::wsServiceThread, this));
}


void WsClient::wsServiceThread()
{
    int n = 0;

    while (!wsServiceThreadExit_ && n >= 0) {
        if (!wsStateConnected_) {
            log_error("ws try connect...");
            wsStateConnected_ = true;
            connLws_ = lws_client_connect_via_info(&connInfo_);
            if (connLws_) {
                lws_set_opaque_user_data(connLws_, this);
            }
            n = lws_service(wsCtx_, 300);
            sleep(1);
        } else  {
            n = lws_service(wsCtx_, 300);
        }
    }
}

int WsClient::wsClientCallback(struct lws *wsi, enum lws_callback_reasons reason, 
                            void *user, void *in, size_t  len)
{
    WsClient * wc = (WsClient *)lws_get_opaque_user_data(wsi);
    switch (reason) {
        case LWS_CALLBACK_CLIENT_CONNECTION_ERROR:
            {
                /*lwsl_notice("ws client connect error: %s\n",
                            in ? (char *)in : "(null)");*/
                if (wc) {
                    wc->wsStateConnected_ = false;
                    // log_error("ws disconnect");
                }
            }
            break;
        case LWS_CALLBACK_CLIENT_ESTABLISHED:
            {
                // lwsl_notice("%s: established\n", __func__);
                if (wc) {
                    wc->wsStateConnected_ = true;
                    // log_info("ws connected");
                }
            }
            break;
        case LWS_CALLBACK_CLIENT_RECEIVE:
            {
                // lwsl_notice("ws server receive message\n");
                if (wc && wc->readCallback_) {
                    // log_info("ws data 111:%s", (const char *)in);
                    wc->readCallback_((const char *)in);
                    // log_info("ws data 222");
                }
            }
            break;
        case LWS_CALLBACK_CLIENT_CLOSED:
        case LWS_CALLBACK_WSI_DESTROY:
            // log_info("ws LWS_CALLBACK_CLIENT_CLOSED LWS_CALLBACK_WSI_DESTROY");
            // lwsl_notice("ws server closed\n");
            if (wc) {
                wc->wsStateConnected_ = false;
                // log_info("ws destroyed");
            }
            break;
        default:
            //log_info("ws unknow state");
            //lwsl_notice("reason is %d\n", reason);
            break;
    }

    return lws_callback_http_dummy(wsi, reason, user, in, len);
}

void WsClient::sendMessageToServer(const std::string &message)
{
    struct session_data data;
    memset( data.buf, 0, sizeof(data.buf));
    char *msg = (char *) &data.buf[ LWS_PRE ];
    if (message.length() <= MAX_PAYLOAD_SIZE) {
        memcpy(msg, message.c_str(), message.length());
    }
    data.len = message.length();
    
    if (connLws_ && wsStateConnected_) {
        lws_write(connLws_, &data.buf[ LWS_PRE ], data.len, LWS_WRITE_TEXT );
    }
}
