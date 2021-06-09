#include "negotiates.h"
#include "log/log.h"
#include <json/json.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <boost/thread.hpp>

#include "imemory/atris_imemory_api.h"

Negotiates::Negotiates()
{
    create_udp_server();
}


void Negotiates::create_udp_server()
{
    new boost::thread(boost::bind(&Negotiates::udp_server_proc, this));
}

void Negotiates::udp_server_proc()
{
retry:
    int sockfd = -1;
    int opt = 1;
    Json::Reader reader;
    Json::Value root;
    unsigned int l = sizeof(struct sockaddr_in);
    struct sockaddr_in client;
    unsigned char data[1024];
    unsigned char sndbuf;
    struct sockaddr_in server;
    bzero(&server,sizeof(server));
    server.sin_family=AF_INET;
    server.sin_addr.s_addr=htonl(INADDR_ANY);
    server.sin_port=htons(16755);

    if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
        log_once_error("%s create udp socket fail.", __FUNCTION__);
        sleep(1);
        goto retry;
    }
    
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt));

    if((bind(sockfd, (struct sockaddr *)&server, sizeof(server))) < 0){
        log_once_error("%s bind error.", __FUNCTION__);
        close(sockfd);
        sleep(1);
        goto retry;
    }
    
    while(1){
        memset(data, 0, sizeof(data));

        int ret = ::recvfrom(sockfd, data, sizeof(data), 0, (struct sockaddr *)&client, &l);
        char client_addr[INET_ADDRSTRLEN] = {0};
        inet_ntop(AF_INET,&client.sin_addr, client_addr, sizeof(client_addr));
        log_once_info("%s recvfrom ip: %s, port: %d, dataSize: %d, data: %s", 
          __FUNCTION__, client_addr, ntohs(client.sin_port), ret, data);
        if(ret < 0) {
            log_once_error("%s errorno: %d  %s", __FUNCTION__, errno, strerror(errno));
            close(sockfd);
            sleep(1);
            goto retry;
        }

        if(!reader.parse((const char *)data, root)){
            log_once_error("%s wrong data.", __FUNCTION__);
            sleep(1);
            continue;
        }

        if (!root["cmd"].isNull()) {
            std::string cmd = root["cmd"].asString();
            if(cmd == "req_sn"){
                shm::Robot shmrbt;
                shm::iMemory_read_Robot(&shmrbt);

                Json::Value resp;
                resp["sn"] = Json::Value(shmrbt.robot.sn);

                Json::FastWriter fw;
                std::string resp_data = fw.write(resp);
                
                log_once_info("%s send to client: %s", __FUNCTION__, resp_data.c_str());
                if(sendto(sockfd, resp_data.c_str(), resp_data.size(), 0, (struct sockaddr*)&client, l)<0){
                    log_once_error("%s send to client fail.", __FUNCTION__);
                    sleep(1);
                }
            }
        }
    }
}

