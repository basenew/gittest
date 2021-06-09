#include <ros/ros.h>
#include <tiny_ros/ros.h>
#include "mosquittopp.h"
#include "police_message.h"
#include "message_to_web.h"
#include "mqttengine.h"
#include <signal.h>
#include <curl/curl.h>
#include "imemory/atris_imemory_api.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "signaling");
    tinyros::init("signaling");
    
    shm::iMemory_init();

    signal(SIGPIPE, SIG_IGN);
    curl_global_init(CURL_GLOBAL_ALL);
    mosqpp::lib_init();

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string sn = shmrbt.robot.sn;

    MessageToWeb *instance = MessageToWeb::get_instance();
    if (instance->InitRemoteMqttTopic() == 0) {
        mqtt_params remoteParams;
        remoteParams.host = Config::get_instance()->remote_mqtt_host_;
        remoteParams.username = Config::get_instance()->remote_mqtt_username_;
        remoteParams.password = Config::get_instance()->remote_mqtt_password_;
        remoteParams.port = Config::get_instance()->remote_mqtt_port_;
        remoteParams.encrypt = Config::get_instance()->remote_mqtt_encrypt_;
        remoteParams.keepalive = Config::get_instance()->remote_mqtt_keepalive_;
        remoteParams.msg_type = "remote_mqtt";
        remoteParams.client_id = "poweratris_" + sn;
        MqttEngine * remoteMqtt_ = new MqttEngine(remoteParams);
    }

    mqtt_params localParams;
    localParams.host = Config::get_instance()->mqtt_host_;
    localParams.username = Config::get_instance()->mqtt_username_;
    localParams.password = Config::get_instance()->mqtt_password_;
    localParams.port = Config::get_instance()->mqtt_port_;
    localParams.encrypt = Config::get_instance()->mqtt_encrypt_;
    localParams.keepalive = Config::get_instance()->mqtt_keepalive_;
    localParams.msg_type = "local_mqtt";
    localParams.client_id = "Atris_" + sn;
    MqttEngine *localMqtt_ = new MqttEngine(localParams);
    
#ifdef _POLICE_
    MessageHandle::createInstance();
#endif

  	ros::MultiThreadedSpinner s(10);
  	ros::spin(s);

    curl_global_cleanup();
    mosqpp::lib_cleanup();

    return 0;
}


