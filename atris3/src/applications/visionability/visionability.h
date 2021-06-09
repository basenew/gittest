#pragma once

#include "vrssg.h"
#include "config/config.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/GetVisionResult.h"
#include "grpcpp/channel.h"

class VisionAbility {
public:
    //virtual ~VisionAbility();
    static VisionAbility* get_instance() {
        static VisionAbility singleton;
        return &singleton;
    }
private:
    VisionAbility();
	std::shared_ptr<grpc::Channel> channel;
	Vrssg *pvrssgStub;

    Config *cfg;
    std::string vision_host_;
    int vision_port_;
    ros::NodeHandle nh_;
    ros::ServiceServer vision_srv_;
    bool on_recv_vision_req(atris_msgs::GetVisionResult::Request &req, atris_msgs::GetVisionResult::Response &res);
};

