#include "visionability.h"
#include <iostream>
#include <fstream>
#include <string>

VisionAbility::VisionAbility()
{
    vision_srv_ = nh_.advertiseService(SRV_GET_VISION_RESULT, &VisionAbility::on_recv_vision_req, this);
    cfg = Config::get_instance();
    vision_host_ = cfg->vision_host_;
    vision_port_ = cfg->vision_port_;
    log_debug("vision srv host:%s , port :%d", vision_host_.c_str(), vision_port_);

    grpc::ChannelArguments channelArg;
    channelArg.SetMaxReceiveMessageSize(8*1024*1024);
    std::string vision_target;
    vision_target = vision_host_ + ":" + std::to_string(vision_port_);
    log_debug("vision target : %s", vision_target.c_str());
    channel = grpc::CreateCustomChannel(vision_target, grpc::InsecureChannelCredentials(), channelArg);
    pvrssgStub = new Vrssg(channel);
}

bool VisionAbility::on_recv_vision_req(atris_msgs::GetVisionResult::Request &req, atris_msgs::GetVisionResult::Response &res)
{
    log_info("####recv_vision_req######");
    Vrssg::RecognizeResult result = pvrssgStub->Rocognize(req.model, req.image, req.index);
    res.error_code = result.code;
    res.result = result.result;
    res.msg = result.msg;
    log_info("result: %s", result.result.c_str());
    log_info("error code : %d", result.code);
    log_info("error msg : %s", result.msg.c_str());
    return true;
}
