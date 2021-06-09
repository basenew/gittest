#include "log/log.h"
#include "proto/api.grpc.pb.h"
#include "grpcpp/support/channel_arguments.h"
#include "grpcpp/grpcpp.h"
#include <vector>

class Vrssg
{
private:
    std::unique_ptr<vrssg::VisionRecognitionService::Stub> stub_;

public:
    struct Meter
    {
        std::string model;
        std::string image;
        std::string comments;
    };

    enum ErrorCode {
        ERROR_SUCCESS = 0,
        ERROR_FAILED = 1,
        // Already in recognizing which SDK can't running two different recognize
        // instance in same time for current
        ERROR_ALREADY_IN_RECOGNIZING = 2,
        ERROR_INVALID_INPUT_IMAGE_DATA = 3,
        ERROR_INVALID_INPUT_MODEL_NAME = 4,
        ERROR_TARGET_NOT_FOUND = 5
    };
    
    enum SwitchStatus {
        SWITCH_STATUS_UNKOWN = 0,
        SWITCH_STATUS_ON = 1,
        SWITCH_STATUS_OFF = 2
    };

    struct Rectangle {
        int32_t left;
        int32_t top;
        int32_t width;
        int32_t height;
    };

    struct SwitchResults
    {
        std::string id;
        Rectangle rect;
        SwitchStatus status;
        float score;
    };

    struct MeterResults
    {
        std::string id;
        Rectangle rect;
        float value;
        float score;
    };

    struct RecognizeResult
    {
        std::string result;
        SwitchResults switch_results;
        MeterResults meter_results;
        std::string msg;
        ErrorCode code;
    };
    
public:
    Vrssg(std::shared_ptr<grpc::Channel> channel) : stub_(vrssg::VisionRecognitionService::NewStub(channel)){}
    ~Vrssg(){}
    std::vector<Meter *> ListMeters()
    {
        std::vector<Meter *> ret;
        grpc::ClientContext context;
        google::protobuf::Empty request;
        vrssg::Meters meters;
        grpc::Status status = stub_->ListMeters(&context, request, &meters);
        if (status.ok())
        {
            for (size_t i = 0; i < meters.meter_size(); i++)
            {
                Meter *m = new Meter();
                vrssg::Meter meter = meters.meter(i);
                m->model = meter.model();
                m->image = meter.image();
                m->comments = meter.comments();
                ret.push_back(m);
            }
        }else
        {
            std::cout << "ListMeters failed !!! " << status.error_message() << std::endl;
        }
        
        return ret;
    }

    RecognizeResult Rocognize(std::string model, std::string image, std::string str_id)
    {
		log_info("%s %s", __FUNCTION__, str_id.c_str());
		grpc::ClientContext context;
        vrssg::Input request;
        request.set_model(model);
        request.set_image(image);

        vrssg::Result result;
        vrssg::MeterResults meter_results;
        vrssg::SwitchResults switch_results;
        RecognizeResult reco_result;

		grpc::Status status = stub_->Recognize(&context, request, &result);
      	if (!status.ok()) {
			log_error("recognize with error detail:%s msg:%s", status.error_details().c_str(), status.error_message().c_str());
			return reco_result;
		}

        if (result.code() != vrssg::ERROR_SUCCESS) {
        	log_error("process with error code:%d msg:%s", result.code(), status.error_message().c_str());
			return reco_result;
		}

        reco_result.code = ErrorCode(result.code());
        reco_result.msg = status.error_message();
		int id = 0;
		if (!str_id.empty())
			id = std::atoi(str_id.c_str());

        if (result.has_meter_results()) {
            int size = result.meter_results().results_size();
            for (int i = 0; i < size; i++) {
                const vrssg::MeterResult& meter_result = result.meter_results().results(i);
                int value_size = meter_result.value_size();
                for (int j = 0; j < value_size; j++) {
                    const vrssg::MeterValue& meter_value = meter_result.value(j);
					log_info("id:%d val:%f", meter_value.id(), meter_value.value());
                    if (meter_value.id() == id){
						reco_result.result = std::to_string(meter_value.value());
						return reco_result;
			        }
                }
            }
        } else if (result.has_switch_results()) {
            int size = result.switch_results().results_size();
            for (int i = 0; i < size; i++) {
                const vrssg::SwitchResult& switch_result = result.switch_results().results()[i];
				log_info("id:%d val:%d", switch_result.id(), switch_result.status());
                if (switch_result.id() == str_id){
                    switch (switch_result.status()) {
                        case SWITCH_STATUS_ON: {reco_result.result = "on";    break;}
                        case SWITCH_STATUS_OFF:{reco_result.result = "off";   break;}
                        default:               {reco_result.result = "unkown";break;}
                    }
					return reco_result;
                }
            }
        }
		return reco_result;
    }
};

