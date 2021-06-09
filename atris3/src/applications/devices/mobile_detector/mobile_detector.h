#ifndef MOBILE_DETECT
#define MOBILE_DETECT

#include <boost/thread.hpp>
#include "list.h"
#include "ros/ros.h"
#include "log/log.h"
#include "atris_defines.h"
#include "atris_msgs/SignalMessage.h"
#include "transferfile/transferfile.h"

typedef struct MobileDetect_Data_{
    std::string imsi;
    std::string latitude;
    std::string longitude;
}MobileDetect_Data;


class MobileDetect
{
    public:
        ~MobileDetect();

        int init(void);
        static MobileDetect* get_instance(){
                static MobileDetect singleton;
                return &singleton;
        }

    private:
        MobileDetect();
        void udp_receive_thread(void);
        void data_process_thread(void);
        int parser_detected_data(std::string databuff);
        void notify_detected_data(void);
        static void upload_state_handler(TransferStates state, std::string &path);
        static void download_state_handler(TransferStates state, std::string &path);
        boost::mutex mobile_decected_data_lock;
        MsgList<char *> msg_list;
        MobileDetect_Data mobileDetect_data;
        ros::NodeHandle nh_;
        ros::Subscriber signal_req_sub_;
        void on_recv_mobile_detect_ctrl(const atris_msgs::SignalMessage &msg);
        void imsi_detection_file_export_thread(atris_msgs::SignalMessage &export_msg);
};


#endif
