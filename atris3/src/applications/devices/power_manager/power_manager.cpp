#include <boost/thread/thread.hpp>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <stdio.h>
#include <json/json.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "power_manager.h"
#include "utils/utils.h"
#include "can_service/can_service.h"
#include "task_manager/task_manager.h"
#include "chassis/chassis_manager.h"
#include "flash_lamp/flash_lamp.h"
#include "database/sqliteengine.h"

#define WHELL_DISTANCE          (0.7898)
#define BUMPER_TTS_INTERVAL     (30.0)
#define BATTERY_SHORT(b1, b2)  (b1 | (b2 & 0x00FF) << 8)
#define BATTERY_INFO_PATH   "/userdata/atris_app/battery/"

#define BATTERY_SOC_REPORT_TIMES    30

PowerManager::PowerManager() {
    power_off_sub_ = nh_.subscribe(TOPIC_POWER_OFF_MESSAGE, 100, &PowerManager::on_power_off, this);
    aisound_tts_pub_ = nh_.advertise<atris_msgs::AisoundTTS>(TOPIC_AISOUND_TTS_MESSAGE, 100);
    lamp_cmd_pub_ = nh_.advertise<atris_msgs::LampCmd>(TOPIC_LAMP_CMD_MESSAGE, 100);
    diag_info_pub_ = nh_.advertise<atris_msgs::RobotInfo>(TOPIC_DIAGNOSTIC_ROBOT_INFO, 100);
    charge_info_pub_ = nh_.advertise<atris_msgs::ChargeInfo>(TOPIC_CHARGE_INFO_MESSAGE, 100);
    charge_cmd_sub_ = nh_.subscribe(TOPIC_CHARGE_CMD_MESSAGE, 100, &PowerManager::on_receive_charge_cmd, this);
    nav_reset_pub_ = nh_.advertise<atris_msgs::NavReset>(TOPIC_NAV_RESET, 100);
    get_gps_position_srv_client_ = nh_.serviceClient<atris_msgs::GetGpsPos>(SRV_GET_GPS_POSITION);
    is_power_off_ = false;
    remote_control_mode = EMC_MODE;
    brake_state = -1;
    can = NULL;
    can_send =false;
    tts_key_ = TTSStrings::TTS_KEY_NULL;
    tts_ext_key_ = TTSStrings::TTS_KEY_NULL;
    brake_data1 = -1;
    brake_data2 = -1;
    bumper_data = -1;
    power_off_serial_num_ = "";

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    rbt_sn_ = shmrbt.robot.sn;
    log_info("%s rbt_sn: %s", __FUNCTION__, rbt_sn_.c_str());
}

int PowerManager::init()
{
    report_count = 0;
    remote_status = DF_REMOTE_OFF_LINE;
    can = NULL;
    can = new CanDevice(DEV_NAME);
    int ret = can->can_open();
    if(ret < 0){
        log_error("open can device fail.");
        can = NULL;
        return -1;
    }

    struct can_filter filters[11];

    filters[0].can_id = CH_BATTERY_0;
    filters[0].can_mask = 0xFFFF;
    filters[1].can_id = CH_BATTERY_1;
    filters[1].can_mask = 0xFFFF;
    filters[2].can_id = CH_BATTERY_2;
    filters[2].can_mask = 0xFFFF;
    filters[3].can_id = CH_PM_CTRL;
    filters[3].can_mask = 0xFFFF;

    filters[4].can_id = CH_BATTERY_TEST_0;
    filters[4].can_mask = 0xFFFF;
    filters[5].can_id = CH_BATTERY_TEST_1;
    filters[5].can_mask = 0xFFFF;
    filters[6].can_id = CH_BATTERY_TEST_2;
    filters[6].can_mask = 0xFFFF;
    filters[7].can_id = CH_BATTERY_TEST_3;
    filters[7].can_mask = 0xFFFF;

    filters[8].can_id = CH_PM_MONITOR;
    filters[8].can_mask = 0xFFFF;

    filters[9].can_id = CH_POWER;
    filters[9].can_mask = 0xFFFF;

    filters[10].can_id = CH_LAMP;
    filters[10].can_mask = 0xFFFF;

    can->set_filter(filters, sizeof(filters));

    TaskManager *tm = TaskManager::get_instance();
    Task fan_task;
    fan_task.cb = boost::bind(&PowerManager::do_cycle_task, this);
    tm->post_cycle(fan_task, 1*1000);

    new boost::thread(boost::bind(&PowerManager::proc_can_data, this));
    brake_time = ros::Time::now();
    return 0;
}

int PowerManager::do_cycle_task() {
    CanPkg package = {0};
    package.head.id.channel = CH_PM_MONITOR;
    package.data[0] = PM_CMD_BOTTOM_FAN;
    CanService::get_instance()->send(package, 1);

    usleep(1000*1000);

    package.head.id.channel = CH_POWER;
    package.data[0] = PM_CMD_MIDDLE_FAN;
    CanService::get_instance()->send(package, 1);

    usleep(1000*1000);

    package.head.id.channel = CH_POWER;
    package.data[0] = PM_CMD_POWER_STATUS;
    CanService::get_instance()->send(package, 1);

    usleep(1000*1000);

    package.head.id.channel = CH_POWER;
    package.data[0] = PM_CMD_POWER_IAP;
    CanService::get_instance()->send(package, 1);

    usleep(1000*1000);

    package.head.id.channel = CH_LAMP;
    package.data[0] = PM_CMD_ULTRASOUND_VER;
    CanService::get_instance()->send(package, 1);

    usleep(1000*1000);
    
    // send project_id to monitor
    package.head.id.channel = CH_PM_MONITOR;
    package.data[0] = PM_INFO_PROJECT_ID;
    package.data[1] = 0x01;
    if (rbt_sn_.find("DAA") != std::string::npos) {
        package.data[1] = 0x01;
    } else if (rbt_sn_.find("DAF") != std::string::npos) {
        package.data[1] = 0x02;
    } else if (rbt_sn_.find("DAD") != std::string::npos) {
        package.data[1] = 0x03;
    } else if (rbt_sn_.find("DAG") != std::string::npos) {
        package.data[1] = 0x04;
    } else if (rbt_sn_.find("DAH") != std::string::npos) {
        package.data[1] = 0x05;
    }
    CanService::get_instance()->send(package, 2);

    return 0;
}

void PowerManager::proc_can_data()
{
    CanPkg pkg = {0};
    int ret = 0;
can_send =true;
    while(1){
        memset(&pkg, 0, sizeof(CanPkg));
        ret = can->can_read(&pkg, 2000);
        if(ret > 0){
            do_can(&pkg);
        }
        else{
            //reconnect can device
#if 1
            can_send =false;
            log_warn("reconnect can device.");

            can->can_close();
            can->can_open();
            can_send =true;
#endif
        }
    }
}

void PowerManager::do_can(CanPkg *pkg)
{
    //dump_can_pkg(pkg);
   // log_info("%s  channed:%2x,cmd:%2x,data1:%2x,data2:%2x", __FUNCTION__,pkg->head.id.channel,pkg->data[0],pkg->data[1],pkg->data[2]);
    int ret = do_battery_data(pkg);
    if(ret == 0)
        return;
    ret = do_power_control(pkg);
    if(ret == 0)
        return;

    ret = do_monitor(pkg);
    if(ret == 0)
        return;

    ret = do_power(pkg);
    if(ret == 0)
        return;
}

int PowerManager::do_battery_data(CanPkg *pkg)
{
    Json::FastWriter fw;
    Json::Value root;
    atris_msgs::RobotInfo rbtInfo;

    if(pkg->head.id.channel == CH_BATTERY_0){
        root["robot_info"]["battery"]["voltage"] = battery_voltage = (pkg->data[0] | (pkg->data[1] << 8)) *  0.1;
        root["robot_info"]["battery"]["current"] = battery_current = ((int16_t)(pkg->data[2] | (pkg->data[3] << 8))) * 0.1;
        root["robot_info"]["battery"]["level"] = pkg->data[4];
        root["robot_info"]["battery"]["temp_max"] = (int8_t)pkg->data[5];
        root["robot_info"]["battery"]["temp_min"] = (int8_t)pkg->data[6];
        root["robot_info"]["battery"]["status"] = pkg->data[7];

        rbtInfo.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo);

        return 0;
    } else if (pkg->head.id.channel == CH_BATTERY_1) {
        root["robot_info"]["battery"]["charge_cnt"] = pkg->data[0];
        root["robot_info"]["battery"]["discharge_cnt"] = pkg->data[1];
#if 0
        root["robot_info"]["battery"]["health"] = pkg->data[2];
#endif
        root["robot_info"]["battery"]["bat_num"] = pkg->data[3];
        root["robot_info"]["battery"]["relay_status"] = pkg->data[4];
        root["robot_info"]["battery"]["charge_status"] = pkg->data[5];
        root["robot_info"]["battery"]["discharge_status"] = pkg->data[6];

        rbtInfo.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo);
        return 0;
    } else if (pkg->head.id.channel == CH_BATTERY_2) {
        log_error("%s CH_BATTERY_2 battery alarm");
        root["robot_info"]["battery"]["vstatus"] = ((uint16_t)(pkg->data[0] | (pkg->data[1] << 8)));
        root["robot_info"]["battery"]["cstatus"] = ((uint16_t)(pkg->data[2] | (pkg->data[3] << 8)));
        root["robot_info"]["battery"]["tstatus"] = ((uint16_t)(pkg->data[4] | (pkg->data[5] << 8)));
        root["robot_info"]["battery"]["alarm"] = pkg->data[6];

        rbtInfo.json = fw.write(root);
        diag_info_pub_.publish(rbtInfo);
        return 0;
    }

    return -1;
}

/**
 * @brief do_power_control process power control information
 *
 * @param pkg
 *
 * @return
 */
int PowerManager::do_power_control(CanPkg *pkg)
{
    if(pkg->head.id.channel == CH_PM_CTRL){
        unsigned char cmd = pkg->data[0];
        if(cmd == PM_CMD_START){
            log_info("recv atris power start cmd");
        }
        else if(cmd == PM_CMD_POWEROFF){
            log_info("recv atris power off cmd after power button is pressed");
            proc_poweroff((int)pkg->data[1]);
        }
        else if(cmd == PM_CMD_ECO){
            log_info("recv atris eco cmd");
        }
        else if(cmd == PM_CMD_POWEROFF_RESP){
            log_info("recv poweroff finish resp.");
            boost::unique_lock<boost::mutex> wlock(poff_mutex);
            poff_cv.notify_one();
        }

        return 0;
    }

    return -1;
}

void PowerManager::on_receive_charge_cmd(const atris_msgs::ChargeCmd &msg)
{
    log_info("[receive charge cmd] = %x", msg.cmd);
    CanPkg pkg;
    pkg.head.id.channel = CH_PM_MONITOR;
    pkg.data[0] = msg.cmd;
    for(int i=0; i<7; i++){
        pkg.data[i+1] = msg.data[i];
    }
    if(can && can_send)
     can->can_write(&pkg, 1);
}

int PowerManager::do_power(CanPkg *pkg) {
    Json::FastWriter fw;
    Json::Value root;
    atris_msgs::RobotInfo rbtInfo;

    if(pkg->head.id.channel == CH_POWER){
        unsigned char cmd = pkg->data[0];
        if(cmd == PM_INFO_MIDDLE_FAN){
            root["robot_info"]["fan"]["middle"]["speed_in"] = pkg->data[1];
            root["robot_info"]["fan"]["middle"]["speed_out"] = pkg->data[2];
            root["robot_info"]["fan"]["middle"]["error"] = pkg->data[3];
            rbtInfo.json = fw.write(root);
            diag_info_pub_.publish(rbtInfo);
        } else if (cmd == PM_INFO_POWER_STATUS) {
            root["robot_info"]["power"]["rgbd"] = (pkg->data[1] >> 0) & 0x01;
            root["robot_info"]["power"]["disperse"] = (pkg->data[1] >> 1) & 0x01;
            root["robot_info"]["power"]["netswitch"] = (pkg->data[1] >> 2) & 0x01;
            root["robot_info"]["power"]["gps"] = (pkg->data[1] >> 3) & 0x01;
            root["robot_info"]["power"]["gs"] = (pkg->data[1] >> 4) & 0x01;
            root["robot_info"]["power"]["slave_lidar"] = (pkg->data[1] >> 5) & 0x01;
            root["robot_info"]["power"]["main_lidar"] = (pkg->data[1] >> 6) & 0x01;
            root["robot_info"]["power"]["yuntai"] = (pkg->data[1] >> 7) & 0x01;
            root["robot_info"]["power"]["m_fan_in"] = (pkg->data[2] >> 0) & 0x01;
            root["robot_info"]["power"]["m_fan_out"] = (pkg->data[2] >> 1) & 0x01;
            root["robot_info"]["power"]["ks136"] = (pkg->data[2] >> 2) & 0x01;
            root["robot_info"]["power"]["ks106"] = (pkg->data[2] >> 2) & 0x01;
            rbtInfo.json = fw.write(root);
            diag_info_pub_.publish(rbtInfo);
        } else if (cmd == PM_INFO_POWER_IAP) {
            root["robot_info"]["version"]["power_iap"] = (uint16_t)(pkg->data[1] | pkg->data[2] << 8);
            rbtInfo.json = fw.write(root);
            diag_info_pub_.publish(rbtInfo);
        }

        return 0;
    } else if (pkg->head.id.channel == CH_LAMP) {
        unsigned char cmd = pkg->data[0];
        if(cmd == PM_INFO_ULTRASOUND_VER){
            root["robot_info"]["version"]["ultroso_ks106"] = (uint16_t)(pkg->data[1] | pkg->data[2] << 8);
            root["robot_info"]["version"]["ultroso_ks136"] = (uint16_t)(pkg->data[3] | pkg->data[4] << 8);
            rbtInfo.json = fw.write(root);
            diag_info_pub_.publish(rbtInfo);
        }
        return 0;
    }

    return -1;
}


double PowerManager::get_uptime()
{
	const char *cmd = "awk '{print $1}' /proc/uptime";
	char buf[128] = {0};

	FILE *fp = popen(cmd, "r");
	fread(buf, 1, sizeof(buf), fp);
	pclose(fp);
	
	return atof(buf);
}

int PowerManager::brake(BrakeState status)
{

    CanPkg pkg;
    pkg.head.id.channel = CH_PM_MONITOR;
    pkg.head.id.size = 1;
    pkg.data[0] = CAN_CMD_BRAKE;
    
    static double time = -1.0f;
#if 0
    if(time < 0.0f)
        time = ros::Time::now().toSec();
    double sec =ros::Time::now().toSec() - time;
    if((sec < 1.0f) && (sec > 0.0f))
        return 1;
    time = ros::Time::now().toSec();
#endif
	if(time < 0.0f){
		time = get_uptime();
	}else{
		double sec = get_uptime() - time;
		if(sec < 1.0f && sec > 0.0f)
			return 0;
	}
	time = get_uptime();   

   if(status == BRAKE_ON){

       pkg.data[1] = 1;
       log_info("%s send chassis manager is brake:%d", __FUNCTION__,pkg.data[1]);
   } else {

       pkg.data[1] = 0;
       log_info("%s send chassis manager is no brake:%d", __FUNCTION__,pkg.data[1]);

   }
   if(can && can_send)
       can->can_write(&pkg, 2);

     
    return 1;

}

/**
 * @brief do_monitor process monitor board sensors information
 *
 * @param pkg
 *
 * @return
 */

int PowerManager::do_monitor(CanPkg *pkg)
{
    Json::FastWriter fw;
    Json::Value root;
    atris_msgs::RobotInfo rbtInfo;
    double curTime = ros::Time::now().toSec();
    //printf("receive power manager data\r\n");
    if(pkg->head.id.channel == CH_PM_MONITOR){
        
        unsigned char cmd = pkg->data[0];
        if((cmd == PM_CMD_BUMPER_STATE) && ((bumper_data !=pkg->data[1]) ||
                                            (bumper_data1 !=pkg->data[2]) ||
                                            (bumper_data2 !=pkg->data[3]) ||
                                            (bumper_data3 !=pkg->data[4]) ||
                                            (bumper_data4 !=pkg->data[5]))){
            bumper_data =pkg->data[1];
            int value = pkg->data[1];
            //int bumper_type = pkg->data[2];
            int AntiClosedStatus = pkg->data[3];
            int AntiOpenedStatus = pkg->data[4];
            int E_StopStatus = pkg->data[5];
            int bumper_type = pkg->data[6];
            double time = ros::Time::now().toSec() - brake_time.toSec() ;

            if(time > 5){
                if((bumper_type & 1) == DF_NORMALLY_OPEN_BUMPER){ //前防撞条常开式
                    root["robot_info"]["brake"]["front_bumper_type"] = "normally_open";
                    root["robot_info"]["brake"]["front_bumper"] = AntiOpenedStatus & 1;
                }else if((bumper_type & 1)  == DF_NORMALLY_CLOSED_BUMPER){//前防撞条常闭式
                   root["robot_info"]["brake"]["front_bumper_type"] = "normally_closed";
                   root["robot_info"]["brake"]["front_bumper"] = AntiClosedStatus & 3;
                }

                if(((bumper_type >> 1) & 1) == DF_NORMALLY_OPEN_BUMPER){//后防撞条常开式
                    root["robot_info"]["brake"]["tear_bumper_type"] = "normally_open";
                    root["robot_info"]["brake"]["tear_bumper"] = (AntiOpenedStatus >> 1) & 1;
                }else if(((bumper_type >> 1) & 1)  == DF_NORMALLY_CLOSED_BUMPER){//后防撞条常闭式
                   root["robot_info"]["brake"]["tear_bumper_type"] = "normally_closed";
                   root["robot_info"]["brake"]["tear_bumper"] = (AntiClosedStatus >> 2) & 3;
                }
                
#ifdef _CHASSIS_MARSHELL_
                root["robot_info"]["brake"]["button"] = E_StopStatus;
#endif
                rbtInfo.json = fw.write(root);
                diag_info_pub_.publish(rbtInfo);
            }

        }else if((cmd == PM_CMD_MIDDLE_BRAKE) && ((brake_data1 != pkg->data[1]) || (brake_data2 != pkg->data[2]))){
            brake_data1 = pkg->data[1];
            brake_data2 = pkg->data[2];

            {
                Json::Value root_control;
                atris_msgs::RobotInfo rbtInfo_control;  
                Json::FastWriter fw_control;
                root_control["robot_info"]["brake"]["control"]["CMotor_brake"] = (pkg->data[1] & 1) ? 1 : 0;
                root_control["robot_info"]["brake"]["control"]["CEmergency_brake"] = (pkg->data[1] & 2) ? 1 : 0;
                root_control["robot_info"]["brake"]["control"]["W_software_brake"] = (pkg->data[1] & 4) ? 1 : 0;

#ifdef _CHASSIS_MARSHELL_
                if(pkg->data[1] & 0x7){
                   // ChassisTracked::get_instance()->clean_chassis_status();

                }
#endif
  
  
                root_control["robot_info"]["brake"]["source"]["reboot"] = (pkg->data[2] & 1) ? 1 : 0;                
                root_control["robot_info"]["brake"]["source"]["front_bumper"] = (pkg->data[2] & 2) ? 1 : 0;
                root_control["robot_info"]["brake"]["source"]["tear_bumper"] = (pkg->data[2] & 0x80) ? 1 : 0;            
                root_control["robot_info"]["brake"]["source"]["charge_bumper"] = (pkg->data[2] & 4) ? 1 : 0;
                root_control["robot_info"]["brake"]["source"]["charge"] = (pkg->data[2] & 8) ? 1 : 0;                
                root_control["robot_info"]["brake"]["source"]["can"] = (pkg->data[2] & 0x10) ? 1 : 0;
                root_control["robot_info"]["brake"]["source"]["iap"] = (pkg->data[2] & 0x20) ? 1 : 0;                
                root_control["robot_info"]["brake"]["source"]["button"] = (pkg->data[2] & 0x40) ? 1 : 0;    
                rbtInfo_control.json = fw.write(root_control);
                diag_info_pub_.publish(rbtInfo_control);
            }

            log_info("%s  chassis manager receive brake:%d,from:%d", __FUNCTION__,pkg->data[1],pkg->data[2]);
            if(pkg->data[1] != 0){
               // ChassisTracked::get_instance()->brake(BRAKE_ON);
            }
        }else if(cmd == PM_CMD_SENSOR_LIQUID) {
#ifdef _CHASSIS_JC_
            root["robot_info"]["sensor_liquid"]["status"] = pkg->data[1];
            rbtInfo.json = fw.write(root);
            diag_info_pub_.publish(rbtInfo);
#endif
        } else if(cmd == PM_CMD_SENSOR_HUMIDITY){
            root["robot_info"]["temp_humi"]["temp_env"] = (((int16_t)(pkg->data[1] | pkg->data[2] << 8)) - 300) * 0.1;
            root["robot_info"]["temp_humi"]["humi_env"] = (pkg->data[3] | pkg->data[4] << 8) * 0.1;
            rbtInfo.json = fw.write(root);
            diag_info_pub_.publish(rbtInfo);
        } else if(cmd == PM_CMD_SENSOR_HOST_TMP){
            root["robot_info"]["temp_humi"]["motor_left"] = (int8_t)pkg->data[1];
            root["robot_info"]["temp_humi"]["motor_right"] = (int8_t)pkg->data[2];
            root["robot_info"]["temp_humi"]["12v"] = (int8_t)pkg->data[3];
            root["robot_info"]["temp_humi"]["24v"] = (int8_t)pkg->data[4];
            rbtInfo.json = fw.write(root);
            diag_info_pub_.publish(rbtInfo);
        } else if(cmd == PM_CMD_SENSOR_HALL_CURRENT){
            root["robot_info"]["sensor_hall"]["hall1"] = (int8_t)pkg->data[1];
            root["robot_info"]["sensor_hall"]["hall2"] = (int8_t)pkg->data[2];
#if 0
            root["robot_info"]["sensor_hall"]["hall3"] = (int8_t)pkg->data[3];
#endif
            rbtInfo.json = fw.write(root);
            diag_info_pub_.publish(rbtInfo);
        } else if(cmd == PM_INFO_BOTTOM_FAN){
            root["robot_info"]["fan"]["bottom"]["speed_in"] = pkg->data[1];
            root["robot_info"]["fan"]["bottom"]["speed_out"] = pkg->data[1];
            
#ifdef _CHASSIS_MARSHELL_
            root["robot_info"]["fan"]["bottom"]["error"] = pkg->data[2] & 0x04;
#else
            root["robot_info"]["fan"]["bottom"]["error"] = pkg->data[2];
#endif
            rbtInfo.json = fw.write(root);
            diag_info_pub_.publish(rbtInfo);
        }else if(cmd == PM_INFO_REMOTE_CONTROL_MODE){
            switch(pkg->data[1]){
                case 0x00:
                    if(remote_control_mode != SAFE_MODE){
                        remote_control_mode = SAFE_MODE;
                        log_info("--------switch to safe mode!!!");
                        sendTtstext(TTSStrings::TTS_KEY_SAFE_MODE);
                        //sendTtsTextForce(TTSStrings::TTS_KEY_SAFE_MODE);
                    }
                    break;
                case 0x01:
                    if(remote_control_mode != STD_MODE){
                        remote_control_mode = STD_MODE;
                        log_info("--------switch to standard mode!!!");
                        sendTtstext(TTSStrings::TTS_KEY_STD_MODE);
                        //sendTtsTextForce(TTSStrings::TTS_KEY_STD_MODE);
                    }
                    break;
                case 0x02:
                    if(remote_control_mode != EMC_MODE){
                        remote_control_mode = EMC_MODE;
                        log_info("--------switch to emergency mode!!!");
                        sendTtstext(TTSStrings::TTS_KEY_EMC_MODE);
                        //sendTtsTextForce(TTSStrings::TTS_KEY_EMC_MODE);
                    }
                    break;
                default:
                    log_info("--------erro mode!!!");

                    break;
            }
        } else if((cmd == PM_INFO_ELECTRODES) || (cmd == PM_INFO_BLUETOOTH) ||
                  (cmd == PM_INFO_CHARGE_STATE) || (cmd == PM_INFO_PC_LEAVE_PILE) || (cmd == PM_INFO_DOCK_LEAVE_PILE)){
            atris_msgs::ChargeInfo msg;
            msg.cmd = cmd;
            for(int i=0; i<7; i++){
                msg.data[i] = pkg->data[i+1];
            }
            charge_info_pub_.publish(msg);
            if (cmd == PM_INFO_ELECTRODES) {
                root["robot_info"]["charge"]["electrodes_status"] = pkg->data[1];
                root["robot_info"]["charge"]["electrodes_voltage"] = pkg->data[2] | pkg->data[3] << 8;
            } else if (cmd == PM_INFO_BLUETOOTH) {
                root["robot_info"]["charge"]["bluetooth"] = pkg->data[1];
            }
            rbtInfo.json = fw.write(root);
            diag_info_pub_.publish(rbtInfo);
        } else if(cmd == PM_CMD_REMOTE_CONTROLLER_SPEED){

             int tem1 = pkg->data[1] | (pkg->data[1+1] & 0x00FF) << 8;
             int tem2 = pkg->data[3] | (pkg->data[3+1] & 0x00FF) << 8;
             ch_2 = tem1;
             ch_4 = tem2;
             ch_1 = pkg->data[6] | (pkg->data[6+1] & 0x00FF) << 8;
             ch_flag =  pkg->data[5];


             if((pkg->head.id.size == 4) || ((pkg->data[5] == 'p') && (pkg->data[6] == 'w') && (pkg->data[7] == 'm'))){
                  send_twish(ch_2,ch_4,REMOTE_CTRL_PWM_CENTER,REMOTE_CTRL_PWM_MAX_VALUE);

             }else{
                  #if 0
                  log_info("%s off_line:%d,ch1:%d,ch2:%d,ch3:%d,ch4:%d,ch5:%d,ch6:%d,ch7:%d,ch8:%d,flag:%.2x",__FUNCTION__,remote_status,
                               ch_1,
                               ch_2,
                               ch_3,
                               ch_4,
                               ch_5,
                               ch_6,
                               ch_7,
                               ch_8,
                               ch_flag);
                 #endif
                 if(ch_flag & 0xc){
                     if(remote_status != DF_REMOTE_OFF_LINE){
                        remote_status = DF_REMOTE_OFF_LINE;
                        log_info("=======================%s Remote Controller power off", __FUNCTION__);
                        atris_msgs::AisoundTTS tts_msg;
                        tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
                        tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_TELECONTROL_POW_OFF);
                        aisound_tts_pub_.publish(tts_msg);
                        send_twish(REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_MAX_VALUE);
                    }
                     return 0;
                 }else if(remote_status == DF_REMOTE_OFF_LINE){
                     if(remote_status != DF_REMOTE_ON){
                         remote_status = DF_REMOTE_ON;
                         atris_msgs::AisoundTTS tts_msg;
                         tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
                         tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_TELECONTROL_POW_ON);
                         aisound_tts_pub_.publish(tts_msg);
                     }
                  }

                 if((ch_7 > (REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) && (remote_status == DF_REMOTE_ON)){
                     remote_status = DF_REMOTE_ON_LINE;
                    log_info("=============%s Remote Controller power on", __FUNCTION__);
                    send_twish(REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_MAX_VALUE);
					
                    atris_msgs::AisoundTTS tts_msg;
                    tts_msg.clear = true;
                    tts_msg.key = TTSStrings::TTS_KEY_TELECONTROL_LOCKING;
                    tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_MID;
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_TELECONTROL_LOCKING);
                    aisound_tts_pub_.publish(tts_msg);
                 }

                 if((ch_7 < ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) && ((remote_status == DF_REMOTE_ON_LINE) || (remote_status == DF_REMOTE_ROCKER_NOT_ON_CENTER))){
                        if( (abs(ch_2 -REMOTE_CTRL_SBUS_CENTER) < REMOTE_CTRL_ABNOR_VALUL)
                                && (abs(ch_4 -REMOTE_CTRL_SBUS_CENTER) < REMOTE_CTRL_ABNOR_VALUL)){
                            remote_status = DF_REMOTE_CONTRUL;

                            log_info("==================%s Remote Controller unlock", __FUNCTION__);
                            atris_msgs::AisoundTTS tts_msg;
                            tts_msg.clear = true;
                            tts_msg.key = TTSStrings::TTS_KEY_TELECONTROL_UNLOCK;
                            tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_MID;
                            tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_TELECONTROL_UNLOCK);
                            aisound_tts_pub_.publish(tts_msg);
                        }else if(remote_status !=  DF_REMOTE_ROCKER_NOT_ON_CENTER){
                           remote_status =  DF_REMOTE_ROCKER_NOT_ON_CENTER;
                           send_twish(REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_MAX_VALUE);
                           log_info("=================%s Please center the rocker of the remote!", __FUNCTION__);
                           //sendTtstext(TTSStrings::TTS_KEY_ROCKER_BACK);
                           
                            atris_msgs::AisoundTTS tts_msg;
                            tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
                            tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_ROCKER_BACK);
                            aisound_tts_pub_.publish(tts_msg);
                        }

                 }else if((ch_7 > ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) && (remote_status == DF_REMOTE_CONTRUL)){
                    remote_status = DF_REMOTE_ON_LINE;
                    send_twish(REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_MAX_VALUE);
                    log_info("==================%s Remote Controller lock!!!\r\n",__FUNCTION__);
                    atris_msgs::AisoundTTS tts_msg;
                    tts_msg.clear = true;
                    tts_msg.key = TTSStrings::TTS_KEY_TELECONTROL_LOCKING;
                    tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_MID;
                    tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_TELECONTROL_LOCKING);
                    aisound_tts_pub_.publish(tts_msg);
                 }


                 if(remote_status == DF_REMOTE_CONTRUL){
                    {
                         if((ch_5 < ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) &&  !(brake_state < (REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) ){
                             log_info("===================%s,remote brake off",__FUNCTION__);
                             brake_time = ros::Time::now();
                             brake(BRAKE_OFF);
					
                             atris_msgs::AisoundTTS tts_msg;
                             tts_msg.clear = true;
                             tts_msg.priority = atris_msgs::AisoundTTS::PRIORITY_LOW;
                             tts_msg.text = TTSStrings::text(TTSStrings::TTS_KEY_CHASSIS_UNLOCKING);
                             aisound_tts_pub_.publish(tts_msg);
                         }

                         if((ch_6 < ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) &&  !(light_state < (REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) ){
                             log_info("==============%s,remote lamp lignt on",__FUNCTION__);
                             //FlashLamp::get_instance()->set_white_lamp_status(FL_ON);
                             atris_msgs::LampCmd msg;
                             msg.type = atris_msgs::LampCmd::LAMP_TYPE_WHITE;
                             msg.status = atris_msgs::LampCmd::STATUS_ON;
                             msg.source = atris_msgs::LampCmd::LAMP_SOURCE_REMOTE;
                             lamp_cmd_pub_.publish(msg);

                             light_state = ch_6;
                         }
                         if((ch_6 > ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) &&  !(light_state > (REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) ){
                             log_info("===============%s,remote lamp lignt off",__FUNCTION__);
                             //FlashLamp::get_instance()->set_white_lamp_status(FL_OFF);
                             atris_msgs::LampCmd msg;
                             msg.type = atris_msgs::LampCmd::LAMP_TYPE_WHITE;
                             msg.status = atris_msgs::LampCmd::STATUS_OFF;
                             msg.source = atris_msgs::LampCmd::LAMP_SOURCE_REMOTE;
                             lamp_cmd_pub_.publish(msg);

                             light_state = ch_6;
                         }

                         if((ch_8 < ( REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) &&  !(flash_light_state < (REMOTE_CTRL_SBUS_CENTER - 4 * REMOTE_CTRL_ABNOR_VALUL)) ){
                             log_info("=================%s,remote flash lamp on",__FUNCTION__);
                             //FlashLamp::get_instance()->set_red_blue_flash_status(FL_ON);
                             atris_msgs::LampCmd msg;
                             msg.type = atris_msgs::LampCmd::LAMP_TYPE_FLASH;
                             msg.status = atris_msgs::LampCmd::STATUS_ON;
                             msg.source = atris_msgs::LampCmd::LAMP_SOURCE_REMOTE;
                             lamp_cmd_pub_.publish(msg);

                             flash_light_state = ch_8;
                         }
                         if((ch_8 > ( REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) &&  !(flash_light_state > (REMOTE_CTRL_SBUS_CENTER + 4 * REMOTE_CTRL_ABNOR_VALUL)) ){
                             log_info("=============================%s,remote flash lamp  off",__FUNCTION__);
                             //FlashLamp::get_instance()->set_red_blue_flash_status(FL_OFF);
                             atris_msgs::LampCmd msg;
                             msg.type = atris_msgs::LampCmd::LAMP_TYPE_FLASH;
                             msg.status = atris_msgs::LampCmd::STATUS_OFF;
                             msg.source = atris_msgs::LampCmd::LAMP_SOURCE_REMOTE;
                             lamp_cmd_pub_.publish(msg);

                             flash_light_state = ch_8;

                         }

                         brake_state = ch_5;

                     }





                     send_twish(ch_2,ch_4,REMOTE_CTRL_SBUS_CENTER,REMOTE_CTRL_SBUS_MAX_VALUE);
 
                }

             }
        }else if (cmd == PM_CMD_REMOTE_SBUS_356) {
            ch_3  = pkg->data[1] | (pkg->data[1+1] & 0x00FF) << 8;
            ch_5  = pkg->data[3] | (pkg->data[3+1] & 0x00FF) << 8;
            ch_6  = pkg->data[5] | (pkg->data[5+1] & 0x00FF) << 8;
        } else if (cmd == PM_CMD_REMOTE_SBUS_78) {
            ch_7  =  pkg->data[1] | (pkg->data[1+1] & 0x00FF) << 8;
            ch_8  =  pkg->data[3] | (pkg->data[3+1] & 0x00FF) << 8;
        } else if (cmd == PM_CMD_SYSTEM_TIME) {
            time_t now =time(NULL);
            struct tm* ptm = localtime(&now);
            if (ptm) {
                CanPkg package = {0};
                package.head.id.channel = CH_PM_MONITOR;
                package.data[0] = PM_INFO_SYSTEM_TIME;
                ptm->tm_year = ptm->tm_year + 1900;
                package.data[1] = (unsigned char)(ptm->tm_year >> 0 & 0xFF);
                package.data[2] = (unsigned char)(ptm->tm_year >> 8 & 0xFF);
                ptm->tm_mon = ptm->tm_mon + 1;
                package.data[3] = (unsigned char)(ptm->tm_mon  >> 0 & 0xFF);
                package.data[4] = (unsigned char)(ptm->tm_mday >> 0 & 0xFF);
                package.data[5] = (unsigned char)(ptm->tm_hour >> 0 & 0xFF);
                package.data[6] = (unsigned char)(ptm->tm_min  >> 0 & 0xFF);
                package.data[7] = (unsigned char)(ptm->tm_sec  >> 0 & 0xFF);
                CanService::get_instance()->send(package, 8);
            }
        }
        return 0;
    } else if(pkg->head.id.channel == CH_PM_CTRL){

    }

    return -1;
}

void PowerManager::sendTtstext(int key,int ext_key)
{

    if((tts_key_ == key) && (tts_ext_key_ == ext_key))
       return;
    log_info("%s  key:%d,ext_key:%d",__FUNCTION__,key,ext_key);

    tts_key_ = key;
    tts_ext_key_ = ext_key;
    atris_msgs::AisoundTTS msg;
    msg.key = key;
    msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
    msg.text = TTSStrings::text(key);
    if(ext_key != TTSStrings::TTS_KEY_NULL)
        msg.text += TTSStrings::text(ext_key);
    log_info("%s key:%d,ext_key:%d,%s",__FUNCTION__,key,ext_key,msg.text.c_str());
    aisound_tts_pub_.publish(msg);
}

void PowerManager::sendTtsTextForce(int key)
{
    log_warn("%s sending tts msg",__FUNCTION__);
    atris_msgs::AisoundTTS msg;
    msg.key = key;
    msg.priority = atris_msgs::AisoundTTS::PRIORITY_HIGH;
    msg.text = TTSStrings::text(key);
    aisound_tts_pub_.publish(msg);
}

bool PowerManager::send_twish(int pwm_L,int pwm_R,int center,int max,bool send)
{
    geometry_msgs::Twist msg;


    static double remote_time = -1.0f;
    if(remote_time < 0.0f)
        remote_time = ros::Time::now().toSec();
    double sec =ros::Time::now().toSec() - remote_time;
    if((sec < DF_REMOTE_CYCLE) && (sec > 0.0f))
        return false;
    remote_time = ros::Time::now().toSec();

    log_once_info("%s pwm_L:%d pwm_R:%d,center:%d,max:%d",__FUNCTION__,pwm_L,pwm_R,center,max);

    if((pwm_R < (center - max - 2 * REMOTE_CTRL_ABNOR_VALUL)) || (pwm_L < (center - max - 2 * REMOTE_CTRL_ABNOR_VALUL))){
        log_info("%s remotedatat error (%d - %d - %d) = %d pwm_L:%d ,pwm_R:%d",center,max,2 * REMOTE_CTRL_ABNOR_VALUL,center - max -2 *  REMOTE_CTRL_ABNOR_VALUL,pwm_L,pwm_R);
        return false;
    }

    if((pwm_R > (2 * REMOTE_CTRL_ABNOR_VALUL + max + center)) || pwm_L > ((2 * REMOTE_CTRL_ABNOR_VALUL + max + center))){
        log_info("(%d + %d + %d) = %d pwm_L:%d ,pwm_R:%d",center,max,2 * REMOTE_CTRL_ABNOR_VALUL,center + max + 2 * REMOTE_CTRL_ABNOR_VALUL,pwm_L,pwm_R);
        return false;
    }

    if(pwm_R > (max + center))
        pwm_R = (max + center);

    if(pwm_L > (max + center))
        pwm_L = (max + center);


    if(pwm_R < (center - max))
        pwm_R = (center - max);

    if(pwm_L < (center - max))
        pwm_L = (center - max);

    float temf1 = CALC_RIGHT_LINE_SPEED(pwm_L,center,max);
    float temf2 = CALC_LEFT_LINE_SPEED(pwm_R,center,max);

    float line_speed = (temf1 + temf2)/2;
    float angle_speed = (temf1 - temf2)/WHELL_DISTANCE;
   //log_error("---pwm_L:%d pwm_R:%d--------recv_pwm,line:%d,angle:%d",pwm_L,pwm_R,pwm_L,pwm_R);
    if(line_speed > 0)
       line_speed = line_speed * Config::get_instance()->forward_wheel_max;
    else
        line_speed = line_speed * Config::get_instance()->backward_wheel_max;
    msg.linear.x =  line_speed;
    #ifdef _CHASSIS_MARSHELL_
      // msg.angular.z = angle_speed / 4.5;
       msg.angular.z = (temf1 - temf2) * Config::get_instance()->angular_wheel_max / 2;
    #else
       msg.angular.z = angle_speed;
    #endif
    if(send){
      //  ChassisManager::get_instance()->on_recv_twist(msg);
        return true;
    }else{
        if(((fabs(msg.linear.x) < 0.15f) &&  (fabs(msg.angular.z) < 0.1f))){
           return false;
        } else{
            return true;
        }

    }

}

void PowerManager::proc_poweroff(int status)
{
    boost::shared_ptr<boost::thread> poweroff_thread(new boost::thread(
                boost::bind(&PowerManager::proc_poweroff_thread, this, status)));
}

void PowerManager::notifyPowerOffEvent(std::string serial_num, Json::Value & reason)
{
    ros::Time now = ros::Time::now();
    Json::Value content;
    Json::Value evt_arr;
    Json::Value power_off_event_msg;
    Json::FastWriter writer;
    std::string event_content = "";

    shm::Robot shmrbt;
    shm::iMemory_read_Robot(&shmrbt);
    std::string accid = std::string(shmrbt.robot.accid);

    // construct json info
    content["category"] = "custom_event"; // custom event

    // eventId according to message type
    content["eventId"] = "shut_down_event";

    // accid
    content["userId"] = accid;
    // timestamp
    content["recordedAt"] = (uint64_t)(now.toSec());

    content["segmentation"] = "";

    // reason
    content["customSegmentation"]["reason"] = reason;

    content["customSegmentation"]["serialNum"] = serial_num;

    // get gps position
    content["customSegmentation"]["lati"] = "";
    content["customSegmentation"]["longi"] = "";
    atris_msgs::GetGpsPos gpspos;
    if (get_gps_position_srv_client_.call(gpspos)) 
    {
        if(gpspos.response.status == 0)
        {
            content["customSegmentation"]["lati"] = gpspos.response.latitude;
            content["customSegmentation"]["longi"] = gpspos.response.longitude;
        }
    } 
    else 
    {
        log_warn("%s get_gps_position_srv_client_ call fail.", __FUNCTION__);
        //return;
    }

    evt_arr.append(content);
    event_content = writer.write(evt_arr);
    log_info("%s event_content is %s",__FUNCTION__, event_content.c_str());

    power_off_event_msg["event_message"] = evt_arr;

    Utils::get_instance()->NotifyRobotStatus("notify_shut_down_event", power_off_event_msg);
}

void PowerManager::reportPowerOffEventToPc(int val)
{
    atris_msgs::SignalMessage diag_msg;
    Json::Value content;
    Json::Value evt_arr;
    Json::Value evt_arr_content;
    std::string event_content = "";
    Json::FastWriter writer;

    content["result"] = "success";
    content["result_code"] = 0;

    evt_arr_content["shutdown"] = val;

    evt_arr.append(evt_arr_content);
    
    content["events"] = evt_arr;
    event_content = writer.write(content);
    log_warn("%s event_content : %s",__FUNCTION__, event_content.c_str());

    Utils::get_instance()->NotifyRobotStatus("notify_event", content, "");
}

void PowerManager::proc_poweroff_thread(int status)
{
    if (!is_power_off_) {
      is_power_off_ = true;
    } else {
      return;
    }
    
    log_info("%s start do poweroff process.", __FUNCTION__);
    Utils *utils = Utils::get_instance();
    bool face_state = false, nav_state = false;
    Json::Value root;
    char rand_str[16] = {0};
    Utils::get_instance()->getRandStr(rand_str);

    power_off_serial_num_ = rand_str;
    root["type"] = "code";
    root["value"] = status;
    notifyPowerOffEvent(power_off_serial_num_, root);
    reportPowerOffEventToPc(status);

    power_off_face_x86();
    power_off_nav_x86();

    double time = ros::Time::now().toSec();
    double duration = 40;
    while(1){
        if(!face_state){
            if(utils->check_network_state(Config::get_instance()->gs_ip.c_str())){
                log_info("%s gs device reachable.", __FUNCTION__);
                sleep(1);
            } else {
                log_info("%s gs device unreachable.", __FUNCTION__);
                face_state = true;
            }
        }

        if(!nav_state){
            if(utils->check_network_state(Config::get_instance()->ptz_box_ip.c_str())){
                log_info("%s face x86 device reachable.", __FUNCTION__);
            } else {
                log_info("%s face x86 device unreachable.", __FUNCTION__);
                nav_state = true;
            }
        }

        if(face_state && nav_state) {
            break;
        }
        
        double now = ros::Time::now().toSec();
        if (now < time) {
            time = now;
        }
        if (((now - time) > duration)) {
            log_warn("%s Waitting timeout.", __FUNCTION__);
            break;
        } else {
            sleep(1);
        }
    }

    // close speaker
    FILE *fp = NULL;
    if((fp = popen("echo 0 > /sys/class/gpio/gpio66/value; "
      "echo 0 > /sys/class/gpio/gpio120/value", "r"))) {
      log_info("%s close speaker success.", __FUNCTION__);
      pclose(fp);
    }
    
    send_poweroff_finish(status);

    // status: 0 shutdown
    // status: 1 charged now
    log_info("%s poweroff: %d", __FUNCTION__, status);
    if (status == 0) {
      log_warn("\n\n%s shutdown now ^_^\n\n", __FUNCTION__);
      if((fp = popen("tinyrosshutdown all; shutdown -h now", "r"))) {
        pclose(fp);
      }
    } else {
      log_warn("\n\n%s Udock charged now ^_^\n\n", __FUNCTION__);
      is_power_off_ = false;
    }
}



int PowerManager::send_poweroff_finish(int status)
{
    int ret;

    boost::unique_lock<boost::mutex> wlock(poff_mutex);

    CanPkg pkg;
    pkg.head.id.channel = CH_PM_CTRL;
    pkg.data[0] = PM_CMD_POWEROFF_FIN;
    pkg.data[1] = status;
   if(can && can_send)
        can->can_write(&pkg, 2);

    if(poff_cv.timed_wait(wlock, boost::get_system_time() + boost::posix_time::seconds(2))){
        ret = 0;
    }
    else
        ret = -1;

    return ret;
}

/**
 * @brief power_off_face_x86 关闭旷世工控机
 *
 * @return
 */
int PowerManager::power_off_face_x86()
{
    log_info("power_off_face_x86");
    Json::FastWriter fw;
    Json::Value root;
    atris_msgs::RobotInfo rbtInfo;
    root["robot_info"]["power"]["imx"] = 0;
    rbtInfo.json = fw.write(root);
    diag_info_pub_.publish(rbtInfo);
    return 0;
}

/**
 * @brief power_off_nav_x86 关闭导航工控机
 *
 * @return
 */
int PowerManager::power_off_nav_x86()
{
    log_info("power_off_nav_x86");
    atris_msgs::NavReset nav_reset_msg;
    nav_reset_msg.reset = atris_msgs::NavReset::NAV_POWER_OFF;
    nav_reset_pub_.publish(nav_reset_msg);
    return 0;
}

void PowerManager::on_power_off(const std_msgs::Int32 &msg) {
    proc_poweroff(msg.data);
}

