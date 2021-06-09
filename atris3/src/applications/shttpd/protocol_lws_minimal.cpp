/*
 * ws protocol handler plugin for "lws-minimal" demonstrating multithread
 *
 * Written in 2010-2019 by Andy Green <andy@warmcat.com>
 *
 * This file is made available under the Creative Commons CC0 1.0
 * Universal Public Domain Dedication.
 */
#include <sys/types.h>
#include <sys/select.h>
#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <time.h>
#include <thread>
#include <libwebsockets.h>
#include "shttpd.h"
#include <pthread.h>
#include <signal.h>
#include "ros/ros.h"
#include "tiny_ros/ros.h"
#include <json/json.h>
#include <string>
#include "utils/utils.h"
#include "config/config.h"
#include "database/sqliteengine.h"
#include "imemory/atris_imemory_api.h"
#include "atris_defines.h"
#include "log/log.h"
#include "atris_msgs/SignalMessage.h"

static pthread_mutex_t lock_lws_vhd = PTHREAD_MUTEX_INITIALIZER; 

static struct per_vhost_data__minimal *lws_vhd = NULL;

static ros::Subscriber shttpd_res_sub;

static ros::Subscriber shttpd_diag_sub;

static ros::Publisher shttpd_req_pub;


/* one of these created for each message in the ringbuffer */

struct msg {
  unsigned char * payload; /* is malloc'd */
  size_t len;
};

/*
 * One of these is created for each client connecting to us.
 *
 * It is ONLY read or written from the lws service thread context.
 */

struct per_session_data__minimal {
  struct per_session_data__minimal *pss_list;
  struct lws *wsi;
  uint32_t tail;
  char kickout;
};

/* one of these is created for each vhost our protocol is used with */

struct per_vhost_data__minimal {
  struct lws_context *context;
  struct lws_vhost *vhost;
  const struct lws_protocols *protocol;

  struct per_session_data__minimal *pss_list; /* linked-list of live pss*/

  pthread_mutex_t lock_ring; /* serialize access to the ring buffer */
  struct lws_ring *ring; /* {lock_ring} ringbuffer holding unsent content */

  char finished;
};

/*
 * This runs under both lws service and "spam threads" contexts.
 * Access is serialized by lws_vhd->lock_ring.
 */

static void
__minimal_destroy_message(void *_msg)
{
  struct msg *msg = (struct msg *)_msg;

  free(msg->payload);
  msg->payload = NULL;
  msg->len = 0;
}

static Json::Value shttpdResponseGlobalInit(const atris_msgs::SignalMessage& msg) {
  Json::Value reponse;
  ros::Time now = ros::Time::now();
  shm::Robot shmrbt;
  shm::iMemory_read_Robot(&shmrbt);
  reponse["title"] = "response" + msg.title.substr(7);
  reponse["accid"] = shmrbt.robot.sn;
  reponse["content"]["id"] = msg.msgID;
  reponse["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000);
  reponse["content"]["result"] = "success";

  /* global params*/
  std::string global_ntp_server_list = "";
  for (std::size_t i=0; i<Config::get_instance()->ntp_server_list.size(); i++) {
    if (i == (Config::get_instance()->ntp_server_list.size() -1)) {
      global_ntp_server_list += Config::get_instance()->ntp_server_list.at(i);
    } else {
      global_ntp_server_list += Config::get_instance()->ntp_server_list.at(i) + ";";
    }
  }
  reponse["content"]["global_ntp_server_list"] = global_ntp_server_list;
  reponse["content"]["global_volume"] = (shmrbt.appdata.volume >= 0 && shmrbt.appdata.volume <= 100) ? shmrbt.appdata.volume : (shmrbt.appdata.volume > 0 ? 100 : 0);
  reponse["content"]["global_dock_wall_width"] = Config::get_instance()->dock_wall_width;
  reponse["content"]["global_ptz_ip"] = Config::get_instance()->ptz_ip;
  reponse["content"]["global_ptz_box_ip"] = Config::get_instance()->ptz_box_ip;
  reponse["content"]["global_nvr_ip"] = Config::get_instance()->nvr_ip;
  reponse["content"]["global_mqtt_host"] = Config::get_instance()->mqtt_host_;
  reponse["content"]["global_mqtt_port"] = Config::get_instance()->mqtt_port_;
  reponse["content"]["global_mqtt_username"] = Config::get_instance()->mqtt_username_;
  reponse["content"]["global_mqtt_password"] = Config::get_instance()->mqtt_password_;
  reponse["content"]["global_mqtt_encrypt"] = Config::get_instance()->mqtt_encrypt_;
  reponse["content"]["global_mqtt_keepalive"] = Config::get_instance()->mqtt_keepalive_;
  reponse["content"]["global_remote_mqtt_topic_url"] = Config::get_instance()->remote_mqtt_topic_url_;
  reponse["content"]["global_remote_mqtt_host"] = Config::get_instance()->remote_mqtt_host_;
  reponse["content"]["global_remote_mqtt_port"] = Config::get_instance()->remote_mqtt_port_;
  reponse["content"]["global_remote_mqtt_username"] = Config::get_instance()->remote_mqtt_username_;
  reponse["content"]["global_remote_mqtt_password"] = Config::get_instance()->remote_mqtt_password_;
  reponse["content"]["global_remote_mqtt_encrypt"] = Config::get_instance()->remote_mqtt_encrypt_;
  reponse["content"]["global_remote_mqtt_keepalive"] = Config::get_instance()->remote_mqtt_keepalive_;
  reponse["content"]["global_router_type"] = Config::get_instance()->router_type;
  reponse["content"]["global_router_ip"] = Config::get_instance()->router_ip;
  reponse["content"]["global_router_user"] = Config::get_instance()->router_user;
  reponse["content"]["global_router_psw"] = Config::get_instance()->router_psw;
  reponse["content"]["global_abi_base_url"] = Config::get_instance()->base_url;
  reponse["content"]["global_abi_robotinfo_url"] = Config::get_instance()->robotinfo_url;
  reponse["content"]["global_abi_timestamp_url"] = Config::get_instance()->timestamp_url;
  reponse["content"]["global_abi_post_event_url"] = Config::get_instance()->post_event_url;
  reponse["content"]["global_abi_enable"] = Config::get_instance()->abi_enable ? 1 : 0;
  reponse["content"]["global_hfs_url"] = Config::get_instance()->hfs_url;
  reponse["content"]["global_hfs_type"] = Config::get_instance()->hfs_type;
  reponse["content"]["global_timezone"] = Config::get_instance()->timezone;
  reponse["content"]["global_voip_auto_setting"] = Config::get_instance()->voip_auto_setting ? 1 : 0;
  reponse["content"]["global_voip_sip_server"] =  Config::get_instance()->voip_sip_server;
  reponse["content"]["global_nav_obs_mode"] =  Config::get_instance()->nav_obs_mode;
  reponse["content"]["global_nav_cycle_enable"] =  Config::get_instance()->nav_cycle_enable ? 1 : 0;
  reponse["content"]["global_nav_cycle_dst"] =  Config::get_instance()->nav_cycle_dst;
  reponse["content"]["global_nav_without_charge_path"] =  Config::get_instance()->nav_without_charge_path ? 1 : 0;
  reponse["content"]["global_nav_without_merge_path"] =  Config::get_instance()->nav_without_merge_path ? 1 : 0;
  reponse["content"]["global_nav_task_start_timeout"] =  Config::get_instance()->nav_task_start_timeout;
  reponse["content"]["global_nav_task_running_timeout"] =  Config::get_instance()->nav_task_running_timeout;
  reponse["content"]["global_nav_auto_charge_self"] =  Config::get_instance()->nav_auto_charge_self ? 1 : 0;
  reponse["content"]["global_nav_anti_drop_enable"] =  Config::get_instance()->nav_anti_drop_enable ? 1 : 0;
  reponse["content"]["global_nav_tsp_enable"] =  Config::get_instance()->nav_tsp_enable ? 1 : 0;
  reponse["content"]["global_nav_auto_charge_battery"] =  Config::get_instance()->nav_auto_charge_battery;
  reponse["content"]["global_red_blue_light_imsi_detection"] =  Config::get_instance()->open_red_blue_light_imsi_detection ? 1 : 0;
  reponse["content"]["global_vision_host"] = Config::get_instance()->vision_host_;
  reponse["content"]["global_vision_port"] = Config::get_instance()->vision_port_;
  reponse["content"]["global_obstacle_mode"] = Config::get_instance()->obstacle_mode_;
  reponse["content"]["global_obstacle_timeout"] = Config::get_instance()->obstacle_timeout_;
  reponse["content"]["global_obstacle_timeout_strategy"] = Config::get_instance()->obstacle_timeout_strategy_;
  return reponse;
}

static void shttpdResponseCb(const atris_msgs::SignalMessage& received_msg) {
  struct msg amsg;
  int n = 0;
  if (received_msg.type == "shttpd" || received_msg.type.empty()) {     
    /* don't generate output if nobody connected */
    pthread_mutex_lock(&lock_lws_vhd); /* --------- lock_lws_vhd { */
    if (lws_vhd && !lws_vhd->finished && lws_vhd->pss_list) {
      pthread_mutex_lock(&lws_vhd->lock_ring); /* --------- ring lock { */
      /* only create if space in ringbuffer */
      n = (int)lws_ring_get_count_free_elements(lws_vhd->ring);
      if (!n) {
        log_warn("OOM: ring no space!");
        lws_ring_clear(lws_vhd->ring);
      }
      
      std::string payload = received_msg.msg;
      amsg.len = payload.size();
      amsg.payload = (unsigned char*)malloc(LWS_PRE + amsg.len);
      if (!amsg.payload) {
        log_error("OOM: dropping no memory!");
        goto wait_unlock;
      }
      memcpy(amsg.payload + LWS_PRE, payload.c_str(), payload.size());
      n = lws_ring_insert(lws_vhd->ring, &amsg, 1);
      if (n != 1) {
        __minimal_destroy_message(&amsg);
        log_error("OOM: dropping insert failed!");
      }

wait_unlock:
      /*
       * This will cause a LWS_CALLBACK_EVENT_WAIT_CANCELLED
       * in the lws service thread context.
       */
      lws_cancel_service(lws_vhd->context);
      
      pthread_mutex_unlock(&lws_vhd->lock_ring); /* } ring lock ------- */
    }
    pthread_mutex_unlock(&lock_lws_vhd); /* } lock_lws_vhd ------- */
  }
}

/* this runs under the lws service thread context only */

static int
callback_minimal(struct lws *wsi, enum lws_callback_reasons reason,
      void *user, void *in, size_t len)
{
  struct per_session_data__minimal *pss = (struct per_session_data__minimal *)user;
  const struct msg *pmsg;
  void *retval;
  int n, m, r = 0;
  
  pthread_mutex_lock(&lock_lws_vhd); /* --------- lock_lws_vhd { */
  lws_vhd = (struct per_vhost_data__minimal *) lws_protocol_vh_priv_get(lws_get_vhost(wsi), lws_get_protocol(wsi));
  pthread_mutex_unlock(&lock_lws_vhd); /* } lock_lws_vhd ------- */

  switch (reason) {
  case LWS_CALLBACK_PROTOCOL_INIT: {
    log_debug("%s LWS_CALLBACK_PROTOCOL_INIT", __FUNCTION__);
    /* create our per-vhost struct */
    pthread_mutex_lock(&lock_lws_vhd); /* --------- lock_lws_vhd { */
    lws_vhd = (struct per_vhost_data__minimal *)lws_protocol_vh_priv_zalloc(lws_get_vhost(wsi), 
          lws_get_protocol(wsi), sizeof(struct per_vhost_data__minimal));
    if (!lws_vhd) {
      pthread_mutex_unlock(&lock_lws_vhd); /* } lock_lws_vhd ------- */
      return 1;
    }

    pthread_mutex_init(&lws_vhd->lock_ring, NULL);

    /* recover the pointer to the globals struct */
    lws_vhd->context = lws_get_context(wsi);
    lws_vhd->protocol = lws_get_protocol(wsi);
    lws_vhd->vhost = lws_get_vhost(wsi);

    lws_vhd->ring = lws_ring_create(sizeof(struct msg), 100, __minimal_destroy_message);
    if (!lws_vhd->ring) {
      log_error("%s: failed to create ring", __func__);
      pthread_mutex_unlock(&lock_lws_vhd); /* } lock_lws_vhd ------- */
      return 1;
    }
    pthread_mutex_unlock(&lock_lws_vhd); /* } lock_lws_vhd ------- */
    break;
  }

  case LWS_CALLBACK_PROTOCOL_DESTROY: {
    log_debug("%s LWS_CALLBACK_PROTOCOL_DESTROY", __FUNCTION__);
    pthread_mutex_lock(&lock_lws_vhd); /* --------- lock_lws_vhd { */
    lws_vhd->finished = 1;
    if (lws_vhd->ring)
      lws_ring_destroy(lws_vhd->ring);

    pthread_mutex_destroy(&lws_vhd->lock_ring);
    pthread_mutex_unlock(&lock_lws_vhd); /* } lock_lws_vhd ------- */
    break;
  }

  case LWS_CALLBACK_ESTABLISHED: {
    log_debug("%s LWS_CALLBACK_ESTABLISHED", __FUNCTION__);
    lws_start_foreach_llp(struct per_session_data__minimal **,
              ppss, lws_vhd->pss_list) {
      (*ppss)->kickout = 1;
      lws_callback_on_writable((*ppss)->wsi);
    } lws_end_foreach_llp(ppss, pss_list);
    
    /* add ourselves to the list of live pss held in the lws_vhd */
    lws_ll_fwd_insert(pss, pss_list, lws_vhd->pss_list);
    pss->tail = lws_ring_get_oldest_tail(lws_vhd->ring);
    pss->wsi = wsi;
    pss->kickout = 0;
    break;
  }

  case LWS_CALLBACK_CLOSED: {
    log_debug("%s LWS_CALLBACK_CLOSED", __FUNCTION__);
    /* remove our closing pss from the list of live pss */
    lws_ll_fwd_remove(struct per_session_data__minimal, pss_list,
          pss, lws_vhd->pss_list);
    break;
  }

  case LWS_CALLBACK_SERVER_WRITEABLE: {
    log_debug("%s LWS_CALLBACK_SERVER_WRITEABLE", __FUNCTION__);
    pthread_mutex_lock(&lws_vhd->lock_ring); /* --------- ring lock { */
    /* notice we allowed for LWS_PRE in the payload already */
    if (!pss->kickout) {
      pmsg = (const struct msg *)lws_ring_get_element(lws_vhd->ring, &pss->tail);
      if (!pmsg) {
        pthread_mutex_unlock(&lws_vhd->lock_ring); /* } ring lock ------- */
        break;
      }
      m = lws_write(wsi, pmsg->payload + LWS_PRE, pmsg->len, LWS_WRITE_TEXT);
      if (m < (int)pmsg->len) {
        log_error("ERROR %d writing to ws socket", m);
        lws_close_reason(wsi, LWS_CLOSE_STATUS_NORMAL, (unsigned char *)"socket", 6);
        pthread_mutex_unlock(&lws_vhd->lock_ring); /* } ring lock ------- */
        return -1;
      }
    } else {
       log_error("ERROR ws kickout");
       lws_close_reason(wsi, LWS_CLOSE_STATUS_NORMAL, (unsigned char *)"kickout", 7);
       pthread_mutex_unlock(&lws_vhd->lock_ring); /* } ring lock ------- */
       return -1;
    }

    lws_ring_consume_and_update_oldest_tail(
      lws_vhd->ring,  /* lws_ring object */
      struct per_session_data__minimal, /* type of objects with tails */
      &pss->tail,  /* tail of guy doing the consuming */
      1,    /* number of payload objects being consumed */
      lws_vhd->pss_list,  /* head of list of objects with tails */
      tail,    /* member name of tail in objects with tails */
      pss_list  /* member name of next object in objects with tails */
    );

    /* more to do? */
    if (lws_ring_get_element(lws_vhd->ring, &pss->tail))
      /* come back as soon as we can write more */
      lws_callback_on_writable(pss->wsi);

    pthread_mutex_unlock(&lws_vhd->lock_ring); /* } ring lock ------- */
    break;
  }

  case LWS_CALLBACK_RECEIVE: {
    log_debug("%s LWS_CALLBACK_RECEIVE", __FUNCTION__);
    
    Json::Reader reader;
    Json::Value root;
    atris_msgs::SignalMessage msg;
    msg.msg.assign((const char*)in, len);
    if (reader.parse(msg.msg, root)) {
      if (!root["title"].isNull() && !root["accid"].isNull() && !root["content"].isNull() 
        && !root["content"]["id"].isNull() && !root["content"]["timestamp"].isNull()) {
        msg.msgID = root["content"]["id"].asString();
        msg.timestamp = root["content"]["timestamp"].asInt64();
        msg.title = root["title"].asString();
        msg.account = root["accid"].asString();
        msg.type = "shttpd";
        
        Json::FastWriter jwriter;
        Json::Value reponse;
        bool need_reponse = true;
        ros::Time now = ros::Time::now();
        shm::Robot shmrbt;
        shm::iMemory_read_Robot(&shmrbt);
        
        reponse["title"] = "response" + msg.title.substr(7);
        reponse["accid"] = shmrbt.robot.sn;
        reponse["content"]["id"] = msg.msgID;
        reponse["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000);
        reponse["content"]["result"] = "fail_invalid_data";
        if (msg.title == "request_rosshttpd_pwd") {
          if (!root["content"]["old"].isNull() && !root["content"]["new"].isNull()) {
            char **result;
            int row, column;
            SqliteEngine::query("SELECT pwd FROM rosshttpd", &result, &row, &column, SQLITE_ROSSHTTPD_FILE);
            std::string pwd = row > 0 ? result[column] : "";
            std::string pwd_old = root["content"]["old"].asString();
            std::string pwd_new = root["content"]["new"].asString();
            SqliteEngine::freeQuery(result);
            if (!pwd_new.empty() && (pwd == pwd_old)) {
              SqliteEngine::execSQL("UPDATE rosshttpd SET pwd='"+pwd_new+"'", SQLITE_ROSSHTTPD_FILE);
              reponse["content"]["result"] = "success";
            } else {
              if (pwd != pwd_old) {
                reponse["content"]["result"] = "fail_invalid_old_pwd";
              }
              if (pwd_new.empty()) {
                reponse["content"]["result"] = "fail_emtpy_new_pwd";
              }
            }
          }
        } else if (msg.title == "request_rosshttpd_cfg_devices_ip") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if (!root["content"]["ptz_ip"].isNull() 
            && !root["content"]["ptz_box_ip"].isNull()
            && !root["content"]["nvr_ip"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setDevicesIP(
              root["content"]["ptz_ip"].asString(),
              root["content"]["ptz_box_ip"].asString(),
              root["content"]["nvr_ip"].asString()
            ) ? "success" : "fail_set_devices_ip";
          }
        } else if (msg.title == "request_rosshttpd_cfg_udock") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if (!root["content"]["dock_wall_width"].isNull() 
            && !root["content"]["dock_1_0"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setUdock(
              root["content"]["dock_wall_width"].asInt(),
              root["content"]["dock_1_0"].asInt()
            ) ? "success" : "fail_set_udock";
          }
        } else if (msg.title == "request_rosshttpd_cfg_mqtt") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if (!root["content"]["host"].isNull()
            && !root["content"]["port"].isNull()
            && !root["content"]["username"].isNull()
            && !root["content"]["password"].isNull()
            && !root["content"]["encrypt"].isNull()
            && !root["content"]["keepalive"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setMQTT(
              root["content"]["host"].asString(),
              root["content"]["port"].asInt(),
              root["content"]["username"].asString(),
              root["content"]["password"].asString(),
              root["content"]["encrypt"].asInt(),
              root["content"]["keepalive"].asInt()
            ) ? "success" : "fail_set_mqtt";
          }
        } else if (msg.title == "request_rosshttpd_cfg_remote_mqtt") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if ( !root["content"]["topic_url"].isNull()
            && !root["content"]["host"].isNull()
            && !root["content"]["port"].isNull()
            && !root["content"]["username"].isNull()
            && !root["content"]["password"].isNull()
            && !root["content"]["encrypt"].isNull()
            && !root["content"]["keepalive"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setRemoteMQTT(
              root["content"]["topic_url"].asString(),
              root["content"]["host"].asString(),
              root["content"]["port"].asInt(),
              root["content"]["username"].asString(),
              root["content"]["password"].asString(),
              root["content"]["encrypt"].asInt(),
              root["content"]["keepalive"].asInt()
            ) ? "success" : "fail_set_remote_mqtt";
          }
        } else if (msg.title == "request_rosshttpd_cfg_router") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if (!root["content"]["type"].isNull() 
            && !root["content"]["ip"].isNull()
            && !root["content"]["username"].isNull()
            && !root["content"]["password"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setRouter(
              root["content"]["type"].asString(),
              root["content"]["ip"].asString(),
              root["content"]["username"].asString(),
              root["content"]["password"].asString()
            ) ? "success" : "fail_set_router";
          }
        } else if (msg.title == "request_rosshttpd_cfg_ntp") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if (!root["content"]["ntp_server"].isNull() 
            && !root["content"]["timezone"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setNTP(
              root["content"]["ntp_server"].asString(),
              root["content"]["timezone"].asInt()
            ) ? "success" : "fail_set_ntp";
          }
        } else if (msg.title == "request_rosshttpd_cfg_hfs") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if (!root["content"]["hfs_type"].isNull() 
            && !root["content"]["hfs_url"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setHFS(
              root["content"]["hfs_type"].asString(),
              root["content"]["hfs_url"].asString()
            ) ? "success" : "fail_set_hfs";
          }
        } else if (msg.title == "request_rosshttpd_cfg_abi") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if (!root["content"]["base_url"].isNull() 
            && !root["content"]["timestamp_url"].isNull()
            && !root["content"]["robotinfo_url"].isNull()
            && !root["content"]["abi_enable"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setABI(
              root["content"]["base_url"].asString(),
              root["content"]["timestamp_url"].asString(),
              root["content"]["robotinfo_url"].asString(),
              root["content"]["abi_enable"].asInt()
            ) ? "success" : "fail_set_abi";
          }
          if (!root["content"]["event_url"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setPostEventUrl(
              root["content"]["event_url"].asString()
            ) ? "success" : "fail_event_url";
          }
        } else if (msg.title == "request_rosshttpd_cfg_voip_params") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if (!root["content"]["auto_setting"].isNull()
            && !root["content"]["sip_server"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setVoipParams(
              root["content"]["auto_setting"].asInt(),
              root["content"]["sip_server"].asString()
            ) ? "success" : "fail_set_voip_params";
          }
         } else if (msg.title == "request_rosshttpd_cfg_nav") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if (!root["content"]["obs_mode"].isNull()
            && !root["content"]["cycle_enable"].isNull()
            && !root["content"]["cycle_dst"].isNull()
            && !root["content"]["without_charge_path"].isNull()
            && !root["content"]["task_start_timeout"].isNull()
            && !root["content"]["task_running_timeout"].isNull()
            && !root["content"]["without_merge_path"].isNull()
            && !root["content"]["auto_charge_self"].isNull()
            && !root["content"]["anti_drop_enable"].isNull()
            && !root["content"]["tsp_enable"].isNull()
            && !root["content"]["auto_charge_battery"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setNav(
              root["content"]["obs_mode"].asInt(),
              root["content"]["cycle_dst"].asString(),
              root["content"]["cycle_enable"].asInt(),
              root["content"]["without_charge_path"].asInt(),
              root["content"]["task_start_timeout"].asInt(),
              root["content"]["task_running_timeout"].asInt(),
              root["content"]["without_merge_path"].asInt(),
              root["content"]["auto_charge_self"].asInt(),
              root["content"]["auto_charge_battery"].asInt(),
              root["content"]["anti_drop_enable"].asInt(),
              root["content"]["tsp_enable"].asInt()
            ) ? "success" : "fail_set_nav";
          }
        } else if (msg.title == "request_rosshttpd_cfg_vision") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if (!root["content"]["vision_host"].isNull()
            && !root["content"]["vision_port"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setVision(
              root["content"]["vision_host"].asString(),
              root["content"]["vision_port"].asInt()
            ) ? "success" : "fail_set_vision";
          }
        } else if(msg.title == "request_rosshttpd_cfg_obstacle") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if (!root["content"]["obstacle_mode"].isNull()
            && !root["content"]["obstacle_timeout"].isNull()
            && !root["content"]["obstacle_timeout_strategy"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setObstacle(
              root["content"]["obstacle_mode"].asInt(),
              root["content"]["obstacle_timeout"].asInt(),
              root["content"]["obstacle_timeout_strategy"].asInt()         
            ) ? "success" : "fail_set_obstacle";
          }           
        }else if (msg.title == "request_rosshttpd_cfg_default") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          reponse["content"]["result"] = Config::get_instance()->setDefault(
          ) ? "success" : "fail_set_default";
        } else if (msg.title == "request_roshttpd_music_play") {
          char **result;
          int row, column, columnIdx;
          Json::Value req, list;
          msg.title = "request_music_play";
          req["title"] = msg.title;
          req["accid"] = msg.account;
          req["content"]["id"] = msg.msgID;
          req["content"]["timestamp"] = msg.timestamp;
          req["content"]["action"] = "play";
          req["content"]["mode"] = "recycle";
          
          SqliteEngine::query("SELECT name FROM ppplay", &result, &row, &column);
          columnIdx = column;
          list.resize(0);
          for (int i = 0; i < row; i++) {
            list.append(result[columnIdx]);
            columnIdx += column;
          }
          SqliteEngine::freeQuery(result);

          req["content"]["play_list"] = list;

          msg.msg = jwriter.write(req);
          
          need_reponse = false;
          shttpd_req_pub.publish(msg);
        } else if (msg.title == "request_rosshttpd_cfg_open_light_imsi_detection") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          if (!root["content"]["open_light_imsi_detection"].isNull()) {
            reponse["content"]["result"] = Config::get_instance()->setOpenRedBlueLightImsiDetection(
            root["content"]["open_light_imsi_detection"].asInt()) ? "success" : "fail_set_open_light_imsi_detection";
          }
        } else if (msg.title == "request_rosshttpd_global_init") {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          reponse = shttpdResponseGlobalInit(msg);
        } else {
          need_reponse = false;
          shttpd_req_pub.publish(msg);
        }

        if (need_reponse) {
          std::string payload = jwriter.write(reponse);
          struct msg amsg;
          amsg.len = payload.size();
          amsg.payload = (unsigned char*)malloc(LWS_PRE + amsg.len);
          memcpy(amsg.payload + LWS_PRE, payload.c_str(), payload.size());
          pthread_mutex_lock(&lws_vhd->lock_ring); /* --------- ring lock { */
          lws_ring_insert(lws_vhd->ring, &amsg, 1);
          lws_cancel_service(lws_vhd->context);
          pthread_mutex_unlock(&lws_vhd->lock_ring); /* } ring lock ------- */
        } else {
          if (msg.title != "request_joystick_move") {
            log_info("%s account: %s, msg: %s", __FUNCTION__, msg.account.c_str(), msg.msg.c_str());
          }
        }
      }
    }else{
      log_info("\033[1;31m parse json filure !\033[0m");
    }
    break;
  }

  case LWS_CALLBACK_EVENT_WAIT_CANCELLED: {
    log_debug("%s LWS_CALLBACK_EVENT_WAIT_CANCELLED", __FUNCTION__);
    if (!lws_vhd)
      break;
    /*
     * When the "spam" threads add a message to the ringbuffer,
     * they create this event in the lws service thread context
     * using lws_cancel_service().
     *
     * We respond by scheduling a writable callback for all
     * connected clients.
     */
    lws_start_foreach_llp(struct per_session_data__minimal **, ppss, lws_vhd->pss_list) {
      lws_callback_on_writable((*ppss)->wsi);
    } lws_end_foreach_llp(ppss, pss_list);
    break;
  }

  default:
    break;
  }

  return r;
}

#define MAX_PAYLOAD_SIZE  (65 * 1024)
static const struct lws_protocols protocols[] = {
    {
        "lws-minimal", callback_minimal, sizeof( struct per_session_data__minimal ), MAX_PAYLOAD_SIZE,
    },
    {
        NULL, NULL, 0 
    }
};

