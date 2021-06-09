#ifndef __ATRIS_PARAM_TABLE_H_
#define __ATRIS_PARAM_TABLE_H_

namespace shm {
  typedef struct {
    char imx_ver[60];
    char power_ver[60];
    char gs_ver[200];
    char bm_ver[60];
    char bms_ver[60];
  } SWVersion;
  
  typedef struct {
    bool upgrading;
    int status;
  } UpgradeStatus;
  
  typedef struct {
    bool calibrated;
    int64_t interval;
  } TimeCalibrated;
  
  typedef struct {
    bool braked;
  } RobotBraked;
  
  typedef struct {
    uint16_t data[8];
  } UltraSound;
  
  typedef struct {
    double speed_x;
    double speed_y;
    double speed_z;
    double dist_left;
    double dist_right;
    double dist_center;
    double dist_theta;
    double odom;
  } OdomInfo;
  
  typedef struct {
    int state;
  } ChargeState;
  
  typedef struct {
    bool braked;
  } RebootBraked;
  
  typedef struct {
    bool protect;
  } ProtectMode;
  
  typedef struct {
    struct {
      char sn[20];
      char name[36];
      char pwd[36];
      char receiver[36];
      char accid[36];
      char token[36];
      char channel[36];
      int binded;
    } robot;
    struct {
      double chassis_speed;
      float ptz_step;
      int volume;
      int play_interval;
      int muted;
      int shaked;
      int tts_lng;
      int tts_speaker;
      int tts_enable;
      int sip_volume;
      int power_warning_value;
      int power_warning_charge;
      char sip_num[20];
      char speed_level[20];
    } appdata;
  } Robot;

  typedef struct {
     char company_id[36];
  } CompanyId;

  typedef struct {
    char map_name[64];
    char scheme_name[128];
    char operation_type[32];
    char task_mould_id[128];
    char task_timestamp[64];
    int pointBaseId;
  } TaskInfo;

  typedef struct {
    char recognitionType[64];
    char meterType[32];
    char meterResult[64];
    char visiblePicUrl[80];
    char thermometryPicUrl[80];
    char audioUrl[80];
    char videoUrl[80];
    bool audioStatus;
  } TaskResult;

}
#endif
