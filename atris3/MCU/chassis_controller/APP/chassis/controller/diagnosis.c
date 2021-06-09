#include <rtthread.h>
#include "diagnosis.h"
#include "controller.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "chassis_common.h"
#include "chassis_odom.h"
#include "json.h"
#include "power_ctrl.h"
#include "voltage_detect.h"
#include "e_stop_detect.h"
#include "brake.h"
#include "ntc_sample.h"
#include "remote_ctrl.h"
#include "sensor_detec.h"
#include "curr_sample.h"
#include "bms.h"

#define LOG_TAG              "dia"
#define LOG_LVL              LOG_LVL_INFO
#include <ulog.h>

volatile rt_bool_t is_diagnosis_web = RT_FALSE;

cWebCmd diagnosis_web_cmd[] ={
    {"lfs<",(void *)&leftFront,steerLeftMove},{"lfs>",(void *)&leftFront,steerRightMove},{"lfm^",(void *)&leftFront,motorRunForward},{"lfmv",(void *)&leftFront,motorRunBackward},
    {"rfs<",(void *)&rightFront,steerLeftMove},{"rfs>",(void *)&rightFront,steerRightMove},{"rfm^",(void *)&rightFront,motorRunForward},{"rfmv",(void *)&rightFront,motorRunBackward},
    {"lts<",(void *)&leftRear,steerLeftMove},{"lts>",(void *)&leftRear,steerRightMove},{"ltm^",(void *)&leftRear,motorRunForward},{"ltmv",(void *)&leftRear,motorRunBackward},
    {"rts<",(void *)&rightRear,steerLeftMove},{"rts>",(void *)&rightRear,steerRightMove},{"rtm^",(void *)&rightRear,motorRunForward},{"rtmv",(void *)&rightRear,motorRunBackward}     
};

rt_bool_t diagnosisCmdfun(const char *group,const char *cmd)
{
    
    int cmd_num = sizeof(diagnosis_web_cmd) / sizeof(cWebCmd);
    //rt_kprintf("diagnosisCmdfun(%s):%s\r\n",group,cmd);
    if (!strcmp(group, "dia")){
        for(int i=0;i<cmd_num;i++){
           // rt_kprintf("Cmdfun(%s):%s\r\n",diagnosis_web_cmd[i].para,cmd);
          if (!strcmp(cmd, diagnosis_web_cmd[i].cmd)){
               diagnosis_web_cmd[i].fun(diagnosis_web_cmd[i].para,cmd);  
                return RT_TRUE;
          }
          
        }
    }else return RT_FALSE;
    
    return RT_TRUE;
    
}


void enterDiagnosisWeb(rt_bool_t is_web){
    if(is_diagnosis_web)
        is_diagnosis_web = is_web;
}

void controlDiagnosisWeb(){
    if(!is_diagnosis_web){
                     Motor_enable(0x11);
                     Motor_enable(0x12);
                     Motor_enable(0x13);
                     Motor_enable(0x14); 
                     Motor_enable(0x21);
                     Motor_enable(0x22);
                     Motor_enable(0x23);
                     Motor_enable(0x24); 
                    is_diagnosis_web = RT_TRUE;  
    }
    
}
 
void steerLeftMove(void  *steerP,const char *cmd){
    WHEEL_STEER_def *p = (WHEEL_STEER_def*)steerP;
    //rt_kprintf("left move(%s):%s\r\n",p->name,cmd);
    controlDiagnosisWeb();
    SteeringMoveSetp(p,0.1);
}
void steerRightMove(void  *steerP,const char *cmd){
     WHEEL_STEER_def *p = (WHEEL_STEER_def*)steerP;
   // rt_kprintf("right move(%s):%s\r\n",p->name,cmd);  
    controlDiagnosisWeb();
    SteeringMoveSetp(p,-0.1);    
}

void motorRunForward(void  *steerP,const char *cmd){
     WHEEL_STEER_def *p = (WHEEL_STEER_def*)steerP;
    //rt_kprintf("forward move(%s):%s\r\n",p->name,cmd);
    controlDiagnosisWeb();    
    SteeringMoveSpeed(p,0.02);
}
void motorRunBackward(void  *steerP,const char *cmd){
     WHEEL_STEER_def *p = (WHEEL_STEER_def*)steerP;
    //rt_kprintf("backward move(%s):%s\r\n",p->name,cmd);
    controlDiagnosisWeb();
    SteeringMoveSpeed(p,-0.02);    
}



int getDiagnosisJson(char *res,char * web_name)
{
    
   int lf = (int)(wheelTheta(&leftFront) * 180 * 100 / PI);
   int rf = (int)(wheelTheta(&rightFront) * 180 * 100 / PI);
   int lt = (int)(wheelTheta(&leftRear) * 180 * 100 / PI);
   int rt = (int)(wheelTheta(&rightRear) * 180 * 100 / PI);
    
   int lf485 = (int)(wheelTheta485(&leftFront) * 180 * 100 / PI);
   int rf485 = (int)(wheelTheta485(&rightFront) * 180 * 100 / PI);
   int lt485 = (int)(wheelTheta485(&leftRear) * 180 * 100 / PI);
   int rt485 = (int)(wheelTheta485(&rightRear) * 180 * 100 / PI);

   int lfs = (int)(wheelSpeedTrans(&leftFront) * 100);
   int rfs = (int)(wheelSpeedTrans(&rightFront) * 100);
   int lts = (int)(wheelSpeedTrans(&leftRear) * 100);
   int rts = (int)(wheelSpeedTrans(&rightRear) * 100);
   
    int len = initJson(res,web_name);
   
    if(leftFront.steering_error_code != 0  && leftFront.abs_angle_error)
        len = json_printf(res,"lfS","error code:%.2x",leftFront.abs_angle_error?0x11:leftFront.steering_error_code);
    else if((leftFront.init_error_code & 5) == 0)
           len = json_printf(res,"lfS","%d.%d^(%d.%d^,%d^C)",lf / 100, abs(lf) % 100, lf485 / 100, abs(lf485) % 100,leftFront.steering_motor_temperature);
    else len = json_printf(res,"lfS","init error:0x%.2x",leftFront.init_error_code);
    
    if(rightFront.steering_error_code != 0  && rightFront.abs_angle_error)
        len = json_printf(res,"rfS","error code:%.2x",rightFront.abs_angle_error?0x11:rightFront.steering_error_code);
    else if((rightFront.init_error_code & 5) == 0)
           len = json_printf(res,"rfS","%d.%d^(%d.%d^,%d^C)",rf / 100, abs(rf) % 100, rf485 / 100, abs(rf485) % 100,rightFront.steering_motor_temperature);
       else len = json_printf(res,"rfS","init error:0x%.2x",rightFront.init_error_code);
    
    if(leftRear.steering_error_code != 0  && leftRear.abs_angle_error)
        len = json_printf(res,"ltS","error code:%.2x",leftRear.abs_angle_error?0x11:leftRear.steering_error_code);
    else if((leftRear.init_error_code & 5) == 0)
           len = json_printf(res,"ltS","%d.%d^(%d.%d^,%d^C)",lt / 100, abs(lt) % 100, lt485 / 100, abs(lt485) % 100,leftRear.steering_motor_temperature);
       else len = json_printf(res,"ltS","init error:0x%.2x",leftRear.init_error_code);
    
    if(rightRear.steering_error_code != 0  && rightRear.abs_angle_error)
        len = json_printf(res,"rtS","error code:%.2x",rightRear.abs_angle_error?0x11:rightRear.steering_error_code);
    else if((rightRear.init_error_code & 5) == 0)
           len = json_printf(res,"rtS","%d.%d^(%d.%d^,%d^C)",rt / 100, abs(rt) % 100, rt485 / 100, abs(rt485) % 100,rightRear.steering_motor_temperature);
       else len = json_printf(res,"rtS","init error:0x%.2x",rightRear.init_error_code);
 
    
    
    if(leftFront.drive_error_code != 0)
        len = json_printf(res,"lfM","error code:%.2x",leftFront.drive_error_code);
    else if((leftFront.init_error_code & 10) == 0)
           len = json_printf(res,"lfM","%d.%dm/s(%d^C)",lfs / 100, abs(lfs) % 100,leftFront.drive_motor_temperature);
       else len = json_printf(res,"lfM","init error:0x%.2x",leftFront.init_error_code);
    
    if(rightFront.drive_error_code != 0)
        len = json_printf(res,"rfM","error code:%.2x",rightFront.drive_error_code);
    else if((rightFront.init_error_code & 10) == 0)
           len = json_printf(res,"rfM","%d.%dm/s(%d^C)",rfs / 100, abs(rfs) % 100,rightFront.drive_motor_temperature);
       else len = json_printf(res,"rfM","init error:0x%.2x",leftFront.init_error_code);
    
    if(leftRear.drive_error_code != 0)
        len = json_printf(res,"ltM","error code:%.2x",leftRear.drive_error_code);
    else if((leftRear.init_error_code & 10) == 0)
           len = json_printf(res,"ltM","%d.%dm/s(%d^C)",lts / 100, abs(lts) % 100,leftRear.drive_motor_temperature);
       else len = json_printf(res,"ltM","init error:0x%.2x",leftFront.init_error_code);
    
    if(rightRear.drive_error_code != 0)
        len = json_printf(res,"rtM","error code:%.2x",rightRear.drive_error_code);
    else if((rightRear.init_error_code & 10) == 0)
           len = json_printf(res,"rtM","%d.%dm/s(%d^C)",rts / 100, abs(rts) % 100,rightRear.drive_motor_temperature);
       else len = json_printf(res,"rtM","init error:0x%.2x",leftFront.init_error_code);
 
  return len;    
    
}



rt_bool_t diagnosisControl(void)
{
    
    return is_diagnosis_web;
}



//power///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const cWebCmd power_web_cmd[] ={
    {"lf_steer_power",(void *)POWER_24V_LEFT_FRONT_STEER,powerControl},
    {"lf_direct_power",(void *)POWER_24V_LEFT_FRONT_DIRECT,powerControl},
    {"rf_steer_power",(void *)POWER_24V_RIGHT_FRONT_STEER,powerControl},
    {"rf_direct_power",(void *)POWER_24V_RIGHT_FRONT_DIRECT,powerControl},
    {"lt_steer_power",(void *)POWER_24V_LEFT_TEAR_STEER,powerControl},
    {"lt_direct_power",(void *)POWER_24V_LEFT_TEAR_DIRECT,powerControl},
    {"rt_steer_power",(void *)POWER_24V_RIGHT_TEAR_STEER,powerControl},
    {"rt_direct_power",(void *)POWER_24V_RIGHT_TEAR_DIRECT,powerControl},
    
    {"3_3_power",(void *)POWER_3V3_SYS,powerControl},
    {"5_9_remote_power",(void *)POWER_5V9_REMOTE,powerControl},
    {"12_reserve_power1",(void *)POWER_12V_RES1,powerControl},
    {"12_reserve_power2",(void *)POWER_12V_RES2,powerControl},
    {"12_tdk_power",(void *)POWER_12V_TDK,powerControl},
    {"24_tdk_power",(void *)POWER_24V_TDK,powerControl},

    {"5V_sys_view",(void *)VOLDET_CH_SYS_5V,RT_NULL},
    {"m_charge_view",(void *)VOLDET_CH_MCHARGE_24V,RT_NULL},
    {"12v_sys_view",(void *)VOLDET_CH_SYS_12V,RT_NULL},
    {"a_charge_view",(void *)VOLDET_CH_ACHARGE_24V,RT_NULL},
    {"24v_sys_view",(void *)VOLDET_CH_SYS_24V,RT_NULL},
    {"pole_charge_view",(void *)VOLDET_CH_CHARGER_POLE,RT_NULL},

};



rt_bool_t powerDiaCmdfun(const char *group,char *cmd)
{
    
    int cmd_num = sizeof(power_web_cmd) / sizeof(cWebCmd);
    char *sw = (char *)cmd;
    while((*cmd) != '-') cmd++;
    *cmd = 0;
    cmd++;
   //rt_kprintf("diagnosisCmdfun(%s):%s\r\n",group,cmd);
    if (!strcmp(group, "pow")){
        for(int i=0;i<cmd_num;i++){
            //rt_kprintf("Cmdfun(%.8x):%s %s\r\n",power_web_cmd[i].para,sw, power_web_cmd[i].cmd);
          if (!strcmp(sw, power_web_cmd[i].cmd)){
              if(power_web_cmd[i].fun == powerControl){
                    rt_kprintf("Cmdfun(%.8x):%s %s\r\n",power_web_cmd[i].para,sw, power_web_cmd[i].cmd);
                    power_web_cmd[i].fun(power_web_cmd[i].para,cmd);  
              }
                return RT_TRUE;
          }
          
        }
    }else return RT_FALSE;
    
    return RT_TRUE;
    
}

void powerControl(void  *sw,const char *key){
    int intsw = ((int)sw);
    //int status = ((*key)=='e')?1:0;
    LOG_I("%s(%x):%c\r\n",__FUNCTION__,intsw,*key);
    power_set_single_power(intsw,((*key)=='e')?1:0);
}

int getPowerJson(char *res,char * web_name)
{
    int len = initJson(res,web_name);
    int sw_status = power_get_status();

    int cmd_num = sizeof(power_web_cmd) / sizeof(cWebCmd);

        for(int i=0;i<cmd_num;i++){
            if(power_web_cmd[i].fun)
                len = addStrToJson(res,(char *)power_web_cmd[i].cmd,(((int)power_web_cmd[i].para) & sw_status)?"e":"d" );
            else len = addVoltage(res,(char *)power_web_cmd[i].cmd,(int)power_web_cmd[i].para);

          }
        
  return len;    
    
}

int addVoltage(char *res,char *id,int ch)
{
    int voltage = (int)(get_voltage(ch));

    return  json_printf(res,id,"%2d.%.3dV",voltage / 1000, abs(voltage) % 1000);
}

//power end///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//brake and sensor///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const cWebJson brake_sensor_up[] ={
    {"upemergency_button",RT_NULL,addEStopToJson},
    {"upbumper",RT_NULL,addAntiToJson},
    {"upbrake",RT_NULL,addBrakeStatusToJson},
    {"upbrake_source",RT_NULL,addBrakeCauseToJson},
    
    {"uptemperture1",NTC_IDX_CH1,addNTCTempToJson},
    {"uptemperture2",NTC_IDX_CH2,addNTCTempToJson},
    {"uptemperture3",NTC_IDX_CH3,addNTCTempToJson},
    {"uptemperture4",NTC_IDX_CH4,addNTCTempToJson},      
    {"uphumidity",RT_NULL,addHumidityToJson},
    
    {"upg1voltage1",RT_NULL,addBatteryCellVoltToJson},
    {"upg1voltage2",RT_NULL,addBatteryCellVoltToJson},
    {"upg1voltage3",RT_NULL,addBatteryCellVoltToJson},
    {"upg1voltage4",RT_NULL,addBatteryCellVoltToJson},
    {"upg1voltage5",RT_NULL,addBatteryCellVoltToJson},
    {"upg1voltage6",RT_NULL,addBatteryCellVoltToJson},
    {"upg1voltage7",RT_NULL,addBatteryCellVoltToJson},
    {"upg1voltage8",RT_NULL,addBatteryCellVoltToJson},
    
    {"upg2voltage1",RT_NULL,addBatteryCellVoltToJson},
    {"upg2voltage2",RT_NULL,addBatteryCellVoltToJson},
    {"upg2voltage3",RT_NULL,addBatteryCellVoltToJson},
    {"upg2voltage4",RT_NULL,addBatteryCellVoltToJson},
    {"upg2voltage5",RT_NULL,addBatteryCellVoltToJson},
    {"upg2voltage6",RT_NULL,addBatteryCellVoltToJson},
    {"upg2voltage7",RT_NULL,addBatteryCellVoltToJson},
    {"upg2voltage8",RT_NULL,addBatteryCellVoltToJson},
    
    {"upvoltage_toltal",RT_NULL,addBatteryTotalVoltToJson},
    {"upcurrent",RT_NULL,addBatteryTotalCurrToJson},
    {"upremote",REMOTE_CHARNNEL16,addRemoteToJson},

};



void brakeUnlock(void)
{
    LOG_E("brake unlock by web.");
	brake_set_byhost(BRAKE_STATUS_UNLOCK);
}

void screenTest(void)
{
   LOG_E("screenTest"); 
    
}

int addEStopToJson(char *res,const char *id,int ch)
{
	uint8_t status = e_stop_get_status();
	if(status)
	{
		return  json_printf(res,(char*)id,"trigger");
	}
	else
	{
		return  json_printf(res,(char*)id,"release");
	}
}

int addAntiToJson(char *res,const char *id,int ch)
{
//    return  addIntToJson(res,(char*)id,anti_get_data());
	
	uint8_t status = anti_get_data();
	if(status == 1)
	{
		return  json_printf(res,(char*)id,"trigger");
	}
	else if(status == 0)
	{
		return  json_printf(res,(char*)id,"release");
	}
	else
	{
		return  json_printf(res,(char*)id,"no anti");
	}
}

int addBrakeStatusToJson(char *res,const char *id,int ch)
{
//    return  addIntToJson(res,(char*)id,get_brake_status());
	uint8_t status = get_brake_status();
	if(status)
	{
		return  json_printf(res,(char*)id,"lock");
	}
	else
	{
		return  json_printf(res,(char*)id,"unlock");
	}
}

int addBrakeCauseToJson(char *res,const char *id,int ch)
{
    return  addIntToJson(res,(char*)id,get_brake_cause());
}

int addNTCTempToJson(char *res,const char *id,int ch)
{ 
    return  addIntToJson(res,(char*)id,getTempture(ch));
}
int addHumidityToJson(char *res,const char *id,int ch)
{

    int temp = sensor_get_hsu_temp();
	int hum = sensor_get_hsu_hum();

    return  json_printf(res,(char *)id,"%2d.%d^C-%d.%d%",temp/10, temp%10, hum/10, hum%10);
}

int addBatteryCellVoltToJson(char *res,const char *id,int ch)
{
    uint16_t voltage = get_bms_CellVolt(ch);
    return  json_printf(res,(char *)id,"%dmV",voltage);
}

int addBatteryTotalVoltToJson(char *res,const char *id,int ch)
{
    uint16_t voltage = get_bms_TotalVolt();
  
	return  json_printf(res,(char *)id,"%2d.%.1dV", voltage/10, voltage%10);
}

int addBatteryTotalCurrToJson(char *res,const char *id,int ch)
{
    int16_t current = get_bms_TotalCurr() - 5000;
	
	return  json_printf(res,(char *)id,"%2d.%.1dA", current/10, abs(current)%10);
}
int addRemoteToJson(char *res,const char *id,int ch)
{   
    int len = 0;
    char tem[128];    
    
    for(int i=0;i<ch;i++){
        len+=( sprintf(&tem[len],"%d,",getChannel(i)));
       
    }
    len+=sprintf(&tem[len],"%.2x,",getFlag());
   
    return  addStrToJson(res,(char*)id,tem);
}

int getBrakeSensorJson(char *res,char * web_name)
{
    int len = initJson(res,web_name);

    int cmd_num = sizeof(brake_sensor_up) / sizeof(cWebJson);

    for(int i=0;i<cmd_num;i++){
        len = brake_sensor_up[i].fun(res,brake_sensor_up[i].id,brake_sensor_up[i].ch);
        
       }
      
    return len;  
}

rt_bool_t powerBrakeSensorCmdfun(const char *group,char *cmd)
{

    if (!strcmp(cmd, "unLock")){
            brakeUnlock();  
            return RT_TRUE;
          } 

    if (!strcmp(cmd, "screenTest")){
            screenTest();  
            return RT_TRUE;
          } 

    return RT_FALSE;
}

//brake and sensor end///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
