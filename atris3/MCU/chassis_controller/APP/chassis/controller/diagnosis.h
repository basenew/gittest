#ifndef __DIAGNOSIS_H__
#define __DIAGNOSIS_H__ 

//////////////////////////////////////////////////////////////////////////////////	 
#ifdef __cplusplus
 extern "C" {
#endif			
#include <httpd.h>     
     
void steerLeftMove(void  *steerP,const char *cmd);
void steerRightMove(void  *steerP,const char *cmd);
void motorRunForward(void  *steerP,const char *cmd);
void motorRunBackward(void  *steerP,const char *cmd);
     
rt_bool_t diagnosisCmdfun(const char *group,const char *cmd);     
rt_bool_t diagnosisControl(void);
void enterDiagnosisWeb(rt_bool_t is_web);
int getDiagnosisJson(char *res,char * web_name);
     
//power-------------------------------
void powerControl(void  *sw,const char *key);
rt_bool_t powerDiaCmdfun(const char *group,char *cmd);
int getPowerJson(char *res,char * web_name);
int addVoltage(char *res,char *id,int ch);  
//brake and sensor-------------------------------     
void brakeUnlock(void);
void screenTest(void);
int getBrakeSensorJson(char *res,char * web_name);
rt_bool_t powerBrakeSensorCmdfun(const char *group,char *cmd);


int addEStopToJson(char *res,const char *id,int ch);
int addAntiToJson(char *res,const char *id,int ch);
int addBrakeStatusToJson(char *res,const char *id,int ch);
int addBrakeCauseToJson(char *res,const char *id,int ch);
int addNTCTempToJson(char *res,const char *id,int ch);
int addHumidityToJson(char *res,const char *id,int ch);
int addBatteryCellVoltToJson(char *res,const char *id,int ch);
int addBatteryTotalVoltToJson(char *res,const char *id,int ch);
int addBatteryTotalCurrToJson(char *res,const char *id,int ch);
int addRemoteToJson(char *res,const char *id,int ch);
#ifdef __cplusplus
}
#endif


#endif
