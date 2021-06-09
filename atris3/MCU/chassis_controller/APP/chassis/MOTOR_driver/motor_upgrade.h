

#ifndef __MOTOR_UPGRADE__
#define __MOTOR_UPGRADE__

#ifdef __cplusplus
extern "C" {
#endif
#include "can2.h"
#include "string.h"
#include "can2_fifo_rx.h"    

#define DF_MOTOR_UPGRADE_INC    1
#define DF_MOTOR_UPGRADE_FORCE  2
    

#define DF_MOTOR_UPGRADE_FILE "C:/DS10240C.bin"
    
int handshake(int id);
int checkall(int id,const char *file_name );
int binSend(int id,const char *file_name  );    
rt_bool_t LoadingThread(int id,const char *file_name);   
rt_bool_t motorUpgrade(int upgrade_mode);    
rt_bool_t motorUpgradeCmdfun(const char *group,char *cmd);
rt_bool_t motorUpgrade(int upgrade_mode);
void enterMotorUpgrade(int mode);
void get_char_motot_id(int id,char *char_id);
#ifdef __cplusplus
}
#endif



#endif

