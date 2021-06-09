#ifndef __JSON_H__
#define __JSON_H__ 

//////////////////////////////////////////////////////////////////////////////////	 
#ifdef __cplusplus
 extern "C" {
#endif	

int initJson(char *buf,char *json_name);
int addStrToJson(char *buf,char *var_name,char *var);
int addIntToJson(char *buf,char *var_name,int var);
int addFoatToJson(char *buf,char *var_name,float var);
int addFoatToJsonValidNum(char *buf,char *var_name,float var,int num);
int json_printf(char *buf,char *var_name, const char *format, ...);
    
#ifdef __cplusplus
}
#endif


#endif

