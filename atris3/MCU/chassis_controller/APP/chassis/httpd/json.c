#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "json.h"
#include <math.h>
#include <stdarg.h>
#include <rtthread.h>
int initJson(char *buf,char *json_name)
{
    int len = sprintf(buf,"{\"name\" : \"%s\"}\n", json_name);
     //printf("%s(%d):%s",__FUNCTION__,len,buf);
    return len;    
}

int addStrToJson(char *buf,char *var_name,char *var)
{
    
    int len = strlen(buf);
    char *add_buf = buf + len - strlen("}\n");
    int lens = sprintf(add_buf,", \"%s\" : \"%s\"}\n", var_name,var);
    //printf("%s(%d):%s",__FUNCTION__,len + lens,buf);
    return len + lens - strlen("}\n");
}

int addIntToJson(char *buf,char *var_name,int var)
{
    int len = strlen(buf);
    char *add_buf = buf + len - strlen("}\n");
    int lens = sprintf(add_buf,", \"%s\" : \"%d\"}\n", var_name,var);
    //printf("%s(%d):%s",__FUNCTION__,len + lens,buf);
    return len + lens - strlen("}\n");
    
}

int addFoatToJson(char *buf,char *var_name,float var)
{   
   return addFoatToJsonValidNum(buf,var_name,var,2);
}

int addFoatToJsonValidNum(char *buf,char *var_name,float var,int num)
{
    int len = strlen(buf);
    char *add_buf = buf + len - strlen("}\n");
    if(num > 10)
        num = 10;

    for(int i=0;i<num;i++)
        var = var * 10;
    int int_var = (int)var;
    int lens = sprintf(add_buf,", \"%s\" : \"%d.%d\"}\n", var_name,int_var / 100, abs(int_var) % 100);
    //printf("%s(%d):%s",__FUNCTION__,len + lens,buf);
    return len + lens - strlen("}\n"); 
    
}

int json_printf(char *buf,char *var_name, const char *format, ...)
{
    int len = strlen(buf);  
    char *add_buf = buf + len - strlen("}\n");  
    int len_tem = sprintf(add_buf,", \"%s\" : \"", var_name);
    add_buf+=len_tem;
    len+=len_tem;
   
    va_list args;
    va_start(args, format);
    len_tem = rt_vsprintf(add_buf,format, args);
    va_end(args);
    len+=len_tem;
    add_buf+=len_tem;
    len_tem = sprintf(add_buf,"\"}\n");
   // printf("%s(%d,%d):%s",__FUNCTION__,len + len_tem - strlen("}\n"),strlen(buf),buf);
    return len + len_tem - strlen("}\n");  
    
}

