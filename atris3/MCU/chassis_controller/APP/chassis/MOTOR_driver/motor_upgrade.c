#include "motor_upgrade.h"
#include "chassis_common.h"
#include "fsdata.h"
#include "ftpd.h"
#include "ff.h"
#define LOG_TAG              "moUp"
#define LOG_LVL              LOG_LVL_DBG
#include <ulog.h>
#include "json.h"
#include "power_ctrl.h"
volatile int malloc_inc = 0;
volatile int json_upgrade_id = -1;
volatile char json_upgrade_info[32];

FRESULT webf_open (struct fsdata_file* fp, const TCHAR* path, BYTE mode)
{
    struct fsdata_file* p = (struct fsdata_file*)get_motor_struct(path);
    
     LOG_I("%s(%x) %s",__FUNCTION__,p,path);
    if(p){
        memcpy(fp,p,sizeof(struct fsdata_file));
        LOG_I("%s memcpy(%x) %x",__FUNCTION__,p,fp);
    }
    

    if(fp && p){
        fp->next = 0;
        return FR_OK;
    }
    else return FR_DISK_ERR;
}
FRESULT webf_close (struct fsdata_file* fp){
    LOG_I("%s(%x) ",__FUNCTION__,fp);
    if(fp){
        return FR_OK;
    }
    else return FR_DISK_ERR;
}
FRESULT webf_read (struct fsdata_file* fp, void* buff, UINT btr, UINT* br)
{
    
     if(!fp)
         return FR_DISK_ERR;
    if(((int)fp->next) >= fp->len)
        return FR_DISK_ERR;
    if( (fp->len - ((int)fp->next)) < btr)
        btr = fp->len - ((int)fp->next);
    memcpy(buff,fp->data + ((int)fp->next),btr);
    *br = btr;

    UINT *np = (UINT *)&fp->next;
    *np = *np + btr;
    if(fp)
        return FR_OK;
    else return FR_DISK_ERR;
}
FRESULT webf_lseek (struct fsdata_file* fp, FSIZE_t ofs)
{
    //LOG_I("%s(%x):%d ",__FUNCTION__,fp, ofs);
      if(!fp)
         return FR_DISK_ERR;
    fp->next = (const struct fsdata_file *)ofs;  
    if(fp)
        return FR_OK;
    else return FR_DISK_ERR;      
}
int webf_size(struct fsdata_file *fp)
{
     LOG_I("%s(%x):%d",__FUNCTION__,fp, fp->len);
    if(!fp)
       return 0;
    return fp->len;  

}

    
#define webf_rewind(fp) f_lseek((fp), 0)



int handshake(int id)

{
	/****************************************************************************/
   //从界面获取发送信息
    int error = 1;
    int i;
    
	for (i = 0; i < 5; i++)
	{
		if(CHECK_UPGRADE_STATUS(id) != 0x22){
            CANopen_get_version_year(id);
            CANopen_get_version_date(id);
             rt_thread_mdelay(200);
            error&=CANopen_Activate(0);
            error&=SDO_Write_OD( id, SDO_W4, 0x4607,0x00,1);//从APP进入boot
            error&=SDO_Write_OD( id, SDO_W4, 0x4607,0x00,2);//从APP进入boot
            
            if(error == 0){
                LOG_I("%s(%d):%d send enter boot cmd false",__FUNCTION__, id,i);
                sprintf((char *)json_upgrade_info,"send enter boot false");
               error = 1;
                
            }   
            rt_thread_mdelay(300);
        }else{
            error&=SDO_Write_OD( id, SDO_W4, 0x20a0,0x00,1);//从APP进入boot
            error&=SDO_Write_OD( id, SDO_W4, 0x20a0,0x00,2);//从APP进入boot
             
            if(error == 0){
                LOG_I("%s(%d):%d boot shakehand false",__FUNCTION__, id,i);
                sprintf((char *)json_upgrade_info,"boot shakehand false");
                 error = -1;             
            }else{ 
                LOG_I("%s(%d): enter boot OK",__FUNCTION__, id);  
                sprintf((char *)json_upgrade_info,"enter boot OK");
                return 1;  

            }                
            
        }
        
        
	}
    if(i >= 5){
        LOG_I("%s(%d): enter boot false",__FUNCTION__, id); 
        sprintf((char *)json_upgrade_info,"enter boot false");
        return 0;
    }
     

    LOG_I("%s(%d): boot shakehand false",__FUNCTION__, id);
    sprintf((char *)json_upgrade_info,"boot shakehand false");
	return(1);
}

int calc_crc(const char * file_name)
{
    FIL *fpp;
    struct fsdata_file *fpw =  rt_malloc(sizeof(struct fsdata_file));
    malloc_inc++;
    UINT br = 0;
    uint16_t crc = 0;
    uint16_t r_date;
    int length;
    FRESULT fr;
    int i;
    if(webf_open(fpw,file_name,FA_READ) != FR_OK){
        
        LOG_I("web file %s:(%s) open file false(%x)",__FUNCTION__, file_name,fpw);
       // sprintf((char *)json_upgrade_info,"open file false");
        rt_free(fpw);
        malloc_inc--;
        fpw = NULL;
        fpp = rt_malloc(sizeof(FIL));
        malloc_inc++;
         if(f_open(fpp,file_name,FA_READ) != FR_OK){
            LOG_I("%s:(%s) open file false",__FUNCTION__, file_name);
            //sprintf((char *)json_upgrade_info,"open file false");
            rt_free(fpp);
           malloc_inc--;
            return -1;
        }             
    }

    if(fpw)
        length = webf_size(fpw);
    else length = f_size(fpp);
    LOG_I("%s:(%s) open file length:%d sizeof:%d",__FUNCTION__, file_name,length,sizeof(r_date));
    
    if(fpw)
        webf_lseek(fpw,0);
    else f_rewind(fpp);
    for(i=0;i<length;i+=2)
    {
        if(fpw)
            webf_lseek(fpw,i);
        else f_lseek (fpp,i);
        
        if(fpw)
            fr = webf_read(fpw,&r_date,sizeof(r_date),&br);
        else fr = f_read(fpp,&r_date,sizeof(r_date),&br);
        if(fr != FR_OK){
            LOG_I("%s:(%s)crc error:%d,br:%d sizeof:%d",__FUNCTION__, file_name,i,br,sizeof(r_date));
            sprintf((char *)json_upgrade_info,"crc error");
            return -1;;
        }
        crc += r_date;  
    }
      crc = ~crc;
    LOG_I("%s:(%s)crc 11:%.4x",__FUNCTION__, file_name,crc);
     if(fpw){
        webf_close(fpw);
        rt_free(fpw);
        malloc_inc--;
        fpw = NULL; 
     }else{
        f_close(fpp);
        rt_free(fpp);
         malloc_inc--;
     }

    return crc;
    
}

int checkall(int id,const char *file_name )
{
    
    int crc;
    int i;
    int error = 1;
	for (i = 0; i < 50; i++)
	{
        rt_thread_mdelay(200);
		if(CHECK_UPGRADE_STATUS(id) == 0x25){
            LOG_I("%s(%d):boot receive date ready",__FUNCTION__, id);
            sprintf((char *)json_upgrade_info,"boot upgrade ready");
            break;
        }
	}
    if(i >=50){
        LOG_I("%s(%d):boot receive date ready flase",__FUNCTION__, id);
        sprintf((char *)json_upgrade_info,"boot upgrade ready false");
        return -1;
    }
    
    
    
    
    crc = calc_crc(file_name);
    LOG_I("%s(%d):file crc:%.4x",__FUNCTION__, id,crc);
    if(crc < 0){
        LOG_I("%s(%d):crc error",__FUNCTION__, id);
        sprintf((char *)json_upgrade_info,"crc error");
        return -1;
    }
    
   error&=SDO_Write_OD( id, SDO_W4, 0x20a3,0x00,crc);//send crc
    

	if(error == 0){
        LOG_I("%s(%s):send crc error",__FUNCTION__, id);
        sprintf((char *)json_upgrade_info,"crc error");
        return -1;
    }
    LOG_I("%s(%d):erase app section ...",__FUNCTION__, id);
    sprintf((char *)json_upgrade_info,"erase app section");
    return 1;
}

int package_crc(uint16_t *date,int num)
{
    int i;
    uint16_t crc = 0;
    for(i=0;i<num;i++)
        crc += date[i];
    crc = ~crc;
    return crc;
}

int binSend(int id,const char *file_name  )

{

    int error = 1;
    int re = -1;
    int i;
    FIL *fpp;
    UINT br = 0;
    FRESULT fr;
    int little_pakage_num = 200;
    int up_data_length;
    uint32_t *package_buf = rt_malloc(sizeof(uint32_t) * little_pakage_num);
    malloc_inc++;

    int package_num;
    int end_can_package_num;
    int package_index;
    int can_index;
    struct fsdata_file *fpw =  rt_malloc(sizeof(struct fsdata_file));
    malloc_inc++;


	error&=SDO_Write_OD( id, SDO_W4, 0x20a6,0x00,little_pakage_num);//send little_pakage_num

    sprintf((char *)json_upgrade_info,"open file");

    if(webf_open(fpw,file_name,FA_READ) != FR_OK){
        rt_free(fpw);
        malloc_inc--;
        fpw = NULL;
        LOG_I("web file %s:(%s) open file false",__FUNCTION__, file_name);
        sprintf((char *)json_upgrade_info,"open file false");
        fpp = rt_malloc(sizeof(FIL));
        malloc_inc++;
        if(f_open(fpp,file_name,FA_READ) != FR_OK){
            LOG_I("%s:(%s) open file false",__FUNCTION__, file_name);
            sprintf((char *)json_upgrade_info,"open file false");
            rt_free(package_buf);
            malloc_inc--;            
            rt_free(fpp);
            malloc_inc--;
            re = -1;
            goto bin_send_exit;
        }             
    }
   

    if(fpw)
        up_data_length = webf_size(fpw);
    else up_data_length = f_size(fpp);
    end_can_package_num  = (up_data_length / sizeof(uint32_t)) % little_pakage_num;//最后一大包字节数
    package_num = up_data_length / (sizeof(uint32_t) * little_pakage_num);//大包数量
    
    for(package_index=0; package_index < package_num; package_index++){
        
        error&=SDO_Write_OD( id, SDO_W4, 0x20a6,0x00,little_pakage_num);//send little_pakage_num
        if(error == 0){
            LOG_I("%s(%d):package(%d) init error",__FUNCTION__, id,package_index);
            sprintf((char *)json_upgrade_info,"package(%d) init error",package_index);
            re = -1;
            goto bin_send_exit;
        }
        
        error&=SDO_Write_OD( id, SDO_W4, 0x20a7,0x00,package_index);//send little_pakage_num
        if(error == 0){
            LOG_I("%s(%d):package(%d) init error",__FUNCTION__, id,package_index);
            sprintf((char *)json_upgrade_info,"package(%d) init error",package_index);
            re = -1;
            goto bin_send_exit;
        }
        
        if(fpw){
            LOG_I("%s(%d):next(%d):%d/%d",__FUNCTION__, id,little_pakage_num,(int)fpw->next,fpw->len);
            sprintf((char *)json_upgrade_info,"next(%d):%d/%d",little_pakage_num,(int)fpw->next,fpw->len);
            fr = webf_read(fpw,package_buf,sizeof(uint32_t) * little_pakage_num,&br);
        }else fr = f_read(fpp,package_buf,sizeof(uint32_t) * little_pakage_num,&br);
        if(fr != FR_OK)
            break;
        int crc = package_crc((uint16_t *)package_buf , little_pakage_num * (sizeof(uint32_t) / (sizeof(uint16_t))));
        LOG_I("%s(%d):package crc(%d/%d):%.4x",__FUNCTION__, id,package_index,package_num,crc);
        sprintf((char *)json_upgrade_info,"package (%d/%d) %d%% crc:%.4x",package_index,package_num,(int)(package_index *100/package_num),crc);
        error&=SDO_Write_OD( id, SDO_W4, 0x20a5,0x00,crc);//send little_pakage_num
        if(error == 0){
            LOG_I("%s(%d):package(%d) crc error",__FUNCTION__, id,package_index);
            sprintf((char *)json_upgrade_info,"package(%d) crc error",package_index);
            re = -1;
            goto bin_send_exit;
        }
        
        for (i = 0; i < 50; i++)
        {
            int check = CHECK_UPGRADE_STATUS(id);
            if(check == 0x26){
                LOG_I("%s(%d):boot receive date start receive:%d",__FUNCTION__, id,package_index);
               // sprintf((char *)json_upgrade_info,"date start receive:%d",package_index);
                break;
            }
           if(check == 0x24){
                LOG_I("%s(%d):boot receive refesh error:%d",__FUNCTION__, id,package_index);
               sprintf((char *)json_upgrade_info,"receive refesh error:%d",package_index);
                re = -1;
                goto bin_send_exit;
            }
        }
        if(i >= 50){
            LOG_I("%s(%d):boot receive check error 11111",__FUNCTION__, id);
            sprintf((char *)json_upgrade_info,"receive check error:%d",package_index);
                re = -1;
                goto bin_send_exit;
        }
        
        i = 0;
        for(can_index=0; can_index<little_pakage_num; ){
           
            error = SDO_Write_OD( id, SDO_W4, 0x20a4,can_index,package_buf[can_index]);//send little_pakage_num
            if(error == 0){
                LOG_I("%s(%d):package(%d)can package(%d) data send error",__FUNCTION__, id,package_index,can_index);
                sprintf((char *)json_upgrade_info,"package(%d)can package(%d) data send error",package_index,can_index);
                i++;
                if(i > 50){ 
                    re = -1;
                    goto bin_send_exit;
                }
                 rt_thread_mdelay(1);
                continue;
            }   
              i = 0;       
            can_index++;
        }
         LOG_I("%s(%d):package(%d/%d) ",__FUNCTION__, id,package_index,package_num); 
        sprintf((char *)json_upgrade_info,"package(%d/%d)",package_index,package_num);        
        for (i = 0; i < 50; i++)
        {
            int check = CHECK_UPGRADE_STATUS(id);
            if(check == 0x25){
                LOG_I("%s(%d):boot receive date ready:%d",__FUNCTION__, id,package_index);
                sprintf((char *)json_upgrade_info,"boot receive date ready:%d",package_index);     
                break;
            }
           if(check == 0x24){
               LOG_I("%s(%d):boot receive refesh error:%d",__FUNCTION__, id,package_index);
               sprintf((char *)json_upgrade_info,"boot receive refesh error:%d",package_index);     
               re = -1;
            goto bin_send_exit;
            }
        }
        if(i >= 50){
            LOG_I("%s(%d):boot receive check error:%d",__FUNCTION__, id,package_index); 
            sprintf((char *)json_upgrade_info,"boot receive check error:%d",package_index);     
            re = -1;
            goto bin_send_exit;
        }            
        
    }
    

    LOG_I("%s(%d):end  package(%d) read(%d) %d",__FUNCTION__, id,end_can_package_num,br,br/sizeof(uint32_t));   
    sprintf((char *)json_upgrade_info,"end  package(%d) read(%d) %d",end_can_package_num,br,br/sizeof(uint32_t));   
    for (i = 0; i < 50; i++)
	{
        int check = CHECK_UPGRADE_STATUS(id);
		if(check == 0x25){
            LOG_I("%s(%d):boot receive date ready111",__FUNCTION__, id);
            sprintf((char *)json_upgrade_info,"boot receive date ready");  
            break;
        }
       if(check == 0x24){
            LOG_I("%s(%d):boot receive refesh error",__FUNCTION__, id);
           sprintf((char *)json_upgrade_info,"boot receive refesh error");  
            re = -1;
            goto bin_send_exit;
        }
	}
    if(i >= 50){
        LOG_I("%s(%d):boot receive check error 11111",__FUNCTION__, id);
        sprintf((char *)json_upgrade_info,"boot receive check error");  
           re = -1;
            goto bin_send_exit;
    }
    
    if(end_can_package_num > 0){
        
        memset(package_buf,0,sizeof(uint32_t) * little_pakage_num);
        if(fpw)
            fr = webf_read(fpw,package_buf,sizeof(uint32_t) * little_pakage_num,&br);
        else fr = f_read(fpp,package_buf,sizeof(uint32_t) * little_pakage_num,&br);
        //     fr = f_read(fpp,package_buf,sizeof(uint32_t) * little_pakage_num,&br);
        if(fr != FR_OK){
            LOG_I("%s(%d):end  package(%d) read error",__FUNCTION__, id,end_can_package_num);
            sprintf((char *)json_upgrade_info,"end  package(%d) read error",end_can_package_num);  
             re = -1;
            goto bin_send_exit;         
        }
        int crc = package_crc((uint16_t *)package_buf , end_can_package_num * (sizeof(uint32_t) / (sizeof(uint16_t))));
        LOG_I("%s(%d):end package crc(%d):%.4x",__FUNCTION__, id,end_can_package_num,crc);
        sprintf((char *)json_upgrade_info,"end package crc(%d):%.4x",end_can_package_num,crc);
        error&=SDO_Write_OD( id, SDO_W4, 0x20a6,0x00,end_can_package_num);//send little_pakage_num
        if(error == 0){
            LOG_I("%s(%d):end package(%d) init error",__FUNCTION__, id,end_can_package_num);
            sprintf((char *)json_upgrade_info,"end  package(%d) init error",end_can_package_num);
            re = -1;
            goto bin_send_exit;
        }
        error&=SDO_Write_OD( id, SDO_W4, 0x20a5,0x00,crc);//send little_pakage_num
        if(error == 0){
            LOG_I("%s(%d):end package(%d) crc error",__FUNCTION__, id,end_can_package_num);
            sprintf((char *)json_upgrade_info,"end  package(%d) crc error",end_can_package_num);
            re = -1;
            goto bin_send_exit;
        }
        error&=SDO_Write_OD( id, SDO_W4, 0x20a7,0x00,package_num);//send little_pakage_num
        if(error == 0){
            LOG_I("%s(%d):end package(%d) crc error",__FUNCTION__, id,end_can_package_num);
            sprintf((char *)json_upgrade_info,"end  package(%d) crc error",end_can_package_num);
            re = -1;
            goto bin_send_exit;
        }        
        for (i = 0; i < 50; i++)
        {int check = CHECK_UPGRADE_STATUS(id);
            if(check == 0x26){
                LOG_I("%s(%d):end receive date ready2222",__FUNCTION__, id);
                sprintf((char *)json_upgrade_info,"receive date ready");
                break;
            }
           if(check == 0x24){
                LOG_I("%s(%d):boot receive refesh error",__FUNCTION__, id);
               sprintf((char *)json_upgrade_info,"receive refesh error");
                re = -1;
            goto bin_send_exit;
            }
        }
        if(i >= 50){
            LOG_I("%s(%d):boot receive check error2222",__FUNCTION__, id);
            sprintf((char *)json_upgrade_info,"receive check error");
                re = -1;
            goto bin_send_exit;
            }
        i = 0;
        for(can_index=0; can_index<end_can_package_num; ){
            
            error = SDO_Write_OD( id, SDO_W4, 0x20a4,can_index,package_buf[can_index]);//send little_pakage_num
            if(error == 0){
                
                 i++;
                if(i > 50) {
                   re = -1;
                    goto bin_send_exit;
                }
                 rt_thread_mdelay(1);
                continue;            
            }  
            i = 0;
            //LOG_I("%s(%d):package(%d)end can package(%d/%d) ",__FUNCTION__, id,package_index,can_index,end_can_package_num);
            can_index++ ;           
            
        }
        
        
        
       LOG_I("%s(%d):package(%d) data send finished  ^_^",__FUNCTION__, id,package_index,can_index);
       sprintf((char *)json_upgrade_info,"package(%d) data send finished",can_index);

       // return -1;
//       rt_thread_mdelay(1000);
//       error&=SDO_Write_OD( id, SDO_W4, 0x20a8,0x00,1);//send little_pakage_num
//        if(error == 0){
//            LOG_I("%s(%d):data send OK ,reboot cmd error",__FUNCTION__, id);
//            return -1;
//        }  
//        rt_thread_mdelay(1000);
      for (i = 0; i < 50; i++)
        {int check = CHECK_UPGRADE_STATUS(id);
            if(check == 0x25){
                 
               error&=SDO_Write_OD( id, SDO_W4, 0x20a8,0x00,1);//send little_pakage_num
                if(error == 0){
                    LOG_I("%s(%d):data send OK ,reboot cmd error",__FUNCTION__, id);
                    sprintf((char *)json_upgrade_info,"reboot cmd error");
                    re = -1;
                    goto bin_send_exit;
                } 
                i = 0;  
                rt_thread_mdelay(1000);                
            }
            if(check == 0x27){
                    LOG_I("%s(%d):Burning ......",__FUNCTION__, id); 
                sprintf((char *)json_upgrade_info,"Burning ......");
                    i = 0;
            }
            if(check == 0x28){
                    LOG_I("%s(%d):Burning finished",__FUNCTION__, id); 
                    sprintf((char *)json_upgrade_info,"upgrade OK");
                    rt_thread_mdelay(1000);  
                    i = 0;  
                    break;
            }
            
            if(check == 0){
                LOG_I("%s(%d):reboot ok333",__FUNCTION__, id);
                sprintf((char *)json_upgrade_info,"reboot ok ^_^");
                 i = 0;
                break;
            }
           if(check == 0x24){
                LOG_I("%s(%d):boot receive refesh error333",__FUNCTION__, id);
               sprintf((char *)json_upgrade_info,"receive refesh error");
                re = -1;
            goto bin_send_exit;
            }
        }
    }
    if(i >= 50){
        LOG_I("%s(%d):boot receive check error3333",__FUNCTION__, id);
        sprintf((char *)json_upgrade_info,"boot receive check error");
                re = -1;
            goto bin_send_exit;
            }
bin_send_exit:
     rt_free(package_buf);
     malloc_inc--;
     if(fpw){
         webf_close(fpw);
         rt_free(fpw);
         malloc_inc--;
        fpw = NULL; 
     }else{
        f_close(fpp);
        rt_free(fpp);
         malloc_inc--;
     }

	return re;
}

rt_bool_t LoadingThread(int id,const char *file_name)
{
    LOG_I("%s(%d):motor upgrade start...-----------------!",__FUNCTION__, id);
    json_upgrade_id = id;

    sprintf((char *)json_upgrade_info,"motor upgrade start");
	if(handshake(id) != 1){
        LOG_I("%s(%d):handshake false",__FUNCTION__, id);
        sprintf((char *)json_upgrade_info,"handshake false");
        return RT_FALSE;
    }
   
	if(checkall(id,file_name) != 1){
        LOG_I("%s(%d):checkall false",__FUNCTION__, id);
        sprintf((char *)json_upgrade_info,"checkall false");
        return RT_FALSE;
    }
	if(binSend(id,file_name) != 1){
        LOG_I("%s(%d):binSend false",__FUNCTION__, id);
        sprintf((char *)json_upgrade_info,"binSend false");
        return RT_FALSE;
    }
    
	return RT_TRUE;
}

rt_bool_t cToUpgradeInfo(char *chp,int *version_year,int *version_date,char *file_name)
{
    char tem[8];
    char *p;
   // LOG_I("%s(%d):%s\r\n",__FUNCTION__,br, chp);
    rt_thread_mdelay(2000);
    memset(tem,0,sizeof(tem));
    p = strstr(chp,"version_year");
    if(!p)return RT_FALSE;
    p+=strlen("version_year");
    while(*p <= '0' || *p >='9')p++;
    memcpy(tem,p,4);
    *version_year = atoi(tem);
    
    LOG_I("%s(%version_year):%s  %d\r\n",__FUNCTION__,tem,*version_year);
    p = strstr(chp,"version_date");
    if(!p)return RT_FALSE;
    p+=strlen("version_year");
    while(*p <= '0' || *p >='9')p++;
    memcpy(tem,p,4);
    *version_date = atoi(tem);
    LOG_I("%s(%version_date):%s %d\r\n",__FUNCTION__,tem,*version_date);
    p = strstr(chp,"bin:");
    if(!p)return RT_FALSE;
    p+=strlen("bin:");
    strcpy(file_name,p);
    LOG_I("%s(bin):%s %s\r\n",__FUNCTION__,p,file_name); 
   
   return RT_TRUE;
}

rt_bool_t getUpgradeInfo(int *version_year,int *version_date,char *file_name)
{
    LOG_I("%s:web motor upgrade start...-----------------!",__FUNCTION__);
    FIL *fpp = rt_malloc(sizeof(FIL));
    malloc_inc++;
    //struct fsdata_file *fpw =  rt_malloc(sizeof(struct fsdata_file));
    char *chp;
    char *p;
    char tem[8];
    int i;
    if(f_open(fpp,"C:/upgrade.upg",FA_READ) == FR_OK){
            LOG_I("%s:(%s) open file ok",__FUNCTION__, "C:/upgrade.upg");
            UINT br = 0;
            int ch_len = f_size(fpp) + 10;
            chp = rt_malloc(ch_len);
            malloc_inc++;
            memset(chp,0,ch_len);
            FRESULT fr = f_read(fpp,chp,ch_len,&br);
            if(fr ==  FR_OK){
                strcpy(file_name,"C:/");
                if(!cToUpgradeInfo(chp,version_year,version_date,file_name + strlen("C:/"))){
                    f_close(fpp);
                    rt_free(fpp);
                    malloc_inc--;
                    rt_free(chp);
                    malloc_inc--;
                    return RT_FALSE;
                }
            }
            f_close(fpp);
            rt_free(fpp);
            malloc_inc--;
            rt_free(chp);
            malloc_inc--;
            
            return RT_TRUE;
     }
    
     rt_free(fpp);
        
     struct fsdata_file *fpw =  rt_malloc(sizeof(struct fsdata_file));
      malloc_inc++;                 
     if(webf_open(fpw,"/upgrade.upg",FA_READ) == FR_OK){
         LOG_I("%s:(%s) open file ok",__FUNCTION__, "/upgrade.upg");
          UINT br = 0;
         int ch_len = webf_size(fpw) + 10;
         chp = rt_malloc(ch_len);
         malloc_inc++;
         memset(chp,0,ch_len);    
         FRESULT fr = webf_read(fpw,chp,ch_len,&br); 
         if(fr ==  FR_OK){
                strcpy(file_name,"/");
                if(!cToUpgradeInfo(chp,version_year,version_date,file_name + strlen("/"))){
                    webf_close(fpw);
                    rt_free(fpw);
                    malloc_inc--;
                    rt_free(chp);
                    malloc_inc--;
                    return RT_FALSE;
                }
            }
            webf_close(fpw);
            rt_free(fpw);
            malloc_inc--;
            rt_free(chp);  
            malloc_inc--;            
    }
    rt_free(fpw);
    malloc_inc--;
	return RT_FALSE;
}

void steerUpgrade(WHEEL_STEER_def  *steerP,int upgrade_mode,int version_year,int version_date,char *file_name)
{
    if((((upgrade_mode == DF_MOTOR_UPGRADE_FORCE) || ((version_year * 10000 + version_date) > (steerP->drive_version_year * 10000 + steerP->drive_version_date)))
        && (upgrade_mode <= DF_MOTOR_UPGRADE_FORCE)) || (upgrade_mode == steerP->driveId)){
        json_upgrade_id = steerP->driveId;
        sprintf((char *)json_upgrade_info,"power ON");
        power_set_single_power(steerP->drive_power_pin,POWER_ON); 
        rt_thread_mdelay(1000);      
        LoadingThread(steerP->driveId,file_name);
        power_set_single_power(steerP->steering_power_pin,POWER_OFF); 
    }
    
   if((((upgrade_mode == DF_MOTOR_UPGRADE_FORCE) || ((version_year * 10000 + version_date) > (steerP->steering_version_year * 10000 + steerP->steering_version_date)))
        && (upgrade_mode <= DF_MOTOR_UPGRADE_FORCE)) || (upgrade_mode == steerP->steeringId)){
       json_upgrade_id = steerP->steeringId; 
       sprintf((char *)json_upgrade_info,"power ON");
         power_set_single_power(steerP->steering_power_pin,POWER_ON); 
        rt_thread_mdelay(1000);  
        LoadingThread(steerP->steeringId,file_name);
        power_set_single_power(steerP->steering_power_pin,POWER_OFF); 

    }   


}
void get_char_motot_id(int id,char *char_id)
{
    if(id == leftFront.steeringId)
        sprintf(char_id,"%s_S",leftFront.name);
    if(id == leftFront.driveId)
        sprintf(char_id,"%s_D",leftFront.name);
    
    if(id == rightFront.steeringId)
        sprintf(char_id,"%s_S",rightFront.name);
    if(id == rightFront.driveId)
        sprintf(char_id,"%s_D",rightFront.name);
    
    if(id == leftRear.steeringId)
        sprintf(char_id,"%s_S",leftRear.name);
    if(id == leftRear.driveId)
        sprintf(char_id,"%s_D",leftRear.name);
    
    if(id == rightRear.steeringId)
        sprintf(char_id,"%s_S",rightRear.name);
    if(id == rightRear.driveId)
        sprintf(char_id,"%s_D",rightRear.name);
    
}

rt_bool_t motorUpgrade(int upgrade_mode)
{
    int year;
    int date;
    char file_name[128];
    json_upgrade_id = 0;
    LOG_I("%s:%d\r\n",__FUNCTION__, upgrade_mode);
    getUpgradeInfo(&year,&date,file_name);
    LOG_I("%s:%d\r\n",__FUNCTION__, upgrade_mode);
    LOG_I("\t\tyear:%d\r\n", year);
    LOG_I("\t\tdate:%d\r\n", date);
    LOG_I("\t\tfile_name:%s\r\n", file_name);

    steerUpgrade(&leftFront,upgrade_mode,year,date,file_name);
    steerUpgrade(&rightFront,upgrade_mode,year,date,file_name);
    steerUpgrade(&leftRear,upgrade_mode,year,date,file_name);
    steerUpgrade(&rightRear,upgrade_mode,year,date,file_name);
    json_upgrade_id = -1;

	return RT_TRUE;
}

int getUpgradeJson(char *res,char * web_name)
{
    char key[16];
    int len = initJson(res,web_name);
    LOG_I("%s:malloc_inc: %d \r\n",__FUNCTION__, malloc_inc);
    if(json_upgrade_id > 0){
        get_char_motot_id(json_upgrade_id,key);
        len = addStrToJson(res,key,(char *)json_upgrade_info);
        return len;  
    }
    
    if(json_upgrade_id == 0){
        get_char_motot_id(leftFront.steeringId,key);
        len = addStrToJson(res,key,"power OFF");        
        get_char_motot_id(leftFront.driveId,key);
        len = addStrToJson(res,key,"power OFF");
        
        get_char_motot_id(rightFront.steeringId,key);
        len = addStrToJson(res,key,"power OFF");        
        get_char_motot_id(rightFront.driveId,key);
        len = addStrToJson(res,key,"power OFF");  
        
        get_char_motot_id(leftRear.steeringId,key);
        len = addStrToJson(res,key,"power OFF");  
        get_char_motot_id(leftRear.driveId,key);
        len = addStrToJson(res,key,"power OFF");  
        
        get_char_motot_id(rightRear.steeringId,key);
        len = addStrToJson(res,key,"power OFF");     
        get_char_motot_id(rightRear.driveId,key);
        len = addStrToJson(res,key,"power OFF");  
        

        
        return len;  
    }
    get_char_motot_id(leftFront.steeringId,key);
    len = addIntToJson(res,key,leftFront.steering_version_year * 10000 +leftFront.steering_version_date);
    get_char_motot_id(leftFront.driveId,key);
    len = addIntToJson(res,key,leftFront.drive_version_year * 10000 +leftFront.drive_version_date);
      
    get_char_motot_id(rightFront.steeringId,key);
    len = addIntToJson(res,key,rightFront.steering_version_year * 10000 +rightFront.steering_version_date);
    get_char_motot_id(rightFront.driveId,key);
    len = addIntToJson(res,key,rightFront.drive_version_year * 10000 +rightFront.drive_version_date);
      
    get_char_motot_id(leftRear.steeringId,key);
    len = addIntToJson(res,key,leftRear.steering_version_year * 10000 +leftRear.steering_version_date);
    get_char_motot_id(leftRear.driveId,key);
    len = addIntToJson(res,key,leftRear.drive_version_year * 10000 +leftRear.drive_version_date);
      
    get_char_motot_id(rightRear.steeringId,key);
    len = addIntToJson(res,key,rightRear.steering_version_year * 10000 +rightRear.steering_version_date);
    get_char_motot_id(rightRear.driveId,key);
    len = addIntToJson(res,key,rightRear.drive_version_year * 10000 +rightRear.drive_version_date);
   
    return len;  
}

rt_bool_t motorUpgradeCmdfun(const char *group,char *cmd)
{
    LOG_I("%s:%s %s \r\n",__FUNCTION__, group,cmd);
    json_upgrade_id = 0;
    if (strcmp(group, "upg")){
        LOG_I("%s:enter group \r\n",__FUNCTION__);
            return RT_FALSE;
          }
    if (!strcmp(cmd, "inc")){
        LOG_I("%s:enter inc \r\n",__FUNCTION__);
            enterMotorUpgrade(DF_MOTOR_UPGRADE_INC);
            return RT_TRUE;
          } 

    if (!strcmp(cmd, "force")){
        LOG_I("%s:enter force \r\n",__FUNCTION__);
           enterMotorUpgrade(DF_MOTOR_UPGRADE_FORCE);
            return RT_TRUE;
          } 
    
          
   if (!strcmp(cmd, "leftFront_S")){
        LOG_I("%s:%s single motor upgrade \r\n",__FUNCTION__,cmd);
           enterMotorUpgrade(leftFront.steeringId);
            return RT_TRUE;
          } 
   if (!strcmp(cmd, "leftFront_D")){
        LOG_I("%s:%s single motor upgrade \r\n",__FUNCTION__,cmd);
           enterMotorUpgrade(leftFront.driveId);
            return RT_TRUE;
          } 

          
          
   if (!strcmp(cmd, "rightFront_S")){
        LOG_I("%s:%s single motor upgrade \r\n",__FUNCTION__,cmd);
           enterMotorUpgrade(rightFront.steeringId);
            return RT_TRUE;
          } 

   if (!strcmp(cmd, "rightFront_D")){
        LOG_I("%s:%s single motor upgrade \r\n",__FUNCTION__,cmd);
           enterMotorUpgrade(rightFront.driveId);
            return RT_TRUE;
          } 

          
          
          
   if (!strcmp(cmd, "leftRear_S")){
        LOG_I("%s:%s single motor upgrade \r\n",__FUNCTION__,cmd);
           enterMotorUpgrade(leftRear.steeringId);
            return RT_TRUE;
          } 

   if (!strcmp(cmd, "leftRear_D")){
        LOG_I("%s:%s single motor upgrade \r\n",__FUNCTION__,cmd);
           enterMotorUpgrade(leftRear.driveId);
            return RT_TRUE;
          } 

          
          
          
   if (!strcmp(cmd, "rightRear_S")){
        LOG_I("%s:%s single motor upgrade \r\n",__FUNCTION__,cmd);
           enterMotorUpgrade(rightRear.steeringId);
            return RT_TRUE;
          } 

   if (!strcmp(cmd, "rightRear_D")){
        LOG_I("%s:%s single motor upgrade \r\n",__FUNCTION__,cmd);
           enterMotorUpgrade(rightRear.driveId);
            return RT_TRUE;
          } 

    return RT_FALSE;
}