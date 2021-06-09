/******************************************************************************

  Copyright (C), 2001-2011,UBT.CO.

 ******************************************************************************
  File Name     : bmsUpgrade.cpp
  Version       : Initial Draft
  Author        : marty.gong@ubtrobot.com
  Created       : 2018/8/22
  Last Modified :
  Description   : bmsupgrade function

******************************************************************************/

/*----------------------------------------------*
 * include files                           *
 *----------------------------------------------*/
#include <json/json.h>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include "include/bmsUpgrade.h"
/*----------------------------------------------*
 * external variables                           *
 *----------------------------------------------*/

/*----------------------------------------------*
 * external routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * internal routine prototypes                  *
 *----------------------------------------------*/

/*----------------------------------------------*
 * project-wide global variables                *
 *----------------------------------------------*/

/*----------------------------------------------*
 * module-wide global variables                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * constants                                    *
 *----------------------------------------------*/

/*----------------------------------------------*
 * macros                                       *
 *----------------------------------------------*/
#define BMS_FILE_CFG_PAHT "/home/atris/atris_app/firmware/bms/version_info.yaml"
#define BMS_FILE_IMAGE_PATH "/home/atris/atris_app/firmware/bms/"
#define BMS_MAX_PRIT_SIZE 256

#define IF_BMS_PACK_FIT_IN_CONDITION(channel_, condition_)               \
        (((package.head.id.channel & 0xff) == channel_) &&                         \
        (package.data[0] == condition_))

#define BMS_WAITE_TIME_IN_GET_QUE   1
#define BMS_MAX_RECEVIE_TIME 5000 
#define BMS_UPGRADE_TRANS_WAITE_TIME 6
#define BMS_UPGRADE_VER_GET_WAITE_TIME 3000
#define BMS_USLEEP_500MS (500 *1000)
#define BMS_USLEEP_1S (1000*1000)
#define BMS_RETRY_NUM 3

#define BMS_TRANS_BUS 1
#define BMS_UPGRADE_SEND_ID 0xe1
#define BMS_UPGRADE_RECV_ID 0xe2
#define BMS_VER_GET_ID 0xe1
#define BMS_VER_RECV_ID 0xe2

#define BMS_START_UP_CMD 0xff
#define BMS_START_UP_RECV_CMD 0xc3

#define BMS_SEND_TRANS_DATA_CMD 0xfe
#define BMS_RECV_TRANS_DATA_CMD 0xc3

#define BMS_END_UP_CMD  0xfd
#define BMS_END_UP_RECV 0xc1

#define BMS_REBOOT_CMD 0xfc
#define BMS_REBOOT_RECV 0xc4

#define BMS_VER_GET_CMD 0xcc
#define BMS_VER_RECV_CMD 0xcd

#define BMS_UPGRADE_MAX_PRIT_SIZE 256
#define GET_BMS_PACK_FROM_QUE() ((canPkg.getPackageFromQueue( &package,BMS_WAITE_TIME_IN_GET_QUE)))


//最新的yaml-cpp 0.5取消了运算符">>"，但是还是会有好多的旧代码  
//依旧在使用，所以重载下">>"运算符  
template<typename T>  
void operator >> (const YAML::Node& node, T& i)  
{  
  i = node.as<T>();  
}

BMSupgrade::BMSupgrade():canPkg("can0")
{

}

BMSupgrade::~BMSupgrade()
{

}

bool BMSupgrade::init()
{
    canPkg.init();
    return true;
}


void BMSupgrade::switchPktRecvier(bool on_)
{
   if(true == on_)
   {
        canPkg.packageGetRestart();
        log_info("bms:switch the pkg reciver on.");
   }
   else
   {
        canPkg.packgetGetPause();
        log_info("bms:switch the pkg reciver off.");
   }

}


bool BMSupgrade::getVersion(string & ver_)
{
    string ver_now_hd="";
    string ver_now_sf="";
    BMSCfg bms_cfg={0};
    BMS_VERSION_STRUCT struct_ver_now={0};
    char ch[BMS_MAX_PRIT_SIZE]={0};
    if(false == verGet(struct_ver_now,bms_cfg))
    {
        log_error("get ver false");
        return false;
    }
    
    sprintf(ch,"%d.%.2d",struct_ver_now.hardware_version_high,struct_ver_now.hardware_version_low);
    ver_now_hd = ch;
    sprintf(ch,"%d.%.2d",struct_ver_now.software_version_high,struct_ver_now.software_version_low);
    ver_now_sf = ch;
    char needUp = ( !(ver_now_hd == bms_cfg.ver_hard&&ver_now_sf !=bms_cfg.ver_soft))?'N':'Y';
    sprintf(ch,"version_hard:%s,version_now:%s,version_cfg:%s,needUp:%c",ver_now_hd.c_str(),ver_now_sf.c_str(),bms_cfg.ver_soft.c_str(),needUp);
    ver_ = ch;
    return true;
}
bool BMSupgrade::upgrade()
{
    log_info("start upgrade BMS=====>start.");
    bool isNeedUp = true;
    BMS_VERSION_STRUCT ver_now={0};
    BMSCfg bms_cfg;
    if(false == verCompare(ver_now,bms_cfg,isNeedUp))
    {
        return false;        
    }
    //check is needup   modified by marty.gong@ubtrobot.com:2018-8-02-21:11:22 
    if(false == isNeedUp) return true;
    if(false == startUpgrade(bms_cfg.bin_path))
    {
        return false;
    }

    if(false == versionCheck(bms_cfg))
    {
        log_error("version check error after upgrading.");
        return false;
    }
    log_info("upgrade BMS successful.");
    return true;
}




/**************************private************************************/

bool BMSupgrade::verRunRead(BMS_VERSION_STRUCT &ver_now)
{
    if(false == sendPacketToCanBusWithoutPacket(BMS_TRANS_BUS,BMS_VER_GET_ID, BMS_VER_GET_CMD))
    {
        return false;
    }

    BMS_VERSION_STRUCT *p =NULL;
    if(NULL == (p = (BMS_VERSION_STRUCT*)getPacktFromCanBus(BMS_TRANS_BUS,BMS_VER_RECV_ID, BMS_VER_RECV_CMD,BMS_UPGRADE_VER_GET_WAITE_TIME)))
    {
        return false;
    }

    memcpy(&ver_now,p,sizeof(BMS_VERSION_STRUCT));

    return true;
}


bool BMSupgrade::verCfgRead(BMSCfg & cfg)
{
    ifstream fin(BMS_FILE_CFG_PAHT);
    if(!fin)
    {
        log_error("can not open cfg file.");
        return false;
    }
	YAML::Node doc;
    doc = YAML::Load(fin);

    if(doc[0]["bmscfg"])
    {
        if(doc[0]["forceUp"])
        {
            string temp_force;
            doc[0]["forceUp"] >> temp_force;
            cfg.is_force = (("yes" == temp_force) || ("true" == temp_force) )?true:false;
        }
        else
        {
            cfg.is_force = false;
        }

        if(doc[0]["hard_version"])
        {
            doc[0]["hard_version"] >> cfg.ver_hard;
        }
        else
        {
            cfg.ver_hard = "";
        }
        if(doc[0]["soft_version"])
        {
            doc[0]["soft_version"] >> cfg.ver_soft;
        }
        else
        {
            cfg.ver_soft = "";
        }
        if(doc[0]["bin_file"])
        {
            doc[0]["bin_file"] >> bin_file_name;
            cfg.bin_path = BMS_FILE_IMAGE_PATH + bin_file_name;
        }
        else
        {
            cfg.bin_path = "/home/atris/atris_app/firmware/bms/bms_image.bin";
        }

        if(doc[0]["pid"])
        {
            string pid_ ;
            doc[0]["pid"] >> pid_ ;
            cfg.pid_ = atoi(pid_.c_str());
        }
        else
        {
            cfg.pid_ = 0;
        }
    }
    fin.close();
    return true;
}


bool BMSupgrade::verGet(BMS_VERSION_STRUCT & ver_now, BMSCfg & cfg)
{
    if(false == verRunReadwithRetry(ver_now,BMS_RETRY_NUM,BMS_USLEEP_1S))
    {
        log_error("read the running software version error.");
        return false;
    }

    if(false == verCfgRead(cfg))
    {
        log_error("read the cfg software version error.");
        return false;
    }
    return true;

}
bool BMSupgrade::verCompare(BMS_VERSION_STRUCT &ver_now,BMSCfg &cfg,bool &isNeedUp)
{
    string  ver_now_sf="";
    string  ver_now_hd="";
    char ch[BMS_MAX_PRIT_SIZE] = {0};

    isNeedUp = true;
    
    if(false == verGet(ver_now,cfg))
    {
        snprintf(ch, sizeof(ch), "get version false when upgrading.");
        log_error(ch);
        return false;
    }

    sprintf(ch,"%d.%.2d",ver_now.hardware_version_high,ver_now.hardware_version_low);
    ver_now_hd = ch;
    sprintf(ch,"%d.%.2d",ver_now.software_version_high,ver_now.software_version_low);
    ver_now_sf = ch;

    int lpid_ = ver_now.pid_high * 256 + ver_now.pid_low;

    if(0 == lpid_)
    {
        snprintf(ch, sizeof(ch), "the bms device is in bios state.need upgrade force!.");
        log_info(ch);
        return true;
    }
    
    if(cfg.pid_ != lpid_)
    {
        snprintf(ch, sizeof(ch), "the running pid: %d is not same with the cfg pid: %d.",lpid_,cfg.pid_);
        log_info(ch);
        isNeedUp = false;
        return true;
    }
    if(cfg.ver_hard != ver_now_hd)
    {
        snprintf(ch, sizeof(ch), "the running hard: %s is not same with the cfg hardware: %s.",ver_now_hd.c_str(),cfg.ver_hard.c_str());
        log_info(ch);
        isNeedUp = false;
        return true;
    }

    if((cfg.ver_soft == ver_now_sf) &&  (false == cfg.is_force) )
    {
        snprintf(ch, sizeof(ch), "there is no need upgrading.====>soft version is:%s ",ver_now_sf.c_str());
        log_info(ch);
        isNeedUp = false;
        return true;
    }
    snprintf(ch, sizeof(ch), "the running hard: %s,soft:%s, config hard:%s sotf:%s",ver_now_hd.c_str(),ver_now_sf.c_str(),cfg.ver_hard.c_str(),cfg.ver_soft.c_str());
    log_info(ch);
    
    return true;
    
}

bool BMSupgrade::startUpgrade(string & bin_path)
{
    unsigned char *update_data = NULL;
    char ch[BMS_MAX_PRIT_SIZE] = {0};
    std::string str;

    //read file
    int fileLength = getDataFromFile(bin_path.c_str(), (char **) &update_data);
    if (fileLength <= 0) {
        log_error("read update file error");
        return false;

    }
    sprintf(ch, "%d", fileLength);
    str = ch;
    log_info("read update file length: %s", str.c_str());
    
    if(false == upgrading(update_data,fileLength))
    {
        log_warn("upgrading failed temp to retry.");
        //retry 3times;
        for(int i =0; i < 3; i++)
        {
            if(false == sendRebootTransCmd())
            {
                log_error("send reboot trans data faild.");
                usleep(BMS_USLEEP_500MS);
                continue;
            }
            sleep(8);
            if(false == upgrading(update_data,fileLength))
            {
               snprintf(ch, sizeof(ch), "retry upgrade false. ==%d==",i);
               log_error(ch);
                    
            }
            else
            {
                free(update_data);
                return true;
            }
            
        }
        free(update_data);
        return false;
    
    }
    free(update_data);
    return true;
}

bool BMSupgrade::versionCheck(BMSCfg &cfg)
{
    BMS_VERSION_STRUCT ver_now;
    if(false == verRunReadwithRetry(ver_now,BMS_RETRY_NUM,BMS_USLEEP_1S))
    {
        log_error("read the running software version error.");
        return false;
    }
   char ch[BMS_MAX_PRIT_SIZE] = {0};
   string ver_now_sf="";
   sprintf(ch,"%d.%.2d",ver_now.software_version_high,ver_now.software_version_low);
   ver_now_sf = ch;

   if(ver_now_sf != cfg.ver_soft)
   {
        snprintf(ch, sizeof(ch), "version recheck error.now:%s=====cfg:%s",ver_now_sf.c_str(),cfg.ver_soft.c_str());
        log_error(ch);
        return false;
   }
   return true;
}


bool BMSupgrade::upgrading(unsigned char* updata_data,int fileLength)
{
    
   
    char ch[BMS_MAX_PRIT_SIZE] = {0};

    std::string str;


    //upgrade

    if(false == sendUpgradeStartCmdWithRetry(BMS_RETRY_NUM,BMS_USLEEP_1S))
    {
        log_error("send start upgrade cmd faild.");
        return false;
    }
    sleep(8);
    int address = 0;
    int percent = -1; 
    int size = 7;
    unsigned char tempBuff[7]= {0};
    initPercentPara();
    while(address < fileLength)
    {

        size = 0;
        memset(tempBuff,0,7);
        for(int j =0; j <7;j++)
        {
            
            if(address < fileLength)
            {

                // 遇到回车换行下一个阶段发送
                if((updata_data[address] == 0x0d))
                {
                   
                   address +=2;
                   //每次发送必须保证有数据
                   if(0 == j)
                   {
                        tempBuff[j] = updata_data[address++];
                   }
                   else
                   {
                        break;
                   }
                   
                }
                else
                {
                    tempBuff[j]= updata_data[address++];
                }
                size++;
            }
        }
        //增加重试机制
        bool tran_flag = false;  
        for(int tran_i = 0; tran_i < 3; tran_i++)
        {

             if(true == sendUpgradedata(BMS_TRANS_BUS,BMS_UPGRADE_SEND_ID,BMS_UPGRADE_RECV_ID,tempBuff,size, BMS_UPGRADE_TRANS_WAITE_TIME))
            {
                tran_flag = true;
                break;
            }
            usleep(BMS_USLEEP_500MS);
        }
        if(false == tran_flag)
        {
            log_error("send upgrade data failed.");
            return  false;
        }
                        


         sprintf(ch, "address:%d  %d%%", address, address * 100 / fileLength);
         str = ch;
        if (percent != (address * 100 / fileLength)) 
        {
            percent = address * 100 / fileLength;

            if ((percent > 0) && (true == printPercent((unsigned int) percent))) 
            {
                log_info("programming %s", str.c_str());
            }
        }

        if (address >= fileLength)
            break;
        
    }

    // tell end
    if(false == sendUpgradeEndCmdwithRetry(BMS_RETRY_NUM,BMS_USLEEP_1S))
    {
        log_error("send upgrade data finish cmd faild.");
        return false;
    }
    
   sleep(10);
    return true;
}

bool BMSupgrade::printPercent(unsigned int percent)
{
    bool flag = false;

    if ((percent < BMS_PRIT_PERCNT::print_percent_30_) && (false == upgrade_percent[0])) //
    {
        upgrade_percent[0] = true;
        flag = true;
    }
    else if ((BMS_PRIT_PERCNT::print_percent_30_ <= percent && percent < BMS_PRIT_PERCNT::print_percent_60_) && (false == upgrade_percent[1])) {
        upgrade_percent[1] = true;
        flag = true;
    }
    else if ((BMS_PRIT_PERCNT::print_percent_60_ <= percent && percent < BMS_PRIT_PERCNT::print_percent_100_) && (false == upgrade_percent[2])) {
        upgrade_percent[2] = true;
        flag = true;
    }
    else if ((BMS_PRIT_PERCNT::print_percent_100_ == percent) && (false == upgrade_percent[3])) {
        upgrade_percent[3] = true;
        flag = true;

    }
    else // the percent can not in this case;
    {
        return false;
    }
    return flag;
}

void BMSupgrade::initPercentPara()
{
    int i = 0;

    for (i = 0; i < BMS_UP_GRADE_PERCENT_SETION; i++) {
        upgrade_percent[i] = false;
    }

}


bool BMSupgrade::sendRebootTransCmd()
{
     if(false == sendPacketToCanBusWithoutPacket(BMS_TRANS_BUS,BMS_UPGRADE_SEND_ID,BMS_REBOOT_CMD))
    {
        return false;
    }

     if(NULL == getPacktFromCanBus(BMS_TRANS_BUS,BMS_UPGRADE_RECV_ID,BMS_REBOOT_RECV,BMS_MAX_RECEVIE_TIME))
     {
        return false;
     }
    return true; 
}

bool BMSupgrade::sendUpgradeStartCmd()
{
    if(false == sendPacketToCanBusWithoutPacket(BMS_TRANS_BUS,BMS_UPGRADE_SEND_ID,BMS_START_UP_CMD))
    {
        return false;
    }

     if(NULL == getPacktFromCanBus(BMS_TRANS_BUS,BMS_UPGRADE_RECV_ID,BMS_START_UP_RECV_CMD,BMS_MAX_RECEVIE_TIME))
     {
        return false;
     }
    
    return true;
}

bool BMSupgrade::sendUpgradeEndCmd()
{
     if(false == sendPacketToCanBusWithoutPacket(BMS_TRANS_BUS,BMS_UPGRADE_SEND_ID,BMS_END_UP_CMD))
    {
        return false;
    }

     if(NULL == getPacktFromCanBus(BMS_TRANS_BUS,BMS_UPGRADE_RECV_ID,BMS_END_UP_RECV,BMS_MAX_RECEVIE_TIME))
     {
        return false;
     }
    
    return true;
}

int BMSupgrade::getDataFromFile(const char * name, char ** buffer)
{
    //int lenth;
    if (name == NULL)
        return 0;

    FILE * fp = fopen(name, "rt");

    if (fp == NULL) 
	{
        printf("GetDataFromFile failed to open file:%s\n", name);
        return 0;
    }
    else 
	{

        int size_;

        fseek(fp, 0, SEEK_END);
        size_ = ftell(fp);
        
        *buffer = new char[size_];
        memset(*buffer,0,size_);

        fseek(fp, 0, SEEK_SET);

        //int ret =  fread(buffer,size,1,fp);
        fread(*buffer, size_, 1, fp);
        fclose(fp);
        return size_;
    }
}
bool BMSupgrade::sendUpgradedata(unsigned int bus, unsigned int channel_send,unsigned int channel_recv,unsigned char * data,int size,int timeWaite)
{
    
    clockid_t time_end;

    if(NULL == data) return false;

    BMS_PACKAGE_STRUCT data_;
    memset(&data_,0,sizeof(data_));
    data_.cmd = BMS_SEND_TRANS_DATA_CMD;
    memcpy(&data_.data,data,size);

   // printf(" %0x %0x %0x %0x %0x %0x %0x",data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
    
	if(false == sendPacketToCanBusWithUpgradePacket((unsigned int) bus, (unsigned int) channel_send,data_,size))
	{
		return false;
	}


    usleep(1000 * timeWaite);
    time_end = clock() + 60000;                      //60ms

    while (true) 
	{
        if (GET_BMS_PACK_FROM_QUE())
		{
            if (IF_BMS_PACK_FIT_IN_CONDITION(channel_recv, BMS_RECV_TRANS_DATA_CMD))
			{
                return true;
                // char ch[128];
                //std::string str;
                // sprintf(ch,"address:%d  data:%.8x,redata:%.8x",p->body.programming_response.next_address,code,p->body.programming_response.pre_code);
                // str = ch;
                // progressBar("programming " + str);
            }
        }

        if (clock() > time_end) 
		{
            break;
        }

    }

    return false;
}


//retry cmd
bool BMSupgrade::sendUpgradeEndCmdwithRetry(unsigned int retry_num,unsigned long sleep_time)
{
    for(unsigned int i = 0 ;i < retry_num; i++)
    {

        if(true == sendUpgradeEndCmd())
        {
            return true;
        }
        usleep(sleep_time);
    }
    return false;

}

bool BMSupgrade::sendUpgradeStartCmdWithRetry(unsigned int retry_num,unsigned long sleep_time)
{
     for(unsigned int i = 0 ;i < retry_num; i++)
    {

        if(true == sendUpgradeStartCmd())
        {
            return true;
        }
        usleep(sleep_time);
    }
    return false;
}


bool  BMSupgrade::verRunReadwithRetry(BMS_VERSION_STRUCT &ver_now,unsigned int retry_num,unsigned long sleep_time)
{
      for(unsigned int i = 0 ;i < retry_num; i++)
    {

        if(true == verRunRead(ver_now))
        {
            return true;
        }
        usleep(sleep_time);
    }
    return false;
}
bool BMSupgrade::sendPacketToCanBusWithoutPacket(unsigned int bus, unsigned int channel,unsigned char cmd)
{
    if (0 == channel)return false;

    package.head.id.can = bus;
    package.head.id.channel = channel;
    package.head.id.mode = 0;
    package.data[0] = cmd;
    package.head.id.size = 0;

    canPkg.packageToCanBus(package);
	return true;
}


bool BMSupgrade::sendPacketToCanBusWithUpgradePacket(unsigned int bus, unsigned int channel,BMS_PACKAGE_STRUCT &data,int size)
{	

	if (0 == channel) return false;


    package.head.id.can = bus;
    package.head.id.channel = channel;
    package.head.id.mode = 0;
    package.head.id.size = size;
    

    mempcpy(package.data, &data, sizeof(BMS_PACKAGE_STRUCT));

    
	canPkg.packageToCanBus(package);

    return true;
	
}

u8* BMSupgrade::getPacktFromCanBus(int bus,int channel,int condition,int timeWaite)
{
    //默认已经清空了队列   modified by marty.gong@ubtrobot.com:2018-1-09-11:34:50 

    struct  timeval start;
    struct  timeval end;
    unsigned  long diff = 0;
    unsigned  long time_end = timeWaite*1000; //timeWaite(ms)
    gettimeofday(&start,NULL);
	while(true)
	{

		 if(true == canPkg.getPackageFromQueue( &package,BMS_WAITE_TIME_IN_GET_QUE))
		{
            
             
             if(1 && package.data[0] >= 0xbb)
    		 {
                printf("in size:%d mode:%d,channel:%.2x bus:%d==data =%.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x \n",package.head.id.size,package.head.id.mode,package.head.id.channel,package.head.id.can,
    				   package.data[0],package.data[1],package.data[2],package.data[3],package.data[4],package.data[5],package.data[6],package.data[7]);
             }
            if (IF_BMS_PACK_FIT_IN_CONDITION(channel,condition)) 
			{
				u8 * p =  &package.data[1];
		
				return p;
			}
		
		}
		gettimeofday(&end,NULL);
		diff = 1000000 * (end.tv_sec-start.tv_sec)+ end.tv_usec-start.tv_usec;
		if (diff > time_end)
        break;
	}
    return NULL;
    
}


