#include "include/datarouterJointSet.h"
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <time.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <dirent.h>
#include "include/bmsUpgrade.h"
#include "gs_upgrade/gs_upgrade.h"


#define CAN_UP_DEBUG           false
#define UPGRADE_VERSION_FILE   "/tmp/upgradeVersion"
#define UPGRADE_COMP_FILE 	   "/tmp/cruiser/firmware/compare_version"
#define UPGRADE_FIRMVIRE_FILE  "/tmp/cruiser/firmware/version"
#define UPGRADE_FLAG_FILE   "upgradeStatus"
#define UPGRADE_SOFT_THRESHOLD_NONE "0"
#define UPGRAXE_GET_SUBVER_LEN 8
#define UPGRADE_SETTING_FILE "/setting.yaml"
#define UPGRADE_FOLDER "/home/atris/atris_app/firmware"
#define UPGRADE_MAX_CHECK_LOWPOWER_CIRCLE 20
#define BOTTOM_POWER_ID 0x69
//#define UPGRADE_SHARE_MEM_KEY 0x9527
#define CHANNEL_IS_CHASIS(channel_) (0x21 == channel_ || 0x22 == channel_ || 0x23 == channel_) 
#define DF_UPDATE_SIZE (64*1024)

#define SERVO_NUM	14

#define GET_INFO_ERRRO_OUT_PUT(ch,Pupgrade,log_,error_list,nor_upgraded_str)\
{\
    std::cout << (ch) << std::endl;\
    log_error((ch));\
    usleep(1000*1);\
    sprintf((ch),"id:%x;",(Pupgrade)->channel);\
    (nor_upgraded_str) += ch;\
    (error_list).push_back(ch);\
}

#define NOR_UPGRADE_OUT_PUT(ch,upgrade,log_,nor_upgradede_str)\
{\
    std::cout << (ch) << std::endl;\
    log_info((ch));\
    usleep(1000*1);\
    sprintf((ch),"id:%x;",(upgrade).channel);\
    (nor_upgradede_str) += ch;\
}
#define UPGRADE_OK_LIST_PRAM_SET(upgrade,product,upok_list,time_start) \
{\
     CHEKVSION up_ok;\
    (up_ok).bus = (upgrade).bus;\
    (up_ok).channel = (upgrade).channel;\
    (up_ok).name = (upgrade).name;\
    (up_ok).sotftversion = (product).software_version;\
    (up_ok).time = time(NULL)-(time_start);\
    (upok_list).push_back(up_ok);\
}

//最新的yaml-cpp 0.5取消了运算符">>"，但是还是会有好多的旧代码  
//依旧在使用，所以重载下">>"运算符  
template<typename T>  
void operator >> (const YAML::Node& node, T& i)  
{  
  i = node.as<T>();  
}




bool SOFTWARE_RECHECK_CONTROL = true;
bool SOFTWARE_VERSION_CONTROL = true;

dataRouterJointSet::dataRouterJointSet()
{
}

dataRouterJointSet::~dataRouterJointSet()
{
	
}


bool dataRouterJointSet::init(void)
{
    initCommandList();
	upgrade_p_ = new upGrade(boost::bind(&dataRouterJointSet::updateProgressBar,this,_1));
    setUpgradeFolder(UPGRADE_FOLDER);
    isloadcfg = false;
	return true;
}

void dataRouterJointSet::packetReciveSwitch(bool on_)
{
    if(upgrade_p_)
    {
        upgrade_p_->swtichPktRecvier(on_);
    }
}
std::string dataRouterJointSet::upgradeMcu(bool isForce,bool isAll, unsigned int channel,bool &version_get_flag,std::string &failed_mcu)
{
    
        char ch[UPGRADE_MAX_PRIT_SIZE]={0};
        std::string nor_upgraded_str;
        std::string _check_result;
        std::vector<CHEKVSION>upok_list;
        std::vector<std::string> error_list;


        upok_list.clear();
        error_list.clear();
        unsigned int inc = 0;
        char temp_failed_muc[10]={0};
        version_get_flag = true;
       //初始化   modified by marty.gong@ubtrobot.com:2018-1-30-14:56:42 
        _check_result = loadYaml();
        if(compare(_check_result,"error"))
        {
            
            failed_mcu = "e1 ";//配置读取失败标记
            log_error("load config file failed.");  
            usleep(1000*1);
            return _check_result;
        }
        //升级开始清除上报多余信息,升级结束后必须恢复mcu
        SetMcuReportTimer(0);
        time_t start_time = time(NULL);
    
        log_info("upgrade  start\n");
        usleep(1000*1);
        BOOST_FOREACH(UPGRADE upgrade,upgrade_list_)
       {
    
            if(false == isAll && upgrade.channel != channel) continue;
            //升级前开启升级状态   modified by marty.gong@ubtrobot.com:2018-1-25-14:37:4 
            setUpgradeStatus(upgrade.channel,true);
            time_t start_time_one = time(NULL);//记录每一个部件的起始时间；
            snprintf(ch, sizeof(ch), "upgrade name:%s,channel:%x,bus:%d\n",upgrade.name.c_str(),upgrade.channel,upgrade.bus);
            log_info(ch);
            usleep(1000*1);

             //补丁，如果是电源板和传感器板，需要开启上报电源频率   modified by marty.gong@ubtrobot.com:2017-10-23-10:7:24 
            StartPowerAndSensorReportTImer(upgrade.bus,upgrade.channel);
            
            VERSION_STRUCT version_struct ;
            PRODUCT_CODE_STRUCT product_struct;
 
            if(false == getBaseInfo(version_struct,product_struct,&upgrade,&error_list,nor_upgraded_str))
            {
                version_get_flag = false;
                sprintf(temp_failed_muc," %x",upgrade.channel);
                failed_mcu += temp_failed_muc;
                //本次升级结束，重新关闭上报频率   modified by marty.gong@ubtrobot.com:2017-10-23-10:29:16 
                 StopPowerAndSensorReportTImer(upgrade.bus,upgrade.channel);
                 
                 continue;
            }

            BOOST_FOREACH(PRODUCT product,product_list_)
            {

                std::string hardware_version;
                std::string software_version;

                if((upgrade.driver != product.product) ||(product.pid != product_struct.pid) || (product.vid != product_struct.vid))continue;

                //get version   modified by marty.gong@ubtrobot.com:2018-1-30-10:26:11 
                if(false == versionProcess(hardware_version,software_version,&version_struct,upgrade.channel))continue;
                
                if(false == checkHardWareVersion(hardware_version,product.hardware_version))
                {
                    sprintf(ch,"version check: %s,channel: %x,bus:%d your hardware dose not support upgrading.",upgrade.name.c_str(),upgrade.channel,upgrade.bus);

                    NOR_UPGRADE_OUT_PUT(ch,upgrade,log_ ,nor_upgraded_str);
                    //补丁本次升级结束，重新关闭上报频率，注意电机有两个升级文件，不能这么做   modified by marty.gong@ubtrobot.com:2017-10-23-10:29:16 
                    StopPowerAndSensorReportTImer(upgrade.bus,upgrade.channel);
                    setUpgradeStatus(upgrade.channel,false);
                    continue;
                }

                if(false == upgradeCheckSoftVersion(&product_struct,&product,software_version,isForce))
                {
                     sprintf(ch,"version check: %s,channel:%x,bus:%d  your currunt version no need upgrading.",upgrade.name.c_str(),upgrade.channel,upgrade.bus);
                     NOR_UPGRADE_OUT_PUT(ch,upgrade,log_ ,nor_upgraded_str); 
                      //补丁本次升级结束，重新关闭上报频率   ，注意 
                      StopPowerAndSensorReportTImer(upgrade.bus,upgrade.channel);
                      setUpgradeStatus(upgrade.channel,false);
                      continue;
                }
                
                std::string update_file = folder_ + "/" + product.bin_file;
                sprintf(ch,"%d",(inc++)*100/(upgrade_list_.size()));
                upgrading_product_ = ch;
                upgrading_product_ += upgrade.name;
                
                if(false == upgradeUpgrading(upgrade.bus,upgrade.channel,product.vid,product.pid,hardware_version,update_file))
                {
                    snprintf(ch, sizeof(ch), "upgrade error:%s,channel:%x,bus:%d\n",upgrade.name.c_str(),upgrade.channel,upgrade.bus);
                    log_info(ch);
   
                    //恢复上报频率   modified by marty.gong@ubtrobot.com:2017-10-19-16:26:51 
                    SetMcuReportTimer(-1);
                    //升级失败关闭升级状态   modified by marty.gong@ubtrobot.com:2018-1-25-15:6:20 
                    setUpgradeStatus(upgrade.channel,false);
                    sprintf(temp_failed_muc," %x",upgrade.channel);
                    failed_mcu += temp_failed_muc;
                    return ch;
                }
                UPGRADE_OK_LIST_PRAM_SET(upgrade,product,upok_list,start_time_one);
                snprintf(ch, sizeof(ch), "upgrade ok:%s,channel:%x,bus:%d\n",upgrade.name.c_str(),upgrade.channel,upgrade.bus);
                log_info(ch);
                 //本次升级结束，重新关闭上报频率   modified by marty.gong@ubtrobot.com:2017-10-23-10:29:16 
                StopPowerAndSensorReportTImer(upgrade.bus,upgrade.channel); 
                setUpgradeStatus(upgrade.channel,false);
                break;
    
    
            }
    
        }
    
        if(false == upgradeSoftwarerecheck(&upok_list,failed_mcu))
        {
             snprintf(ch, sizeof(ch), "SoftWare recheck  error!\n");
             log_error(ch);
             usleep(1000*1);
             //恢复上报频率   modified by marty.gong@ubtrobot.com:2017-10-19-16:26:51 
             SetMcuReportTimer(-1); 
             return "SoftWare recheck  error!";
        }
    
       // 处理升级信息  modified by marty.gong@ubtrobot.com:2018-1-30-14:55:46 
       outPutInfoAftrupgrade(&start_time,&upok_list,&error_list,nor_upgraded_str);
        
       //恢复上报频率   modified by marty.gong@ubtrobot.com:2017-10-19-16:26:51 
       SetMcuReportTimer(-1);
       log_info("can mcu upgrade finish.");
       return "ok";
}

bool dataRouterJointSet::getMcuVersion(int bus, int channel, string &ver)
{
    char ch[UPGRADE_MAX_PRIT_SIZE]={0};

    //load cfg file   modified by marty.gong@ubtrobot.com:2018-8-28-20:38:33 

    if(false == isloadcfg)
    {
         if(compare(loadYaml(),"error"))
        {
            return false;
        }
        
    }
    string hardware_version,software_version,software_version_config;
    bool get_right_config = false;
    
    
    //get ver   modified by marty.gong@ubtrobot.com:2018-8-28-20:38:47 
    
    BOOST_FOREACH(UPGRADE upgrade,upgrade_list_)
    {
        if(upgrade.channel != (unsigned int)channel) continue;

        VERSION_STRUCT *version_struct_p = upgradeGetVersion(upgrade.bus,upgrade.channel);
        if(!version_struct_p)return false;
        if(false == versionProcess(hardware_version,software_version,version_struct_p,upgrade.channel))return false;
         PRODUCT_CODE_STRUCT *product_struct_p = upgradeGetProductCode(upgrade.bus,upgrade.channel);
        if(!product_struct_p)return false;

        char isUp='N';

        BOOST_FOREACH(PRODUCT product,product_list_)
        {
            if((upgrade.driver != product.product))continue;

            get_right_config = true;
            software_version_config = product.software_version;
            //ver   modified by marty.gong@ubtrobot.com:2018-8-28-20:43:3 
            if(hardware_version == product.hardware_version)
            {
                if((software_version != product.software_version) && (software_version > product.software_upgrade_threshold))
                {
                    isUp='Y';
                    break;
                }
            }
        }
        if(true == get_right_config)
        {
            sprintf(ch,"hardware:%s,sotfware:(%s)%s,mode:%c,%c",hardware_version.c_str(),software_version.c_str(),software_version_config.c_str(),product_struct_p->mode,isUp);
            ver =ch;
            return true;
        }
        
    }
    return false;



}


bool dataRouterJointSet::configServerSrv(std::string &req,std::string &resq)
{

   char c;
   char ch[22]={0};
   std::string tem_command;
   std::string command;
    c = (char)atoi(req.c_str());
    sprintf(ch,"%c",c);
    

   switch(c)
   {
        case 27:
             config_command_ = "";
             resq = "\n";
             return true;
        case  127:
              tem_command = config_command_;
              command = config_command_;
             do{

                 tem_command = tem_command.substr(0,tem_command.size()-1);
                 command_Completion(tem_command);
                 if(config_command_.size() <= 0)break;
             }while(command == config_command_);  
             break;
        case  '\t':
             resq = command_Completion(config_command_);
             resq = command_Completion(config_command_);
             return true;
         default:
            config_command_ += ch;
            break;
   }
   if(c != '\n' )
   {
             resq = command_Completion(config_command_);
             resq = command_Completion(config_command_);
             resq = config_command_;
             return true;
    }

   else 
   {
         resq =config_command_ ;
         resq += runCommand(config_command_);// + "\n";
         config_command_ = "";
         return true;
    }


}


bool dataRouterJointSet::productVersionOutput()
{
	struct UPVER ver_pre;
    struct UPVER ver_now;
    u8 ret_to_file[2] = {0,1};
    bool ret_flag = false;
    std::string upgrade_name;

	if(false == create_multi_dir(UPGRADE_VERSION_FILE))return ret_flag;
	if(false == create_multi_dir(UPGRADE_COMP_FILE))return ret_flag;
	if(false == create_multi_dir(UPGRADE_FIRMVIRE_FILE))return ret_flag;
    std::ofstream of(UPGRADE_VERSION_FILE);
	std::ofstream of_c(UPGRADE_COMP_FILE);
	std::ofstream of_f(UPGRADE_FIRMVIRE_FILE);

    if("ok" != loadYaml())
    {
		std::cout<<"load yaml failed!"<<std::endl;
		if(of.is_open())
		{
			of.write((char*)&ret_to_file[0],sizeof(char));
			of.close();
		}
		if(of_c.is_open())
		{
			of_c.close();
		}
		if(of_f.is_open())
		{
			of_f.close();
		}
			return  false;
	}
	memset(&ver_pre,0,sizeof(struct UPVER));
    memset(&ver_now,0,sizeof(struct UPVER));

	BOOST_FOREACH(UPGRADE upgrade,upgrade_list_)
	{
		 memset(&ver_now,0,sizeof(struct UPVER));

         ver_now.bus = upgrade.bus;

         ver_now.channel = upgrade.channel;


         strncpy(ver_now.name,upgrade.name.c_str(),upgrade.name.length());
         strncpy(ver_now.driver,upgrade.driver.c_str(),upgrade.driver.length());


		 if(0 == strcmp(ver_now.driver,ver_pre.driver))//share the same version
         {
            strncpy(ver_now.software_version,ver_pre.software_version,sizeof(ver_pre.software_version)-1);
			strncpy(ver_now.hardware_version,ver_pre.hardware_version,sizeof(ver_pre.hardware_version)-1);
			strncpy(ver_now.software_upgrade_threshold,ver_pre.software_upgrade_threshold,sizeof(ver_pre.software_upgrade_threshold)-1);

			if(true == 
			productVersionCompare(ver_now.bus,ver_now.channel,ver_now.name,ver_now.hardware_version,ver_now.software_version,ver_now.software_upgrade_threshold,&of_c,&of_f))
			{
				ret_flag = true;
                upgrade_name += upgrade.name;

			}
			memcpy(&ver_pre,&ver_now,sizeof(struct UPVER));

			
		 }
		 else
		 {

			 BOOST_FOREACH(PRODUCT product,product_list_)
             {

                 if(ver_now.driver == product.product)
                    {

                        //专门为电机版本制定规则   modified by marty.gong@ubtrobot.com:2017-9-25-11:31:23 
                         if(true == upgrade_p_->idIsChassis(upgrade.channel))
                         {
                                printf("upgrade.channel is %x ver_now.driver is %s, product.product is %s\n ",upgrade.channel,
                                ver_now.driver,product.product.c_str());
                                if(true == chassisVersionCompare(ver_now.bus,ver_now.channel,ver_now.name,ver_now.driver,ver_now.hardware_version,ver_now.software_version,ver_now.software_upgrade_threshold,&of_c,&of_f))
                                {
                                    ret_flag = true;
                                    upgrade_name += upgrade.name;
                                }
                         }
                         else
                         {

                              strncpy(ver_now.software_version,product.software_version.c_str(),product.software_version.length());
                              strncpy(ver_now.hardware_version,product.hardware_version.c_str(),product.hardware_version.length());
                              strncpy(ver_now.software_upgrade_threshold,product.software_upgrade_threshold.c_str(),product.software_upgrade_threshold.length());
                              // write file
                             
                             if(true == 
                             productVersionCompare(ver_now.bus,ver_now.channel,ver_now.name,ver_now.hardware_version,ver_now.software_version,ver_now.software_upgrade_threshold,&of_c,&of_f))
                                 {
                                     ret_flag = true;
                                     upgrade_name += upgrade.name;
                             
                                 }
                         }
                         memcpy(&ver_pre,&ver_now,sizeof(struct UPVER));
                         break;
                    }
             }


		 }
		 
	}

	if(of.is_open())
	{

	  if(true == ret_flag)
	  {
          //of.write("upgrade:no",sizeof("upgrade:no"));
            of << "upgrade:yes" << std::endl;
            std::cout<< upgrade_name.c_str() <<std::endl;
	  }
	  else
	  {
          //of.write("upgrade:yes",sizeof("upgrade:yes"));
            of << "upgrade:no" << std::endl;

	  }
	 of.close();
	}
	if(of_c.is_open())
	{
		of_c.close();
	}
	if(of_f.is_open())
	{
		of_f.close();
	}
	return ret_flag;
}


/*********************************for priviate*****************************************************************************/

/***********************************upgrade mcu function list*************************************************************/

void dataRouterJointSet::SetMcuReportTimer(int timer )
{
    if(   timer < -1 ||  timer > 1000 ) return;

    if(upgrade_p_)
    {
        upgrade_p_->SetReportTimer(timer);
    }

    usleep(1000); 
}

void dataRouterJointSet::StartPowerAndSensorReportTImer(int bus,int channel)
{
       if(upgrade_p_)
       {
         upgrade_p_->idPowerStartTimer(bus, channel);
       }
}

void dataRouterJointSet::StopPowerAndSensorReportTImer(int bus,int channel)
{
     if(upgrade_p_)
       {
         upgrade_p_->idPowerStopTimer(bus, channel);
       }
}


VERSION_STRUCT*  dataRouterJointSet::upgradeGetVersion(int bus, int channel)
{
    if(upgrade_p_)
    {
         VERSION_STRUCT *version_struct_p = upgrade_p_->getVersion(bus,channel);
		if(true == CAN_UPGRADE_RETRY)
		{
	        if(!version_struct_p)
            {
	            usleep(1000 * 100);
	            version_struct_p = upgrade_p_->getVersion(bus,channel);
                if(!version_struct_p)
                {
                    usleep(1000 * 100);
                    version_struct_p = upgrade_p_->getVersion(bus,channel);
                }
	        }
		}
        return version_struct_p;
    }
    return NULL;
}

PRODUCT_CODE_STRUCT*  dataRouterJointSet::upgradeGetProductCode(int bus, int channel)
{
    if(upgrade_p_)
    {
          PRODUCT_CODE_STRUCT *product_struct_p = upgrade_p_->getProductCode(bus,channel);
		if(true == CAN_UPGRADE_RETRY)
		{
	        if(!product_struct_p)
            {
	            usleep(1000 * 100);
	            product_struct_p = upgrade_p_->getProductCode(bus,channel);
                if(!product_struct_p)
                {
                    usleep(1000 * 100);
                    product_struct_p = upgrade_p_->getProductCode(bus,channel);
                }
	        }
		}
        return product_struct_p;
    }
    return NULL;
}


bool dataRouterJointSet::upgradeUpgrading(int bus, int channel,int vid,int pid,std::string hardversion ,std::string binfile)
{
    if(upgrade_p_)
    {
        std::string re = upgrade_p_->upGrading(bus,channel,vid,pid,hardversion,binfile);
 	    if(true == CAN_UPGRADE_RETRY)
 	    {
 	
 		   if(!compare(re,"ok"))
            {
 			   if(channel < 0x10)
               {
 				   usleep(1000 * 300);
 			   }
 			   re = upgrade_p_->upGrading(bus,channel,vid,pid,hardversion,binfile);
               if(!compare(re,"ok"))
               {
                   if(channel < 0x10)
                   {
                      usleep(1000 * 300);
                    }
                   re = upgrade_p_->upGrading(bus,channel,vid,pid,hardversion,binfile);
                 
               }
 		   }
 	   }
       if(!compare(re,"ok"))
       {
            return false;
       }
       return true;
    }
    return false;
}



bool  dataRouterJointSet::upgradeSoftwarerecheck(std::vector<CHEKVSION> *upok_list,std::string &failed_mcu,bool isForeced)
{
    if(NULL == upok_list)
    {
        std::cout<<"up_ok list error!"<<std::endl;
        return false;
    }
    char ch[UPGRADE_MAX_PRIT_SIZE] = {0};
    if(true == SOFTWARE_RECHECK_CONTROL )
	{

		BOOST_FOREACH(CHEKVSION _up_ok,*upok_list)
		{
            StartPowerAndSensorReportTImer(_up_ok.bus, _up_ok.channel);
            VERSION_STRUCT *version_struct_p = upgradeGetVersion(_up_ok.bus, _up_ok.channel);
			if(!version_struct_p){
				snprintf(ch, sizeof(ch), "softwareversion recheck error:%s,channel:%x,bus:%d\n",_up_ok.name.c_str(),_up_ok.channel,_up_ok.bus);
				std::cout<<ch <<std::endl;
				log_error(ch);
                usleep(1000*1);
                sprintf(ch," %x",_up_ok.channel);
                failed_mcu +=ch;
				return false;
				
			}
			
			 std::string software_version;
		
			 if(upgrade_p_->idIsSteer(_up_ok.channel))
			 {
				sprintf(ch,"%02x.%02x.%02x.%02x",version_struct_p->ster.software_version_1,version_struct_p->ster.software_version_2,version_struct_p->ster.software_version_3,version_struct_p->ster.software_version_4);
			 }
			 else
			 {
		
				 sprintf(ch,"%d.%2d",version_struct_p->norm.software_version_high,version_struct_p->norm.software_version_low);
			 }
			// printf("upgrade version %d.%.2d\n",version_struct_p->software_version_high,version_struct_p->software_version_low);
			 software_version = ch;
			 
			 if(true == checkSoftWareVersion(software_version, 
			 _up_ok.sotftversion,UPGRADE_SOFT_THRESHOLD_NONE,SOFTWARE_VERSION_CONTROL)&& 
			 SOFTWARE_VERSION_CONTROL!=false)
			 {
				snprintf(ch, sizeof(ch), "softwareversion recheck error:%s,channel:%x,bus:%d,software:%s,configware:%s\n",_up_ok.name.c_str(),_up_ok.channel,_up_ok.bus,software_version.c_str(),_up_ok.sotftversion.c_str());
				printf("softwareversion recheck error:%s,channel:%x,bus:%d,software：%s,configware:%s\n",_up_ok.name.c_str(),_up_ok.channel,_up_ok.bus,software_version.c_str(),_up_ok.sotftversion.c_str());
				log_error(ch);
                usleep(1000*1);
                sprintf(ch," %x",_up_ok.channel);
                failed_mcu +=ch;
				return false;
				
			 }
		     else
		     {
                sprintf(ch,"softwareversion recheck ok:%s,channel:%x,bus:%d,software:%s,configware:%s\n",_up_ok.name.c_str(),_up_ok.channel,_up_ok.bus,software_version.c_str(),_up_ok.sotftversion.c_str());
                usleep(1000*1);
             }
			 
		}
		
	}
    return true;
}



//升级时设置标志位，升级结束后删除标志位   modified by marty.gong@ubtrobot.com:2018-1-25-14:5:59 
void dataRouterJointSet::setUpgradeStatus(int channel,bool status)
{

    if(NULL == upgrade_p_) return ;

    //判断是否是电源板   modified by marty.gong@ubtrobot.com:2018-1-29-15:11:58 
    if(false == upgrade_p_->idIsHeadPower(channel))return ;
    std::string filePath = folder_;
    int pos = filePath.find("update");
    filePath.erase(pos,6);
    
    filePath = filePath+UPGRADE_FLAG_FILE;
    
    std::ofstream fout(filePath);
    if(!fout)
    {
         log_error("error write upgrade status");
         usleep(1000*1);
         return ;
    }

    //开始进行升级   modified by marty.gong@ubtrobot.com:2018-1-25-14:35:3 
    if(true == status)
    {
        fout << "1";
    }
    //结束升级   modified by marty.gong@ubtrobot.com:2018-1-25-14:59:36 
    else
    {
        fout << "0";
    }

    fout.close();
    system("sync");

}


bool dataRouterJointSet::getBaseInfo(VERSION_STRUCT &ver_strt,PRODUCT_CODE_STRUCT &pro_strt,UPGRADE *upgrade,std::vector<std::string>*errlist,std::string norup)
{
    char ch[UPGRADE_MAX_PRIT_SIZE]={0};
    if(NULL == errlist || NULL == upgrade) return false;
    VERSION_STRUCT *version_struct_p = upgradeGetVersion(upgrade->bus,upgrade->channel);
    if(!version_struct_p){
        sprintf(ch,"get version false:%s:upgraded false,channel:%x,bus:%d\n",upgrade->name.c_str(),upgrade->channel,upgrade->bus);
        GET_INFO_ERRRO_OUT_PUT(ch,upgrade,log_,*errlist,norup);
        return false;
    }
    ver_strt = *version_struct_p;
    

    PRODUCT_CODE_STRUCT *product_struct_p = upgradeGetProductCode(upgrade->bus,upgrade->channel);
    if(!product_struct_p){
        sprintf(ch,"get product false:%s:upgraded false,channel:%x,bus:%d\n",upgrade->name.c_str(),upgrade->channel,upgrade->bus);
        GET_INFO_ERRRO_OUT_PUT(ch,upgrade,log_,*errlist,norup);
        return false;
    }
    pro_strt = *product_struct_p;
    return true;
}

bool dataRouterJointSet::versionProcess(std::string &hard_ver, std::string &soft_ver,VERSION_STRUCT* ver,int channel)
{
     char ch[UPGRADE_MAX_PRIT_SIZE];
     /*hardware*/

     if(NULL == ver) return false;
     if(true == upgrade_p_->idIsSteer(channel))
     {
             sprintf(ch,"%02x",ver->ster.hardware_version);
             hard_ver = ch;

             sprintf(ch,"%02x.%02x.%02x.%02x",ver->ster.software_version_1,ver->ster.software_version_2,ver->ster.software_version_3,ver->ster.software_version_4);
             soft_ver = ch;
              
     }
     else
     {

         sprintf(ch,"%d.%.2d",ver->norm.hardware_version_high,ver->norm.hardware_version_low);
         hard_ver = ch;
         /*software*/
         sprintf(ch,"%d.%.2d",ver->norm.software_version_high,ver->norm.software_version_low);
         soft_ver = ch;
     }
     return true;
                         
}

bool dataRouterJointSet::upgradeCheckSoftVersion(PRODUCT_CODE_STRUCT * Pproduct,PRODUCT *PproCfg,std::string software,bool isForce)
{
    if(NULL == Pproduct || NULL == PproCfg)return false;
    if(('B' == Pproduct->mode || 'b' == Pproduct->mode) || (true == isForce || true == PproCfg->forceUp))return true;

    return 
    checkSoftWareVersion(software,PproCfg->software_version,PproCfg->software_upgrade_threshold,SOFTWARE_VERSION_CONTROL);
}



void dataRouterJointSet::outPutInfoAftrupgrade(time_t* start_time,std::vector<CHEKVSION>* oklist,std::vector<std::string>*errlist,std::string norlist)
{
        char ch[UPGRADE_MAX_PRIT_SIZE]={0};
        time_t end_time = time(NULL);
        

        if(NULL == start_time || NULL == oklist || NULL == errlist)return ;

        log_info("upgrade all end\n");
        usleep(1000*1);
        log_info("----------------------------------\n");
        usleep(1000*1);
        log_info("upgrade list\n");

        std::cout<<"start:"<<*start_time<<std::endl;
        std::cout<<"end:"<<end_time<<std::endl;
        snprintf(ch, sizeof(ch), "total upgrade cost:(s) %ld\n",(end_time-*start_time));
        std::cout <<ch<<std::endl;
        
        log_info(ch);
        usleep(1000*1);
        BOOST_FOREACH(CHEKVSION _up_ok,*oklist)
        {
            snprintf(ch, sizeof(ch), "upgrade ok:%s,channel:%x,bus:%d,cost_time:%d\n",_up_ok.name.c_str(),_up_ok.channel,_up_ok.bus,(int)_up_ok.time);
            log_info(ch);
            std::cout <<ch<<std::endl;
            usleep(1000*1);
        }
    
         log_info("\n-----------------------------\n");
         usleep(1000*1);
    
        BOOST_FOREACH(std::string error,*errlist)
       {
           log_error(error.c_str());
           
           usleep(1000*1);
    
        }
         log_info("\n-----------------------------\n");
         usleep(1000*1);
        if( norlist.size() > 0)
        {
             snprintf(ch, sizeof(ch), "nor upgrade: %s\n",norlist.c_str());
             log_warn( ch);
             usleep(1000*1);
             std::cout <<ch<<std::endl;
        }
        usleep(1000*1);
        std::cout<<"upgraded finished"<<std::endl;
        


}


/************************************configServeSrv function list********************************************************/
std::string dataRouterJointSet::command_Completion(std::string command)
{
    std::string tem_command;
    std::string re_str ="\n";
    std::string old_command;
    std::string explain;
    int i =0;
    std::vector<std::string> mapping;

     BOOST_FOREACH(servoCmd com,command_list_){


        if(command == com.command_){
            tem_command = com.command_;
             explain = com.explain_;
             i=1;
            break;
        }

         if(compare(com.command_,command)){
            int start = com.command_.find_first_of(":",command.size());
            if(start < 0)
                start = com.command_.size();
             tem_command = com.command_.substr(0,start+1);
             if(tem_command == old_command)
                 continue;
             old_command = tem_command;
             re_str += "   " + tem_command;
             mapping.push_back(tem_command);
             re_str +="\n";
             i++;
         }
     }

     if(i == 1){
         re_str = config_command_ =  tem_command;

         if(explain.size() > 0)
             re_str = "\n" + explain + "\n" + config_command_;
        return  re_str;
     }
     if(i == 0){
         config_command_ = command;
         return  config_command_;
     }


     i =0;

     while(1){
         char c = tem_command.at(i);
         if(c >= 'a' && c <='z')
             c = c - 32;
          char temc = {0};
          BOOST_FOREACH(std::string com,mapping){
              temc = com.at(i);
              if(temc >= 'a' && temc <='z')
                  temc = temc - 32;
              if(c != temc)
                  break;
          }
          if(c != temc)
              break;
          i++;

     }

         re_str += tem_command.substr(0,i);
        config_command_ = tem_command.substr(0,i);

      return re_str;
}

/*
fun:bool checkHardWareVersion(std::string orgversion,std::string upversion)
para: orgversion: 原软件硬件版本 upversion:要升级的硬件版本
说明: 原软件硬件版本要在升级配置文件的硬件版本中
返回: true:升级配置文件包含原软件硬件 false:不包含或者其它错误

'1.01'是否匹配"1.02,1.01,1.03"
*/


bool dataRouterJointSet::checkHardWareVersion(std::string orgversion,std::string upversion)
{
    std::string strtemp = upversion;
    std::string strcmp ;
    int nIndex = 0;
    while(1)
    {
        nIndex = strtemp.find(',');
        if(true == CAN_UP_DEBUG)
        {
            std::cout <<"split index:"<<nIndex <<std::endl;
        }
        if(nIndex >= 0)//找到
        {

             strcmp = strtemp.substr(0,nIndex);//找到每一个分割域
             if(true == CAN_UP_DEBUG)
             {
                 std::cout <<"configversion:"<<strcmp <<" orgversion:" << orgversion<<std::endl;
             }

             if(orgversion == strcmp) return true;//比较
             else
             {
                strtemp = strtemp.substr(nIndex+1,strtemp.length()-nIndex-1);//',后面的字符串'
             }

        }
        else
        {

            if(true == CAN_UP_DEBUG)
            {
                std::cout <<"configversion:"<<strtemp <<" orgversion:" << orgversion<<std::endl;
            }

           if(strtemp.length() >= 0 && strtemp == orgversion)//配置文件版本就一个直接比较
           {

               if(true == CAN_UP_DEBUG)
               {
                   std::cout << "hardwareversion does match!" <<std::endl;

               }

                return true;
           }
           return false;
        }

    }

}

/*
fun:bool checkSoftWareVersion(std::string orgversion,std::string upversion,bool flag)
para: orgversion: 原软件软件版本 upversion:要升级的软件版本
说明: 原软件版本不能高于升级的软件版本，软件版本控制是否生效，通过flag控制
返回: true:可以升级 false:属于降级
*/


bool dataRouterJointSet::checkSoftWareVersion(std::string orgversion,std::string upversion,std::string softthreshold,bool flag)
{
	if(true != flag) return true;
	std::string new_orgversion;
	std::string new_upversion;
	std::string new_thres;

	//新版舵机需要比较后8位，一共11位。所以必须去除前3位，取子串的时候一定要考虑兼容。   	// modified by marty.gong@ubtrobot.com:2017-7-11-10:57:52 
	
	if(upversion.length()> UPGRAXE_GET_SUBVER_LEN)
	{
		new_upversion = upversion.substr(3,upversion.length()-1);

	}
	else
	{
		new_upversion = upversion;
	}

	if(softthreshold.length()>UPGRAXE_GET_SUBVER_LEN)
	{

		new_thres = softthreshold.substr(3,softthreshold.length()-1);
	}
	else
	{
		new_thres = softthreshold;
	}

	if(orgversion.length()>UPGRAXE_GET_SUBVER_LEN)
	{
		new_orgversion = orgversion.substr(3,orgversion.length()-1);

	}
	else
	{
		new_orgversion = orgversion;
	}
	

    if(true == CAN_UP_DEBUG)
    {
        std::cout<<"upversion:" <<upversion   <<" new_upversion " << new_upversion<< " orgversion:" <<orgversion << " new_orgversion: "<<new_orgversion<<" threshold: "<<softthreshold << "new_threshold: "<<new_thres<<std::endl;
        std::cout<< atof(upversion.c_str()) <<std::endl;

	
    }
	
	if(new_orgversion < new_thres) return false;

    return new_upversion != new_orgversion ? true:false;
}



/****************************************************run command function list**********************************************/

bool dataRouterJointSet::compare(std::string command,std::string cmd_name)
{
    if(command.size() < cmd_name.size())
        return false;
    std::string tem = command.substr(0,cmd_name.size());
    std::transform(tem.begin(),tem.end(),tem.begin(),::toupper);
    std::transform(cmd_name.begin(),cmd_name.end(),cmd_name.begin(),::toupper);
   if(tem == cmd_name)
        return true;

    return false;

}

std::string dataRouterJointSet::runCommand(std::string command)
{

        int para = command.find_first_of(':',0);
        std::string parameter = command.substr(para+1,command.size());

        if(compare(command , "update servo")){

             int bus,channel,hardware_version;
             sscanf(parameter.c_str(),"%d,%x,%x",&bus,&channel,&hardware_version);
             //ROS_INFO("updateServo(bus,channel,hardware_version);\n");
             std::cout << "updateServo(bus,channel,hardware_version);\n" << std::endl;
//             updateServo(bus,channel,hardware_version);
        }
        if(compare(command , "folder test")){
              folderTest();
        }
        if(compare(command , "mcu id show"))
        {
            mcuChannelIdShow();
        }
        if(compare(command , "get key:")){

            int bus,channel;
           sscanf(parameter.c_str(),"%d,%x",&bus,&channel);
           LOCK_KEY_STRUCT *key = upgrade_p_->getKey(bus,channel);
           std::string re_str = "not read key\n";
           char ch[UPGRADE_MAX_PRIT_SIZE];
           if(key){
           sprintf(ch,"lock key:%d,time:%d\n",key->key,key->key_time);
           re_str = ch;
           }
           return re_str;
        }

        if(compare(command , "get product:")){

            int bus,channel;
           sscanf(parameter.c_str(),"%d,%x",&bus,&channel);
           PRODUCT_CODE_STRUCT *product = upgrade_p_->getProductCode(bus,channel);

           std::string re_str = "not read product\n";
           char ch[UPGRADE_MAX_PRIT_SIZE];
           if(product){
           sprintf(ch,"product vid:%.4x,pid:%.4x,bus version:%d.%.2d,mode:%c\n",product->vid,product->pid,
                   product->can_bus_protocol_high,product->can_bus_protocol_low,product->mode);
           re_str = ch;
           }
           return re_str;
        }

        if(compare(command , "upgradeALLservo:")){
            bool ver_get_flag = true;
            std::string failed_mcu="";
            upgradeMcu(false,true,0,ver_get_flag,failed_mcu);
            std::cout<<"faild mcu is "<<failed_mcu<<endl;
            return "upgradeALLservo";
         }
        if(compare(command , "upgradeOneservo:")){
            int channel;
            bool ver_get_flag = true;
            std::string failed_mcu="";
           sscanf(parameter.c_str(),"%x",&channel);
            upgradeMcu(false,false,channel,ver_get_flag,failed_mcu);
            return "upgradeOneservo";

         }

        if(compare(command , "upPresstest:")){
            int channel_ =0;
            int time_ = 0;
            int i = 0;
            bool ver_get_flag = true;
           sscanf(parameter.c_str(),"%x,%d",&channel_,&time_);
           std::string failed_mcu="";
            for( ; i<time_;)
            {
                std::cout<<"start the "<< ++i<<" upgrading..."<<std::endl;
                failed_mcu = "";
                if("ok" != upgradeMcu(true,true,channel_,ver_get_flag,failed_mcu))
                {
                    std::cout<<"the "<<i<< " upgrading faild."<<std::endl;
                    break;
                }
                else
                {
                    std::cout<<"the "<<i<<" upgrading successfull."<<std::endl;
                }
                
            }
            return "upgradeOneservo";

         }

    if(compare(command,"upgrade force set"))
    {   
        SOFTWARE_RECHECK_CONTROL = false;
        SOFTWARE_VERSION_CONTROL = false;
    }

    if(compare(command,"upgrade force clear"))
    {
        SOFTWARE_RECHECK_CONTROL = true;
        SOFTWARE_VERSION_CONTROL = true;
    }
    if(compare(command , "get version:")){

         int bus,channel;
		 int get =0,not_get=0,i = 0,index = 0;
	     std::string re_str = "not read version\n";
         int timer_ = 0;
        sscanf(parameter.c_str(),"%d,%x,%d",&bus,&channel,&timer_);

        
		for(i = 0; i < timer_; i++)
		{

            std::cout<<"index:"<<index++<<std::endl;
			VERSION_STRUCT *version = upgrade_p_->getVersion(bus,channel);

            char ch[UPGRADE_MAX_PRIT_SIZE];
            if(version){

				if(true == upgrade_p_->idIsSteer(channel))
				{
					sprintf(ch,"version hardvare:%02x,software:%02x.%02x.%02x.%02x\n",version->ster.hardware_version,version->ster.software_version_1,version->ster.software_version_2,
					version->ster.software_version_3,version->ster.software_version_4);
				}
				else
				{

					sprintf(ch,"version hardvare:%d.%.2d,software:%d.%.2d,instruction:%d.%.2d\n",version->norm.hardware_version_high,version->norm.hardware_version_low, version->norm.software_version_high,version->norm.software_version_low,version->norm.instruction_version_high,version->norm.instruction_version_low);
				}
				re_str = ch;
				get++;
				std::cout<<re_str<<std::endl;
            }
			else
			{
				not_get++;
				std::cout<<"get version false"<<std::endl;
			}
		}
        std::cout<<"get :"<<get<<"  not get "<<not_get<<std::endl;
        return re_str;
     }

	if(compare(command , "set reportTimer"))
	{
		int timer = 0;

		sscanf(parameter.c_str(),"%d",&timer);


		std::cout<<"timer is :"<<timer<<std::endl;
		upgrade_p_->SetReportTimer(timer);

		return "";
		
    }

     if(compare(command , "bms upgrade:"))
     {
        char ch[UPGRADE_MAX_PRIT_SIZE];
        BMSupgrade bmsUpgrade_;
        int timer_ = 0;
        sscanf(parameter.c_str(),"%d",&timer_);
        bmsUpgrade_.init();

        for(int i = 0; i< timer_; i++)
        {
            std::cout <<"begin the "<<i <<" upgrade starting."<<std::endl;
            
            if(false == bmsUpgrade_.upgrade())
            {
                sprintf(ch,"bms upgrade failed.");
                break;
            }
            else
            {
                sprintf(ch,"bms upgrade sucessfully.");
                std::cout << "the "<< i << "upgrading has succeed."<<std::endl;
                sleep(1);
            }
        }

        std::cout<<ch<<std::endl;
        return ch;
        
     }
      if(compare(command , "get bms version"))
     {
        char ch[UPGRADE_MAX_PRIT_SIZE];
        BMSupgrade bmsUpgrade_;

        bmsUpgrade_.init();
        string str_;

        if(false == bmsUpgrade_.getVersion(str_))
        {
            sprintf(ch,"bms version get failed.");   
        }
        else
        {
            sprintf(ch,"%s",str_.c_str());
        }
        std::cout<<ch<<std::endl;
        return ch;
        
     }

     if(compare(command , "gs upgrade:"))
     {
        int time_;
        GSupgrade gsUpgrade_;
        Config *cfg = Config::get_instance();
        if(NULL == cfg)
        {
            std::cout<<"config init error."<<std::endl;
            return "init error.";
        }
        gsUpgrade_.init();

        sscanf(parameter.c_str(),"%d",&time_);
        for(int i = 0; i< time_;)
        {
              std::cout<<"start the "<< ++i<<" upgrading..."<<std::endl;
              if(false == gsUpgrade_.upgrade())
                {
                      std::cout<<"the "<<i<< " upgrading faild."<<std::endl;
                      break;
                }
                else
                {
                       std::cout<<"the "<<i<< " upgrading sucessfully."<<std::endl;
                }
         }
        std::cout <<"gs upgrading end."<<std::endl;
        return "gs upgrading end.";
        
     }
    
     if(compare(command,"upgrade all:"))
     {
        int time_;
        bool ver_get_flag_=false;
        string failed_mcu = "";
        GSupgrade gsUpgrade_;
        Config *cfg = Config::get_instance();
        if(NULL == cfg)
        {
            std::cout<<"config init error."<<std::endl;
            return "init error.";
        }
        gsUpgrade_.init();

        BMSupgrade bmsUpgrade_;

        bmsUpgrade_.init();

        sscanf(parameter.c_str(),"%d",&time_);


        std::cout <<"upgrade total time : "<<time_<<std::endl;
        for(int i = 0; i< time_;i++)
        {
              std::cout<<"start the "<< i+1<<" upgrading..."<<std::endl;
                
                if(false == gsUpgrade_.upgrade())
                {
                      std::cout<<"the "<<i+1<< "gs  upgrading faild."<<std::endl;
                      break;
                }
                else
                {
                    if("ok" != upgradeMcu(false,true,0,ver_get_flag_,failed_mcu))
                    {
                        std::cout<<"the "<<i+1<< " upgrading faild."<<std::endl;
                        break;
                    }
                    else
                    {
                        std::cout<<"the "<<i+1<< " upgrading all firmware sucessfully."<<std::endl;
                    }
                
                }
         }
        return "upgrade all";
        
     }
     if(compare(command , "get gs version"))
     {
        char ch[UPGRADE_MAX_PRIT_SIZE];
        GSupgrade gsUpgrade_;
        Config *cfg = Config::get_instance();
        if(NULL == cfg)
        {
            std::cout<<"config init error."<<std::endl;
            return "init error.";
        }
        gsUpgrade_.init();
        string str_;
        
        if(false == gsUpgrade_.getVersion(str_))
        {
            sprintf(ch,"gs version get failed.");   
        }
        else
        {
            sprintf(ch,"%s",str_.c_str());
        }
        std::cout<<ch<<std::endl;
        return ch;
        
     }

     
    int start = command.find_first_of(":");
    if(start < 0){
       // ROS_ERROR("command:%s error", command.c_str());
        return "";
    }


    return "";
}



void dataRouterJointSet::updateProgressBar(std::string str)
{
        char ch[UPGRADE_MAX_PRIT_SIZE];
        snprintf(ch, sizeof(ch), "%s:%s\n",upgrading_product_.c_str(),str.c_str());
        std::cout << ch << std::endl;
        std::string re = upgrading_product_ + ":" + str;
        log_info(re.c_str());
}

void dataRouterJointSet::setUpgradeFolder(std::string folder)
{
    folder_ = folder;

}


void dataRouterJointSet::mcuChannelIdShow(void)
{
    
   std::vector<std::string> path_list;
   char ch[128] = {} ;
   listDir(folder_,path_list);
   if( compare(loadYaml(),"error"))
   {
		std::cout << "read file error!"<<std::endl;
		return ;
   }
   BOOST_FOREACH(UPGRADE upgrade,upgrade_list_)
   {
        sprintf(ch,"name: %s   channel:0x%x,bus:%d\n",upgrade.name.c_str(),upgrade.channel,upgrade.bus);
        printf("%s \n",ch);
   }
    
}

void dataRouterJointSet::initCommandList(void)
{


    command_list_.push_back( servoCmd("get key:","input can bus,id,zero angle(0~360):d,xx,f"));
    command_list_.push_back( servoCmd("get name:","input can bus,id,angle:d,xx,dd"));
    command_list_.push_back( servoCmd("get product:","input can bus,id,serial code :d,xx,xx,xx,xx,xx"));
    command_list_.push_back( servoCmd("get version:","input can bus,id,time:d,xx,d"));
    command_list_.push_back( servoCmd("update servo:","input can bus,id:d,xx"));
    command_list_.push_back( servoCmd("enter upgrade:","input can bus,id:d,xx"));
    command_list_.push_back( servoCmd("upgradeALLservo:","input can bus,id:d,xx"));
    command_list_.push_back( servoCmd("upgradeOneservo:","input id:d,xx"));
    command_list_.push_back(servoCmd("upPresstest:","input id,time:xx,d"));
    command_list_.push_back( servoCmd("upgrade force set","clear software version control"));
    command_list_.push_back( servoCmd("upgrade force clear","set software version control"));
    command_list_.push_back( servoCmd("folder test",""));
    command_list_.push_back( servoCmd("mcu id show","show the channel id and bus id "));
	command_list_.push_back(servoCmd("set reportTimer(ms):","set mcu report timer,timer:d"));
    command_list_.push_back(servoCmd("bms upgrade:","upgrade bms,time:d"));
    command_list_.push_back(servoCmd("get bms version","bms version get"));
    command_list_.push_back(servoCmd("gs upgrade:","upgrade gaussian,time:d"));
    command_list_.push_back(servoCmd("get gs version","gaussian version get"));
    command_list_.push_back(servoCmd("upgrade all:","upgrade time:d"));

}

static  std::string read_yaml_to_product(const YAML::Node &doc,PRODUCT *product)
{
	if( NULL == product) return "error: access NULL file.";
    if(doc["product"])
	{
        (doc)["product"] >> product->product;

		if((doc)["vid"] )
		{
            (doc)["vid"] >> product->vid;

		}
		else
		{
			return "error: can not read vid";
		}
		if((doc)["pid"])
		{
            (doc)["pid"] >> product->pid;
		}
		else
		{
			return "error: can not read pid";
		}
		if((doc)["software_version"])
		{
            (doc)["software_version"] >> product->software_version;
		}
		else
		{
			return "error: can not read software version";
		}
		if((doc)["hardware_version"])
		{
            (doc)["hardware_version"] >> product->hardware_version;
		}
		else
		{
			return "error: can not read hardware version";

		}
		if((doc)["software_upgrade_threshold"])
		{
            (doc)["software_upgrade_threshold"] >> product->software_upgrade_threshold;
		}
		else
		{
			return "error: can not read software threshold";
		}
		if((doc)["bin_file"])
		{
            (doc)["bin_file"] >> product->bin_file;
		}
		else
		{
			return "error: can not read bin_file";
		}
        if((doc)["forceUp"])
		{
           std::string isforce_;
           (doc)["forceUp"] >> isforce_;
           if("true" == isforce_||"yes" == isforce_)
           {
                product->forceUp = true;
           }
           else
           {
                product->forceUp = false;
           }
		}
		else
		{
			product->forceUp = false;
		}
		return "ok";
			 
    }
	return "not find";
	
}

static std::string read_yaml_to_config(const YAML::Node &doc,std::string *config)
{
	if(NULL == config) return "error: access NULL file.";

	if((doc)["confirg"])
	{
		if((doc)["release"])
		{
			(doc)["release"] >> *config;
		}
		else
		{
			return "error: can not read release";
		}
		return "ok";
	}
	else
	{
		return "not find";
	}
}

static std::string read_yaml_to_upgrade(const YAML::Node &doc,UPGRADE *upgrade)
{

	if(NULL == upgrade) return "error: access NULL file.";
	if((doc)["name"])
	{

		(doc)["name"] >> upgrade->name;

		if((doc)["bus"])
		{
			(doc)["bus"] >> upgrade->bus;

		}
		else
		{
			return "error: can not read bus";
		}
		if((doc)["channel"] )
		{
			(doc)["channel"] >> upgrade->channel;
		}
		else
		{
			return "error: can not read channel";
		}
		if((doc)["driver"])
		{
	        (doc)["driver"] >> upgrade->driver;
		}
		else
		{
			return "error: can not read driver";
		}
		return "ok";
	}
	return "not find";
}



std::string dataRouterJointSet::parse_servo_yaml(const YAML::Node &doc)
{ 
//	int count = (doc.size() > SERVO_NUM)?SERVO_NUM:(int)doc.size();
	UPGRADE upgrade;
//	for(int i = 0; i < count; i++)
	for(std::size_t i = 0; i < doc.size(); i++)
	{
		doc[i]["name"] >> upgrade.name;
		doc[i]["channel"] >> upgrade.channel;
		doc[i]["can_birdge"] >> upgrade.can_birdge;
		doc[i]["bus"] >> upgrade.bus;
		doc[i]["driver"] >> upgrade.driver;
		
		upgrade_list_.push_back(upgrade);
	}
	return "ok";

//	return "not find";
}

void dataRouterJointSet::listDir(std::string path,std::vector<std::string> &path_list)
{
    DIR* dir = opendir(path.c_str());//打开指定目录

    dirent* p = NULL;//定义遍历指针

    if(NULL == dir)return;//解决load空文件时程序挂起的问题；
    while((p = readdir(dir)) != NULL)//开始逐个遍历
    {

        if(p->d_type & DT_DIR)
        {

                if(strcmp(p->d_name,".")==0 || strcmp(p->d_name,"..")==0)
                        continue;
                std::string name =  std::string(p->d_name);
                name = path +"/" + name;
                std::cout<<"--------------folder:"<<name<<std::endl;
                listDir(name,path_list);

        }
        else
        {
            std::string name =  std::string(p->d_name);
            int size = strlen(p->d_name);


            if(size<5)
                continue;
            if(strcmp( ( p->d_name + (size - 5) ) , ".yaml") != 0)
                continue;

            std::cout<< path +"/" +name<<std::endl;
            path_list.push_back(path +"/" +name);
        }

    }
    closedir(dir);//关闭指定目录


}

std::string dataRouterJointSet::loadYaml(void)
{
	std::vector<std::string> path_list;
	product_list_.clear();
	upgrade_list_.clear();

	listDir(folder_,path_list);
	
	BOOST_FOREACH (std::string path, path_list) 
   {
        std::cout<<path << std::endl;
        std::ifstream fin(path.c_str());
        if(!fin)continue;
		YAML::Node doc;
        doc = YAML::Load(fin);
        for(unsigned int  i=0;i<doc.size();i++)
         {
            std::string release_str; 
			PRODUCT product;
			UPGRADE upgrade;
			std::string check_result;
            
			check_result = read_yaml_to_config(doc[i],&release_str);
			if(true ==	compare(check_result,"error"))
			{
				std::cout << check_result.c_str() << std::endl;
                  fin.close();
				 return check_result;
			}
			if(true == compare(check_result,"ok"))
			{
				if("no" == release_str)//配置禁止升级，返回
				{
					upgrade_list_.clear();
					product_list_.clear();
                    fin.close();
					return "ok";
				}
			}

			check_result = read_yaml_to_product(doc[i],&product);//搜索product文件
			if(true == compare(check_result,"error"))
			{
				std::cout << check_result.c_str() <<std::endl;
                fin.close();
				return check_result;
			}
			if(true == compare(check_result,"ok"))
			{
                std::cout<<"product read ok."<<std::endl;
                product_list_.push_back(product);
				continue;
			}

			check_result = read_yaml_to_upgrade(doc[i],&upgrade);//搜索upgrade文件

			if(true == compare(check_result,"error"))
			{
				std::cout << check_result.c_str() <<std::endl;
                fin.close();
				return check_result;
			}
			if(true == compare(check_result,"ok"))
			{
				upgrade_list_.push_back(upgrade);
			}

		}
        fin.close();
	}
    isloadcfg = true;
	return "ok";
}




void dataRouterJointSet::folderTest(void)
{


   std::vector<std::string> path_list;
   listDir(folder_,path_list);
    //ROS_INFO("---------------------------\n");
     std::cout << "---------------------------\n" << std::endl;
   BOOST_FOREACH (std::string path, path_list) {
       // ROS_INFO("yaml path:%s\n",path.c_str());
         std::cout << "yaml path:";
         std::cout << path.c_str() << std::endl;
  }
   if( compare(loadYaml(),"error"))
   {
		std::cout << "read file error!"<<std::endl;
		return ;
   }
   std::cout << "upgrade list is :"<<std::endl; 

  BOOST_FOREACH(UPGRADE upgrade,upgrade_list_)
  {
	 std::cout<<"name :"<<upgrade.name <<std::endl;
	 std::cout<< "		channel: "<<upgrade.channel<<" can_birdge: "<<upgrade.can_birdge <<std::endl;
	 std::cout<<"		bus: "<<upgrade.bus<<" driver: "<<upgrade.driver <<std::endl;
	 std::cout<<"		upgraded "<<upgrade.upgraded << std::endl; 
	 std::cout<<std::endl;
  }

	std::cout << "product list is :" <<std::endl;
  BOOST_FOREACH(PRODUCT product,product_list_)
  {
		std::cout<<"name: "<<product.product<<std::endl;
		std::cout<< "		pid: "<<product.pid<<" vid:	" << product.vid <<std::endl;
		std::cout<<"		hardware version "<<product.hardware_version<<std::endl;
		std::cout<<"		software version "<< product.software_version <<std::endl;
		std::cout<<" 		software version_threshold "<<product.software_upgrade_threshold<<std::endl;
		std::cout<<" 		bin file "<<product.bin_file <<std::endl;
		std::cout<<"		product " << product.product <<std::endl;
		std::cout<<"		upgraded " << product.upgraded<<std::endl; 
		std::cout<<std::endl;
  }
   

}



/******************************************************data version function list******************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/types.h>
using namespace std;




//false:不升级，true 说明有版本不匹配，要升级。   modified by marty.gong@ubtrobot.com:2017-7-05-17:46:45 
bool dataRouterJointSet::productVersionCompare(int bus,int channel,char*name,char* hardware,char* software,char* 
threshold,std::ofstream* of_c,std::ofstream* of_f)
{

	VERSION_STRUCT *p = NULL;
	PRODUCT_CODE_STRUCT *pro = NULL;
	char ch[UPGRADE_MAX_PRIT_SIZE] = {0};
	std::string hardware_version;
	std::string software_version;
	bool is_getVersion = true;
	char mode = 0;
	bool ret = false;
	char  version_is_same ='Y';
	if(NULL == hardware ||NULL== software ||NULL == threshold || NULL == name||NULL == upgrade_p_|| NULL == of_c || 
	NULL == of_f) return false;


	//调用can获取版本信息
	if(NULL == (p = (VERSION_STRUCT*)upgrade_p_->getVersion( bus, channel)))
	{
	
		if(NULL == (p = (VERSION_STRUCT*)upgrade_p_->getVersion( bus, channel)))
		{
			std::cout<<"get version false"<<std::endl;
			is_getVersion = false;
			
		}
	}
	//注意，获取到版本后必须立刻处理。要不然数据就会失效。   modified by marty.gong@ubtrobot.com:2017-7-20-10:23:58 
	
	 if(NULL != p)
    {
		if(true == upgrade_p_->idIsSteer(channel))
		 {
				 sprintf(ch,"%02x",p->ster.hardware_version);
				 hardware_version = ch;
				 sprintf(ch,"%02x.%02x.%02x.%02x",p->ster.software_version_1,p->ster.software_version_2,
				  p->ster.software_version_3,p->ster.software_version_4);
				 software_version = ch;
				  
		 }
		 else
		 {
			 sprintf(ch,"%d.%.2d",p->norm.hardware_version_high,p->norm.hardware_version_low);
			 hardware_version = ch;
			 /*software*/
			 sprintf(ch,"%d.%.2d",p->norm.software_version_high,p->norm.software_version_low);
			 software_version = ch;
		
		 }
	}
	else
	{
		hardware_version = "N/A";
		software_version = "N/A";
	}
	
	if(NULL == (pro = (PRODUCT_CODE_STRUCT*)upgrade_p_->getProductCode( bus, channel)))
	{
	
		if(NULL == (pro = (PRODUCT_CODE_STRUCT*)upgrade_p_->getProductCode( bus, channel)))
		{
			std::cout<<"get product false"<<std::endl;
			is_getVersion = false;
			
		}
	}
	


	if(NULL != pro)
	{
		mode = pro->mode;

		//转换成大写   modified by marty.gong@ubtrobot.com:2017-7-19-20:42:7 
		if(mode > 'Z')
		{
			mode = mode -  ('a' - 'A');
		}
		
	}
	else
	{
		mode = '-';
	}

	 if(false == is_getVersion)
	 {
		ret = false;
		version_is_same ='-';
		
	 
	 }
	 else
	 {
		if(false == checkHardWareVersion(hardware_version,hardware))
		{
			ret = false;
			version_is_same = 'N';

		}
		else
		{
			if(false  == checkSoftWareVersion(software_version,software,threshold, true))
			{
				ret = false;
				version_is_same = 'N';
			}
			else
			{
				ret = true;
				version_is_same = 'Y';
			}
		}
	 }
	 //output file
	 char tmp[UPGRADE_MAX_PRIT_SIZE] = {0};
	 char version_tmp[UPGRADE_MAX_PRIT_SIZE] = {0};
	 
	 sprintf(tmp,"%s:%s-%s,%s-%s,%s",name,hardware_version.c_str(),hardware,software_version.c_str(),software,threshold);
	 
	 sprintf(version_tmp,"%s:%s,%s(%s),%c,%c",name,hardware_version.c_str(),software_version.c_str(),software,mode,version_is_same);
	 std::cout<<tmp<<endl;
	 if(of_c->is_open())
	 {
		 *of_c<<tmp<<std::endl;
	 }
	 
	 if(of_f->is_open())
	 {
		 *of_f<<version_tmp<<std::endl;
	 }



	 return ret;
	 
	 

	 
	


}


//false：不升级，true  有版本不匹配。需要升级 modified by marty.gong@ubtrobot.com:2017-7-05-17:47:31 
bool dataRouterJointSet::create_multi_dir(const char *path)
{
    int i, len;
    char dir_path[256];

    len = strlen(path);
    dir_path[len] = '\0';

    strncpy(dir_path, path, len);

    for (i=0; i<len; i++)
    {
        if (dir_path[i] == '/' && i > 0)
        {
            dir_path[i]='\0';
            if (access(dir_path, F_OK) < 0)
            {
                if (mkdir(dir_path, 0777) < 0)
                {
                    printf("mkdir=%s:msg=%s\n", dir_path, strerror(errno));
                    return -1;
                }
            }
            dir_path[i]='/';
        }
    }

    return true;
}

/**************************************************************************
 * 函数名: chassisVersionCompare
 * 类型: 普通函数
 * 功能: 电机版本比较函数
 * 
 说明：版本匹配规则：1,配置硬件与机器硬件相同或者机器硬件是0.07或者获取失败那么选第一个找到的。否则找下一个
 * 参数:
 *    @[in ] 
 *    @[out]
 * 返回值: 
 * 时间: 2017/9/25:11:39:35
 * 作者: marty.gong
 **************************************************************************/

bool dataRouterJointSet::chassisVersionCompare(int bus,int channel,char*name,char*driver,char* hardware,char* software,char* 
threshold,std::ofstream* of_c,std::ofstream* of_f)
{
    VERSION_STRUCT *p = NULL;
    PRODUCT_CODE_STRUCT *pro = NULL;
    char ch[UPGRADE_MAX_PRIT_SIZE] = {0};
	std::string hardware_version;
	std::string software_version;
	bool is_getVersion = true;
	char mode = 0;
	bool ret = false;
	char  version_is_same ='Y';
    //param check   modified by marty.gong@ubtrobot.com:2017-9-25-11:39:12 
    if(NULL == hardware ||NULL== software ||NULL == threshold || NULL == name||NULL == upgrade_p_|| NULL == of_c || 
	NULL == of_f) return false;

    //get version   modified by marty.gong@ubtrobot.com:2017-9-25-14:26:48 
    //调用can获取版本信息
	if(NULL == (p = (VERSION_STRUCT*)upgrade_p_->getVersion( bus, channel)))
	{
	
		if(NULL == (p = (VERSION_STRUCT*)upgrade_p_->getVersion( bus, channel)))
		{
			std::cout<<"get version false"<<std::endl;
			is_getVersion = false;
			
		}
	}
    
    BOOST_FOREACH(PRODUCT product,product_list_)
    {
        if(0 == strcmp(product.product.c_str(),driver))
        {
        
            /*
             printf("get chassis config : %s,soft version: %s, hard version %s, threshold %s\n",product.product.c_str(),product.software_version.c_str(),
             product.hardware_version.c_str(),product.software_upgrade_threshold.c_str());
            */
            if(true == is_getVersion)
            {
                sprintf(ch,"%d.%.2d",p->norm.hardware_version_high,p->norm.hardware_version_low);
                hardware_version = ch;
             //   printf("the chassis  version is %s\n",ch);
                
            }
            if(true == is_getVersion && hardware_version != product.hardware_version && "0.07" != hardware_version)//need get the next config yaml
             {
               // printf("the version is not the exact version ,go to next.\n");
                continue;
             }
            
            //read config info and write the file   modified by marty.gong@ubtrobot.com:2017-9-25-16:27:56 
             /*software*/
              strncpy(software,product.software_version.c_str(),product.software_version.length());
              strncpy(hardware,product.hardware_version.c_str(),product.hardware_version.length());
              strncpy(threshold,product.software_upgrade_threshold.c_str(),product.software_upgrade_threshold.length());
            if(true == is_getVersion)
            {

                sprintf(ch,"%d.%.2d",p->norm.software_version_high,p->norm.software_version_low);
                software_version = ch;
                
            }
            else
            {
                hardware_version = "N/A";
                software_version = "N/A";
            }
            // product id   modified by marty.gong@ubtrobot.com:2017-9-25-16:37:11 
            if(NULL == (pro = (PRODUCT_CODE_STRUCT*)upgrade_p_->getProductCode( bus, channel)))
        	{
        	
        		if(NULL == (pro = (PRODUCT_CODE_STRUCT*)upgrade_p_->getProductCode( bus, channel)))
        		{
        			std::cout<<"get product false"<<std::endl;
        			
        		}
        	 }
	
        	if(NULL != pro)
        	{
        		mode = pro->mode;

        		//转换成大写   modified by marty.gong@ubtrobot.com:2017-7-19-20:42:7 
        		if(mode > 'Z')
        		{
        			mode = mode -  ('a' - 'A');
        		}
        		
        	}
        	else
        	{
        		mode = '-';
        	}

      	 if(false == is_getVersion)
      	 {
      		ret = false;
      		version_is_same ='-';
      		
      	 
      	 }
      	 else
      	 {
      		if(false == checkHardWareVersion(hardware_version,hardware))
      		{
      			ret = false;
      			version_is_same = 'N';
  
      		}
      		else
      		{
      			if(false  == checkSoftWareVersion(software_version,software,threshold, true))
      			{
      				ret = false;
      				version_is_same = 'N';
      			}
      			else
      			{
      				ret = true;
      				version_is_same = 'Y';
      			}
      		}
      	 }
	 //output file
	 char tmp[UPGRADE_MAX_PRIT_SIZE] = {0};
	 char version_tmp[UPGRADE_MAX_PRIT_SIZE] = {0};
	 
	 sprintf(tmp,"%s:%s-%s,%s-%s,%s",name,hardware_version.c_str(),hardware,software_version.c_str(),software,threshold);
	 
	 sprintf(version_tmp,"%s:%s,%s(%s),%c,%c",name,hardware_version.c_str(),software_version.c_str(),software,mode,version_is_same);
	 std::cout<<tmp<<endl;
	 if(of_c->is_open())
	 {
		 *of_c<<tmp<<std::endl;
	 }
	 
	 if(of_f->is_open())
	 {
		 *of_f<<version_tmp<<std::endl;
	 }

	 return ret;
        }
    }
    return ret;
}


