
#include "include/upgrade.h"
#include "boost/filesystem/path.hpp"
#include <boost/filesystem.hpp>
#include "can/can_pkg.h"
#include <sys/time.h>
#include <iostream>

#define IF_PACK_FIT_IN_CONDITION(channel_, condition_)               \
        (((package.head.id.channel & 0xff) == channel_) &&                         \
        (package.data[0] == condition_))

#define WAITE_TIME_IN_GET_QUE   1

#define GET_PACK_FROM_QUE() ((canPkg.getPackageFromQueue( &package,WAITE_TIME_IN_GET_QUE)))

#define UPGRADE_TRANS_STEP_ONCE 4

#define UPGRADE_TRANS_WAITE_TIME 3

#define UPGRADE_PERCENT_BASE 100 

#define UPGRADE_RETRANS_NUM 100

#define UPGRADE_ERASE_CRC_WAITE_NUM 2

#define UPGRADE_PREPARE_RETRANS_NUM 10

#define UPGRADE_CRC_POLYNOMIAL  0x04c11db7 
#define BOTTOM_POWER_ID 0x69

//1S   modified by marty.gong@ubtrobot.com:2018-1-10-14:58:57 
#define UPGRADE_MAX_RECEVIE_TIME 1000 
#define CAN_CONTROL_MODE 0

int steerId[] = {0x1,0x2,0x3,0x4,0x5,0x7,0x8,0x9,0xa,0xb,0xd,0xe,0xf,0x10,0 };
int mcu_id[] = {0x1,0x2,0x3,0x4,0x5,0x7,0x8,0x9,0xa,0xb,0xd,0xe,0xf,0x10,0x21,0x22,0x23,0x65,0x66,0x68,0x69,0x6e,0x6f,0};
int mcu_bus[] = {0,1,-1};
int chassisId[] = {0x21,0x22,0x23,0};
//0x66,0x68,0x69,0x6e,0x6f升级前需要开启上报频率，升级后再关闭   modified by marty.gong@ubtrobot.com:2017-10-21-17:1:5 
int powerId[] = {0x66,0x68,0x69,0x6e,0x6f,0};

int HeadPowerId[]= {0x68,0};
upGrade::upGrade(
    boost::function < void(std::string) > callback):callback_(callback),canPkg("can1")
{
    canPkg.init();
}

void upGrade::swtichPktRecvier(bool on_)
{
   if(true == on_)
   {
        canPkg.packageGetRestart();
   }
   else
   {
        canPkg.packgetGetPause();
   }

}


bool upGrade::sendPacketToCanBusWithoutPacket(unsigned int bus, unsigned int channel, unsigned int mode,
     unsigned char cmd,  unsigned int size )
{
    if (0 == channel)return false;

    package.head.id.can = bus;
    package.head.id.channel = channel;
    package.head.id.mode = mode;
    package.data[0] = cmd;
    package.head.id.size = size;

    canPkg.packageToCanBus(package);
	return true;
}


bool upGrade::sendPacketToCanBusWithUpgradePacket(unsigned int bus, unsigned int channel, unsigned int mode,
     unsigned char cmd, int key, int length)
{	

	if (0 == channel) return false;


    package.head.id.can = bus;
    package.head.id.channel = channel;
    package.head.id.mode = mode;
    package.head.id.size = sizeof(UPGRADE_REQUEST_STRUCT);
    CAN_PACKAGE_CMD_STRUCT * can_package_cmd = (CAN_PACKAGE_CMD_STRUCT *) &package.data;

    can_package_cmd->cmd = cmd;
    can_package_cmd->body.upgrade_request.key = key;
    can_package_cmd->body.upgrade_request.program_length = length;


	canPkg.packageToCanBus(package);

    return true;
	
}

bool upGrade::sendPacketToCanBusWithProgramPacket(unsigned int bus, unsigned int channel, unsigned int mode,
     unsigned char cmd, int address, u32 data)
{
	if (0 == channel) return false;


	package.head.id.can = bus;
    package.head.id.channel = channel;
    package.head.id.mode = mode;
    package.head.id.size = sizeof(PROGRAMMING_REQUEST_STRUCT);
    CAN_PACKAGE_CMD_STRUCT * cmd_p = (CAN_PACKAGE_CMD_STRUCT *) &package.data;

    cmd_p->cmd = cmd;
    cmd_p->body.programming_request.program_address = address;
    cmd_p->body.programming_request.code = data;


    canPkg.packageToCanBus(package);

	return true;

}

bool upGrade::sendPacketToCanBusWithProgramEndPacket(unsigned int bus, unsigned int channel, unsigned int mode,
     unsigned char cmd, u32 crc)
{

	if (0 == channel) return false;



	package.head.id.can = bus;
    package.head.id.channel = channel;
    package.head.id.mode = mode;
    package.head.id.size = sizeof(PROGRAM_END_REQUEST_STRUCT);
    CAN_PACKAGE_CMD_STRUCT * cmd_p = (CAN_PACKAGE_CMD_STRUCT *) &package.data;

    cmd_p->cmd = cmd;
    cmd_p->body.program_end_request.send_crc = crc;

    canPkg.packageToCanBus(package);

    return true;
}

u8* upGrade::getPacktFromCanBus(int bus,int channel,int condition,int timeWaite)
{
    //默认已经清空了队列   modified by marty.gong@ubtrobot.com:2018-1-09-11:34:50 

    struct  timeval start;
    struct  timeval end;
    unsigned  long diff = 0;
    unsigned  long time_end = timeWaite*1000; //timeWaite(ms)
    gettimeofday(&start,NULL);
	while(true)
	{

		 if(true == canPkg.getPackageFromQueue( &package,WAITE_TIME_IN_GET_QUE))
		{
            
             
             if(0 && package.data[0] >= 0xbb)
    		 {
                printf("in size:%d mode:%d,channel:%.2x bus:%d==data =%.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x \n",package.head.id.size,package.head.id.mode,package.head.id.channel,package.head.id.can,
    				   package.data[0],package.data[1],package.data[2],package.data[3],package.data[4],package.data[5],package.data[6],package.data[7]);
             }
            if (IF_PACK_FIT_IN_CONDITION(channel,condition)) 
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

PRODUCT_CODE_STRUCT * upGrade::getProductCode(int bus, int channel)
{

//	char ch[UPGRADE_MAX_PRIT_SIZE]={0};
    PRODUCT_CODE_STRUCT *p=NULL;
	
	if (false ==
         sendPacketToCanBusWithoutPacket((unsigned int) bus, (unsigned int) channel, CAN_CONTROL_MODE, HTN_PRODUCT_CODE_REQUEST))
    {
        return NULL;
    }

    p = (PRODUCT_CODE_STRUCT*)getPacktFromCanBus(bus,channel,NTH_PRODUCT_CODE_RESPONSE,UPGRADE_MAX_RECEVIE_TIME);
    
    return p;

}


VERSION_STRUCT * upGrade::getVersion(int bus, int channel)
{

    VERSION_STRUCT * p = NULL;

	if(true == idIsSteer(channel))
	{
		
		if (false ==
			 sendPacketToCanBusWithoutPacket((unsigned int) bus, (unsigned int) channel, CAN_CONTROL_MODE, HIN_VERSION_REQUEST_STEER)) 
		{
			return NULL;
		}

        p = (VERSION_STRUCT *)getPacktFromCanBus(bus, channel, NTH_VERSION_RESPONSE_STEER, UPGRADE_MAX_RECEVIE_TIME);
         return p;
		
	}
	else
	{
		if (false ==
			 sendPacketToCanBusWithoutPacket((unsigned int) bus, (unsigned int) channel, CAN_CONTROL_MODE, HTN_VERSION_REQUEST)) 
		{
			return NULL;
		}
		
		p = (VERSION_STRUCT *)getPacktFromCanBus(bus, channel, NTH_VERSION_RESPONSE, UPGRADE_MAX_RECEVIE_TIME);
         return p;
		
		
	}
}


LOCK_KEY_STRUCT * upGrade::getKey(int bus, int channel)
{


	if(false == sendPacketToCanBusWithoutPacket((unsigned int) bus, (unsigned int) channel, CAN_CONTROL_MODE,HTN_GET_LOCK_KEY_REQUEST))
	{
		return NULL;
	}

    LOCK_KEY_STRUCT * p = NULL;

    p =  (LOCK_KEY_STRUCT * )getPacktFromCanBus( bus,  channel, NTH_GET_LOCK_KEY_RESPONSE, UPGRADE_MAX_RECEVIE_TIME);
    return p;
}


SERIAL_CODE_STRUCT * upGrade::getSerialCode(int bus, int channel)
{

	if(false == sendPacketToCanBusWithoutPacket((unsigned int) bus, (unsigned int) channel,CAN_CONTROL_MODE,HTN_SERIAL_CODE_REQUEST))
	{
		return NULL;
	}


     SERIAL_CODE_STRUCT * p = NULL;

     p = (SERIAL_CODE_STRUCT *)getPacktFromCanBus(bus, channel,NTH_SERIAL_CODE_RESPONSE, UPGRADE_MAX_RECEVIE_TIME);
     return p;
}


NAME_STRUCT upGrade::getname(int bus, int channel)
{

	NAME_STRUCT name_st;
    name_st.key = 0;
    name_st.name[0] = 0;
	
    if (false ==
         sendPacketToCanBusWithoutPacket((unsigned int) bus, (unsigned int) channel, CAN_CONTROL_MODE, HTN_PRODUCT_NAME_REQUEST))
    {
        return name_st;
    }

    NAME_STRUCT * p = NULL;

    p = (NAME_STRUCT * )getPacktFromCanBus(bus, channel, NTH_PRODUCT_NAME_RESPONSE, UPGRADE_MAX_RECEVIE_TIME);

    if(p) return *p;
    return name_st;
}


int upGrade::setUpgrade(int bus, int channel, int key, std::string hardwareVersion, int length)
{

	clockid_t time_end;


	if(false == sendPacketToCanBusWithUpgradePacket((unsigned int) bus, (unsigned int) channel, CAN_CONTROL_MODE, HTN_UPGRADE_REQUEST, 
	key,length))
	{
		return CAN_UP_STS::upgrade_fail;
	}


    time_end = clock() + 1000000;                    //1000ms

    while (true) {

        if (GET_PACK_FROM_QUE())
        {
			
			if (IF_PACK_FIT_IN_CONDITION(channel, NTH_UPGRADE_RESPONSE))
			{
				UPGRADE_RESPONSE_STRUCT * p = (UPGRADE_RESPONSE_STRUCT *) &package.data[1];
                if ((p->status >= CAN_UP_STS::upgrade_erase_begin) || (p->status == CAN_UP_STS::upgrade_boot) ||
                     (p->status == CAN_UP_STS::upgrade_app))
                {

                    return p->status;

				}
            }

        }

        if (clock() > time_end)
            break;
    }

    return CAN_UP_STS::upgrade_fail; //enter upgrade error
}

void upGrade::SetReportTimer(int timer)
{

	int i = 0,j=0;


	while(0 != mcu_id[i])
	{
		j = 0;
		while(-1 != mcu_bus[j])
		{

			package.head.id.can = mcu_bus[j++];
			package.head.id.channel = mcu_id[i];
			package.head.id.mode = CAN_CONTROL_MODE;
			package.data[0] = HIN_SET_TIMER_REPORT;
			package.head.id.size = 2;
			
			//the exact num   modified by marty.gong@ubtrobot.com:2017-7-17-14:40:47 
            if( -1 == timer )
            {
                int timer_temp = 1;

                //streer 50ms   modified by marty.gong@ubtrobot.com:2017-10-19-16:24:9 
                if(true == idIsSteer(mcu_id[i]))
                {
                     timer_temp = 50; 
                }
                else 
                {
                    timer_temp = 256;
                }
                
                package.data[2] = timer_temp%256;
                package.data[1] = timer_temp/256;
            }
            else
            {
                package.data[1] = timer/256;
                package.data[2] = timer%256;
            }

			printf("bus:%d,channel:%x,timer:%d,data[1]:%x,data[2]:%x\n",package.head.id.can,package.head.id.channel,timer,package.data[1],package.data[2]);
			canPkg.packageToCanBus(package);
			usleep(1000);
            canPkg.packageToCanBus(package);
			usleep(1000);
		}
		i++;
	}

}



bool upGrade::GetChassisPower(ChassisPower *chassis_power)
{
    //check the power   modified by marty.gong@ubtrobot.com:2017-9-19-10:39:27 
    const int bus_ = 1;
    const int channel_ = 0x69;
    CanPkg package;

    
    if(NULL == chassis_power) return false;
    //get info   modified by marty.gong@ubtrobot.com:2017-9-19-11:24:34 
    memset(&package,0,sizeof(package));
    memset(chassis_power,0,sizeof(ChassisPower));
    package.head.id.can = bus_;
    package.head.id.channel = channel_;
    package.head.id.size = 1;
    package.data[0] = HTN_GET_POWER_STATE_REQUEST;
  

    //send request   modified by marty.gong@ubtrobot.com:2017-9-19-10:59:5 
    canPkg.packageToCanBus(package);




  ChassisPower * p = NULL;

  p = (ChassisPower*)getPacktFromCanBus(bus_,channel_, HTN_GET_POWER_STATE_RESPONSE, UPGRADE_MAX_RECEVIE_TIME);

  if(p) 
  {
        memcpy(chassis_power,p,sizeof(ChassisPower));
        printf("the currunt power state is : %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,data[0]:%x,data[1]:%x\n",chassis_power->power.chassis_,chassis_power->power.chassis_control_,chassis_power->power.waist_,chassis_power->power.x86_,chassis_power->power._5v_,chassis_power->power.eth2Can_,chassis_power->power.switch_,chassis_power->power.lida_,chassis_power->power._12v_,chassis_power->power.recharge_,chassis_power->power.head_,chassis_power->normal.data0,chassis_power->normal.data1);

        return true;
  }
  return false;
}

bool upGrade::ChassisPowerOff()
{
    ChassisPower chassis_pwer;
    
    CanPkg package;
    int bus_ = 0x1;
    int channel_ = 0x69;
    memset(&package,0,sizeof(package));
    memset(&chassis_pwer,0,sizeof(ChassisPower));
    
    if(false == GetChassisPower(&chassis_pwer))
    {
       std::cout<<"get chassis power error."<<std::endl;
       return false;
    }

   
    chassis_pwer.power.chassis_ = 0;
    memset(&package,0,sizeof(package));
    package.head.id.can = bus_;
    package.head.id.channel = channel_;
    package.head.id.size = 2;
    package.data[0] = HTN_SET_POWER_STATE_REQUEST;


    package.data[1] = chassis_pwer.normal.data0;
    package.data[2] = chassis_pwer.normal.data1;
    
     canPkg.packageToCanBus(package);
     usleep(100*1000);  
     
     canPkg.packageToCanBus(package);

     return true;
}

bool upGrade::ChassisPowerOn()
{
    ChassisPower chassis_pwer;
    
    CanPkg package;
    int bus_ = 0x1;
    int channel_ = 0x69;
    memset(&package,0,sizeof(package));
    memset(&chassis_pwer,0,sizeof(ChassisPower));
    
    if(false == GetChassisPower(&chassis_pwer))
    {
       std::cout<<"get chassis power error."<<std::endl;
       return false;
    }

    chassis_pwer.power.chassis_ = 1;
    memset(&package,0,sizeof(package));
    package.head.id.can = bus_;
    package.head.id.channel = channel_;
    package.head.id.size = 2;
    package.data[0] = HTN_SET_POWER_STATE_REQUEST;


    package.data[1] = chassis_pwer.normal.data0;
    package.data[2] = chassis_pwer.normal.data1;
    
     canPkg.packageToCanBus(package);
     usleep(100*1000);  
     
     canPkg.packageToCanBus(package);


     return true;
}

bool upGrade::checkLowPower(int bus, int channel)
{

    //Just check PowperId   modified by marty.gong@ubtrobot.com:2017-12-20-15:37:20 
    if(BOTTOM_POWER_ID != channel) return true;

    if(NULL == getVersion(bus, channel))return false;

    return true;
}


bool upGrade::idIsSteer(int channel)
{
	int i = 0;
	while(steerId[i] != 0)
	{
		if(channel == steerId[i++] )		
		{
			return true;
		}
		
	}

	return false;
	
}

bool upGrade::idIsHeadPower(int channel)
{
    int i = 0;

   while(HeadPowerId[i] != 0)
	{
		if(channel == HeadPowerId[i++] )		
		{
			return true;
		}
		
	}

	return false;
    
}
bool upGrade::idIsChassis(int channel)
{
   int i = 0;
	while(chassisId[i] != 0)
	{
		if(channel == chassisId[i++] )		
		{
			return true;
		}
		
	}

	return false;
}

void upGrade::idPowerStopTimer(int bus,int channel)
{

    bool isId = false;
    int i = 0 ;
	while(powerId[i] != 0)
	{
		if(channel == powerId[i++] )		
		{
			isId = true;
            break;
		}
		
	}
    if(false == isId ) return ;

	package.head.id.can = bus;
	package.head.id.channel = channel;
	package.head.id.mode = CAN_CONTROL_MODE;
	package.data[0] = HIN_SET_TIMER_REPORT;
	package.head.id.size = 2;
	
	//the exact num   modified by marty.gong@ubtrobot.com:2017-7-17-14:40:47 
    package.data[1] = 0x0;
    package.data[2] = 0x0;
	printf("bus:%d,channel:%x ,data[1]:%x,data[2]:%x\n",package.head.id.can,package.head.id.channel,package.data[1],package.data[2]);
	canPkg.packageToCanBus(package);
	usleep(1000*100);
    canPkg.packageToCanBus(package);
	usleep(1000*100);

    
    
}

void upGrade::idPowerStartTimer(int bus,int channel)
{
    bool isId = false;
    int i = 0;
	while(powerId[i] != 0)
	{
		if(channel == powerId[i++] )		
		{
			isId = true;
            break;
		}
		
	}
    if(false == isId) return ;



	package.head.id.can = bus;
	package.head.id.channel = channel;
	package.head.id.mode = CAN_CONTROL_MODE;
	package.data[0] = HIN_SET_TIMER_REPORT;
	package.head.id.size = 2;
	
	//the exact num   modified by marty.gong@ubtrobot.com:2017-7-17-14:40:47 
    package.data[1] = 0x1;
    package.data[2] = 0;
	printf("bus:%d,channel:%x,data[1]:%x,data[2]:%x\n",package.head.id.can,package.head.id.channel,package.data[1],package.data[2]);
	canPkg.packageToCanBus(package);
	usleep(1000*100);
    canPkg.packageToCanBus(package);
	usleep(1000*100);
        

}

bool upGrade::waitErase(int bus, int channel)
{
    int erase = -1;
    int erase_except = -1;
    struct  timeval start;
    struct  timeval end;
    unsigned  long time_check = 15000000;//15s
    unsigned  long  diff = 0;
    char ch[UPGRADE_MAX_PRIT_SIZE]={0};

	
    InitPercentPara();

    gettimeofday(&start,NULL);

    while (true) 
   {
        if (GET_PACK_FROM_QUE())
        {
                if (IF_PACK_FIT_IN_CONDITION(channel, NTH_UPGRADE_RESPONSE))
                {
                    UPGRADE_RESPONSE_STRUCT * p = (UPGRADE_RESPONSE_STRUCT *) &package.data[1];

                    if (p->status == CAN_UP_STS::upgrade_erase_ok)
                    {
                        progressBar("erase 100%");
                        return true;
                    }

                    if ((p->status >= CAN_UP_STS::upgrade_erase_base_percent) && (true == PrintPercent(p->status - CAN_UP_STS::upgrade_erase_base_percent)))
                    {


                        std::string str;
                        sprintf(ch, "%d%%", p->status - CAN_UP_STS::upgrade_erase_base_percent);
                        str = ch;

                        if (erase != p->status)
                            progressBar("erase " + str);
                        erase = p->status;
                    }
                    else
                    {
                       if(p->status < CAN_UP_STS::upgrade_erase_base_percent)
                       {

                           if (erase_except != p->status)
                           {
                           
                               sprintf(ch,"erase receive erase status : %d ", p->status);
                               progressBar(ch);
                           }
                           erase_except = p->status;
                       }
                    }

                }
            }
        gettimeofday(&end,NULL);
        diff = 1000000 * (end.tv_sec-start.tv_sec)+ end.tv_usec-start.tv_usec;
		if (diff > time_check)//check timeout
		{
             progressBar("erase timeout.");
             break;
        }

    }

    return false;
}



/*
 fun:void upGrade::PrintPercent(unsigned int percent)
 ilu: print the percent when in %30,60%,90% .you should init the upgrade_percent_ para when using it.
 para: percent [0,100]
*/
bool upGrade::PrintPercent(unsigned int percent)
{
    bool flag = false;

    if ((percent < PRIT_PERCNT::print_percent_30) && (false == upgrade_percent[0])) //
    {
        upgrade_percent[0] = true;
        flag = true;
    }
    else if ((PRIT_PERCNT::print_percent_30 <= percent && percent < PRIT_PERCNT::print_percent_60) && (false == upgrade_percent[1])) {
        upgrade_percent[1] = true;
        flag = true;
    }
    else if ((PRIT_PERCNT::print_percent_60 <= percent && percent < PRIT_PERCNT::print_percent_100) && (false == upgrade_percent[2])) {
        upgrade_percent[2] = true;
        flag = true;
    }
    else if ((PRIT_PERCNT::print_percent_100 == percent) && (false == upgrade_percent[3])) {
        upgrade_percent[3] = true;
        flag = true;

    }
    else // the percent can not in this case;
    {
        return false;
    }
    return flag;
}


void upGrade::InitPercentPara()
{
    int i = 0;

    for (i = 0; i < UP_GRADE_PERCENT_SETION; i++) {
        upgrade_percent[i] = false;
    }

}


int upGrade::programming(int bus, int channel, int address, u32 code, int time_out)
{

    clockid_t time_end;


	if(false == sendPacketToCanBusWithProgramPacket((unsigned int) bus, (unsigned int) channel,CAN_CONTROL_MODE,HTN_PROGRAMMING_REQUEST, address, code))
	{
		return -1;
	}


    usleep(100 * time_out);
    time_end = clock() + 6000;                      //6ms

    while (true) 
	{
        if (GET_PACK_FROM_QUE())
		{
            if (IF_PACK_FIT_IN_CONDITION(channel, NTH_PROGRAMMING_RESPONSE))
			{
                CAN_PACKAGE_CMD_STRUCT * p = (CAN_PACKAGE_CMD_STRUCT *) &package.data;

                // char ch[128];
                //std::string str;
                // sprintf(ch,"address:%d  data:%.8x,redata:%.8x",p->body.programming_response.next_address,code,p->body.programming_response.pre_code);
                // str = ch;
                // progressBar("programming " + str);
                if (code == p->body.programming_response.pre_code) {
                    // usleep(1000);
                    // progressBar("------------------" + str);
                    return p->body.programming_response.next_address;

                }
            }
        }

        if (clock() > time_end) 
		{
            break;
        }

    }

    return - 1;
}



std::string upGrade::progradeEnd(int bus, int channel, u32 crc)
{

    clockid_t time_end;


	if(false == sendPacketToCanBusWithProgramEndPacket((unsigned int) bus, (unsigned int) channel, CAN_CONTROL_MODE,HTN_PROGRAM_END_REQUEST, crc))
	{
		return "error";
	}



    time_end = clock() + 1000000;                     //1000ms

    while (true)
	{
        if (GET_PACK_FROM_QUE()) 
		{
            if (IF_PACK_FIT_IN_CONDITION(channel, NTH_PROGRAM_END_RESPONSE)) 
			{
                CAN_PACKAGE_CMD_STRUCT * p = (CAN_PACKAGE_CMD_STRUCT *) &package.data;

                // ROS_ERROR("receive_crc:%.4x,crc:%.4x",p->body.program_end_response.receive_crc ,crc);
                //p->body.program_end_response.receive_crc = crc;
                if (p->body.program_end_response.receive_crc == crc) 
				{
                    // ROS_ERROR("end usleep 33333333333333333333333");
                    if (p->body.program_end_response.status == 0) {
                        // ROS_ERROR("ok:program end ok");
                        return "ok:program end ok";
                    }
                    else if (p->body.program_end_response.status == 2)
                    {
                        progressBar("pro:program check error");
                        return "pro:program check error";
                    }
                }
                else if (p->body.program_end_response.status == 1)
                {
                    progressBar("crc:program crc error");
                    return "crc:program crc error";
                }
            }
        }
        else{
            // progressBar("the queue is empty.");
        }

        if (clock() > time_end)
            break;

    }

    return "error:timeout.";
}


void upGrade::progressBar(std::string str)
{
    if (callback_)
        callback_(str + "\n");
}


int upGrade::getActionFromFile(const char *name,char **buffer)
{
     //int lenth;
    if (name == NULL)
        return 0;

    FILE * fp = fopen(name, "rb");

    if (fp == NULL) 
	{
        printf("getActionFromFile failed to open file:%s\n", name);
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


bool upGrade::compare(std::string command, std::string cmd_name)
{
    if (command.size() < cmd_name.size())
        return false;

    // ROS_INFO("command:%s,cmd_name:%s",command.c_str(),cmd_name.c_str());
    std::string tem = command.substr(0, cmd_name.size());
    std::transform(tem.begin(), tem.end(), tem.begin(), ::toupper);
    std::transform(cmd_name.begin(), cmd_name.end(), cmd_name.begin(), ::toupper);

    // ROS_INFO("tolower --command:%s,cmd_name:%s",command.c_str(),cmd_name.c_str());
    if (tem == cmd_name)
        return true;

    return false;

}



void upGrade::cal_crc(unsigned int ptr)
{
    unsigned int xbit;
    unsigned int data;

    unsigned int dwPolynomial = UPGRADE_CRC_POLYNOMIAL;

    //  while (len--)
    {
        xbit = 1 << 31;

        data = ptr;

        for (int bits = 0; bits < 32; bits++) {
            if (CRC & 0x80000000) {
                CRC <<= 1;
                CRC ^= dwPolynomial;
            }
            else 
                CRC <<= 1;

            if (data & xbit)
                CRC ^= dwPolynomial;

            xbit >>= 1;
        }
    }

}


/*
can 总线BE状态变化说明：
准备阶段：期望收到 0,1,5,6以及【100-200】数据； 如果收到0，状态变为tmp状态。如果收到 
，1,5,6,以及以上。状态变为 升级状态。

tmp状态，收到 1,5,6以及【100-200】。状态变为升级状态。

*/
std::string upGrade::enterUpgrade(int bus, int channel, int vid, int pid, std::string hardware_version, int length, 
    int expectStatus)
{
    char ch[64];

    std::string str;
    std::string re;
    LOCK_KEY_STRUCT * lock_key = NULL;
    int i = 0;


    for (i = 0; i < 2; i++) {
        lock_key = getKey(bus, channel);

        if (NULL != lock_key)
            break;
    }

    if (!lock_key) {
        progressBar("enter upgrade read key error");
        return "error:enter upgrade read key error";
    }
    sprintf(ch, "key:%.2x", lock_key->key);

    // std::cout << ch << std::endl;
    int key = lock_key->key;
    int upgrade_re;

    upgrade_re = setUpgrade(bus, channel, key, hardware_version, length);
    sprintf(ch, "%d", upgrade_re);
    re = ch;

    //canprepare 阶段
    if ((CAN_EXPECT_STATUS_PRE == expectStatus)) {
        if (CAN_UP_STS::upgrade_app == upgrade_re) {
            progressBar("enter boot");
            return "enterBoot";

        }

        if (CAN_UP_STS::upgrade_erase_begin <= upgrade_re || CAN_UP_STS::upgrade_boot == upgrade_re) {
            progressBar("srart upgrade");
            return "srartUpgrade:" + re;
        }
        else {
            progressBar("enter upgrade error" + re);
            return "error:enter upgrade  error" + re;
        }
    }
    else if ((CAN_EXPECT_STATUS_TMP == expectStatus)) {
        if (CAN_UP_STS::upgrade_erase_begin <= upgrade_re || CAN_UP_STS::upgrade_boot == upgrade_re) {
            progressBar("srart upgrade");
            return "srartUpgrade:" + re;
        }
        else {
            progressBar("enter upgrade error" + re);
            return "error:enter upgrade  error" + re;
        }
    }
    progressBar("enter upgrade error time out ");
    return "error:enter upgrade  error ";

}


std::string upGrade::checkhardvare(int bus, int channel, int vid, int pid, std::string hardware_version)
{
    char ch[64];

    std::string str;
    std::string re;
    PRODUCT_CODE_STRUCT * product_code_p = getProductCode(bus, channel);

    if (!product_code_p)
        product_code_p = getProductCode(bus, channel);

    if (!product_code_p) {
        progressBar("read product code error");
        return "error:read product code error";
    }

    if ((product_code_p->vid != vid) || (product_code_p->pid != pid)) {

        sprintf(ch, "pid:%d vid:%d", product_code_p->pid, product_code_p->vid);
        str = ch;
        str = "read product " + str + "error";
        progressBar("read product pid vid error");
        return "error:read product pid vid  error";
    }

    if ((product_code_p->mode == 'a') || (product_code_p->mode == 'A'))
        re = "app";
    else if ((product_code_p->mode == 'b') || (product_code_p->mode == 'B'))
        re = "boot";
    else {
        progressBar("error:invalid mode");
        return "error:invalid mode";
    }

    /* boot area need not check version when not firsttime upgrading*/
    if ('b' == product_code_p->mode || 'B' == product_code_p->mode)
        return re;

    VERSION_STRUCT * version = getVersion(bus, channel);

    if (!version)
        version = getVersion(bus, channel);

    if (!version) {
        progressBar("read version error");
        return "error:read version error";
    }

	if(true == idIsSteer(channel))
	{

	}
	else
	{
		sprintf(ch, "%d.%.2d", version->norm.hardware_version_high, version->norm.hardware_version_low);

	}
    str = ch;
    std::string str_version = hardware_version;

    if (str != hardware_version)
        return "error:hardware_version error:" + hardware_version;

    return re;

    if (str.size() > str_version.size()) {
        progressBar("hardware_version " + hardware_version + " error");
        return "error:hardware_version error" + hardware_version;
    }

    while (!compare(str_version, str)) {
        int para = str_version.find_first_of(';', 0);

        if (para < 0)
            continue;
        str_version = str_version.substr(para + 1, str_version.size());

        if (str.size() > str_version.size()) {
            progressBar("hardware_version" + hardware_version + ":cant't find" + str);
            return "error:hardware_version error :" + str;
        }
        
    }



    return re;


}

/**************************************************************************
 * 函数名: upgradeCheckOK
 * 类型: 普通类型函数
 * 功能: 检查mcu是否能从bios 跳转到app 区。用来解决测试中发现的底部电源板无法正常从bios区跳转到app区的bug；
 * 参数:
 *    @[in ] 
 *    @[out]
 * 返回值: true:正常跳转，false：不能正常跳转
 * 时间: 2017/11/14:20:35:00
 * 作者: marty.gong
 **************************************************************************/

bool upGrade::upgradeCheckOK(int bus, int channel)
{
    
	struct  timeval start;
    struct  timeval end;
    unsigned  long diff = 0;
    bool first_timeout = true;

    PRODUCT_CODE_STRUCT *p = NULL;
    //等待mcu跳转，避免mcu在执行过程中接收信息死机的问题。底部电机容易发生   modified by marty.gong@ubtrobot.com:2018-1-31-14:48:30 
    sleep(3);
    if(idIsChassis(channel))sleep(20);
    gettimeofday(&start,NULL);
    while(true)
    {
        if(NULL != (p = getProductCode(bus,channel)))
        {
             if('a' ==  p->mode || 'A' == p->mode)
             {
                return true;
             }
             else if(false == first_timeout)
             {
                progressBar("upgrade status check false! the status is in bios.");
                return false;
             }
        }

        gettimeofday(&end,NULL);
		diff = 1000000 * (end.tv_sec-start.tv_sec)+ end.tv_usec-start.tv_usec;
		if (diff > 10000000)//10s
		{
            if(false == first_timeout)
            {
                progressBar("upgrade status check false! we can not receive any response.");
                break; 
            }
            first_timeout = false;
        }
        usleep(100*1000);
        
    }
        
    return false;
    
}
std::string upGrade::upGrading(int bus, int channel, int vid, int pid, std::string hardware_version, std::string file)
{

    u8 *update_data=NULL;
    char ch[UPGRADE_MAX_PRIT_SIZE] = {0};

    std::string str;
    std::string check;

    int fileLength = getActionFromFile(file.c_str(), (char **)&update_data);
    int read_length = fileLength;

    // sprintf("ch,read_length");
    if (fileLength <= 0) {
        progressBar("read update file error");
        return "read update file error";

    }
    sprintf(ch, "%d", read_length);
    str = ch;
    progressBar("read update file length:" + str);

    //status:pre
    check = enterUpgrade(bus, channel, vid, pid, hardware_version, read_length, CAN_EXPECT_STATUS_PRE);

    if ((true == CAN_UPGRADE_RETRY) && (compare(check, "error"))) {

        for (int i = 0; i < 10; i++) {
            if (compare(check, "error")) {
                usleep(1000 * 500);
                check = enterUpgrade(bus, channel, vid, pid, hardware_version, read_length, CAN_EXPECT_STATUS_PRE);
            }
            else {
                break;
            }
        }

    }

    if (compare(check, "error")) {
        progressBar("enter Boot error");
        delete[] update_data;
        return "error:enter Boot error";
    }

    //status:tmp
    if (compare(check, "enterBoot")) {
        check = enterUpgrade(bus, channel, vid, pid, hardware_version, read_length, CAN_EXPECT_STATUS_TMP);

        if ((true == CAN_UPGRADE_RETRY) && (compare(check, "error"))) {

            for (int i = 0; i < 10; i++) {

                if (compare(check, "error")) {
                    usleep(1000 * 500);             //这里改成500ms,重传10次(5s)
                    check = enterUpgrade(bus, channel, vid, pid, hardware_version, read_length, 
                        CAN_EXPECT_STATUS_TMP);
                }
                else {
                    break;
                }
            }
        }

        if (compare(check, "error")) {
            progressBar("enter upgrade error");
            delete[ ] update_data;
            return "error:enter upgrade error";
        }
    }


	// erease
    if (false == waitErase(bus, channel))
	{
        bool eraseFlag = false;

        for (int i = 0; i < UPGRADE_ERASE_CRC_WAITE_NUM; i++) {
            //此处必须休息一会，等待收bf；
            usleep(1000 * 100);

            if (true == waitErase(bus, channel)) {
                eraseFlag = true;
                break;
            }
        }

        //擦除flash失败，timeout
        if (false == eraseFlag) {
            progressBar("erase flash error");
            delete[] update_data;
           return "error: erase flash error";
        }
    }

    //ROS_ERROR("waitErase(bus,channel);");
	//crc
	progressBar("cal_crc");
    int address = 0;

    reset_crc();

    while (address < read_length) 
	{
        cal_crc(* (u32 *) &update_data[address]);
        address += UPGRADE_TRANS_STEP_ONCE;
    }
	// trans data
    progressBar("start programming");
    address = 0;
    int percent = -1;
    int address_old = 0;
    int error_number = 0;

    InitPercentPara();
	u32 data = 0;
    while (address < read_length)
	{
        data = * (u32 *) &update_data[address];

        int add = programming(bus, channel, address, data,UPGRADE_TRANS_WAITE_TIME);

        if (error_number > UPGRADE_RETRANS_NUM)
            break;

        // ROS_INFO("-----add=%d add=%x-----address=%x,read_length=%x\n",add,add,address,read_length);
        if (add < 0) {
            // usleep(1000*50);
            error_number++;
            continue;
        }

        address = add;

        if (address <= address_old) {
            error_number++;
        }
        else 
        {
            if(error_number > 10)
            {
                sprintf(ch, "program:after %d the address ok.", error_number);
                progressBar(ch);
            }
            error_number = 0;
        }

        address_old = address;

        sprintf(ch, "address:%d  %d%%", address, address * UPGRADE_PERCENT_BASE / read_length);
        str = ch;

        if (percent != (address * UPGRADE_PERCENT_BASE / read_length)) {
            percent = address * UPGRADE_PERCENT_BASE / read_length;

            if ((percent > 0) && (true == PrintPercent((unsigned int) percent))) {
                progressBar("programming " + str);
            }
        }

        if (address >= read_length)
            break;
    }
    progressBar("end programming");

    if (error_number > UPGRADE_RETRANS_NUM) 
    {
        progressBar("programming error " + str);
        delete[] update_data;
        return "error:programming error " + str;
    }

  //compare crc
    for (int i = 0; i < UPGRADE_ERASE_CRC_WAITE_NUM; i++)
   {
        check = progradeEnd(bus, channel, get_crc());
        
        if (true == compare(check, "ok")) {
            break;
        }


    }
    if (false == compare(check, "ok"))
    {
        std::cout<<check<<std::endl;
        progressBar(check);
        delete[] update_data;
        return "error";
    }

  //需要检查状态有没有跳转   modified by marty.gong@ubtrobot.com:2017-11-13-18:15:6 
   if(false == upgradeCheckOK(bus,channel))
   {
       std::cout<<"the mcu can not jump from bios to app."<<std::endl;
       progressBar("the mcu can not jump from bios to app.");
       return "error:progradeEnd error!";
   }
   std::cout<<"the mcu has jump from bios to app succesfully."<<std::endl;
   progressBar("the mcu has jump from bios to app succesfully.");

   delete[] update_data;
    return "ok";

}



