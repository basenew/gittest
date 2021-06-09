#include <boost/thread.hpp>
#include "can_upgrade/include/datarouterJointSet.h"
#include "tiny_ros/ros.h"
#include "ros/ros.h"
#include "firmware.h"
#include "string.h"
#include <getopt.h>
using namespace std;

#include <signal.h>
#include <curl/curl.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71


int kfd_ = 0;
struct termios cooked_, raw_;

int learn=0;
static const struct option long_option[]={
    {"board",optional_argument,NULL,'b'},
    {"all",required_argument,NULL,'a'},
    {"pressure",no_argument,NULL,'p'},
    {"time",required_argument,NULL,'t'},
    {NULL,0,NULL,0}
};

void CanupgradeDebug(dataRouterJointSet *upgrade)
{
    char c;
    char ch[10];
    std::string cmd;

    tcgetattr(kfd_, &cooked_);
    memcpy(&raw_, &cooked_, sizeof(struct termios));
    raw_.c_lflag &=~ (ICANON | ECHO);

    raw_.c_cc[VEOL] = 1;
    raw_.c_cc[VEOF] = 2;
    tcsetattr(kfd_, TCSANOW, &raw_);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");
    puts("otherwise the key values will be printed");


    for(;;)
    {

      if(read(kfd_, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
      }

      sprintf(ch,"%d",c);
      cmd = ch;



      {
            std::string request;
            std::string response;
          request = cmd;
          if (upgrade->configServerSrv(request,response))
          {

              printf("\r                                                    \r");
              printf("%s",response.c_str());

              fflush(stdout);

          }
      }

    }
    return;
}

void print_usage()
{
    printf("invalid usage!!!\r\n");
    printf("usage: >------ ");
    printf("if you want to upgrade board single one by one\r\n");
    printf("enter the command like this: \r\n");
    printf("./canUpTest --all 0 --board=monitor    or    ./canUpTest --all 0 --board=chassis_controller \r\n");
    printf("if you want to upgrade all of the boards (monitor and chassis controller)\r\n");
    printf("enter the command like this : \r\n");
    printf("./canUpTest --all 1 \r\n");
    printf("if you want to do pressure test\r\n");
    printf("./canUpTest --pressure --time 100 , 100 is the num of times to upgrade all mcus\r\n");
}

int doPressureTest(int num_of_times)
{
    int numTimesUpgraded = 0;
    int numTimesSuccess = 0;
    int numTimesFailed = 0;
    while(numTimesUpgraded < num_of_times)
    {
        if (!Firmware::get_instance()->upgrade()) {

            printf("%s upgrade all mcu success...",__FUNCTION__);
            numTimesSuccess++;

        } else {
            // failed
            printf("%s upgrade all mcu failed",__FUNCTION__);
            numTimesFailed++;
        }

        numTimesUpgraded++;
        printf("num of time upgrade = %d\r\n", numTimesUpgraded);
        printf("num of time success = %d\r\n", numTimesSuccess);
        printf("num of time failed = %d\r\n", numTimesFailed);
        sleep(5);
    }
    
    printf("-----------------summary---------------\r\n");
    printf("total number of times to upgrade : %d\r\n", num_of_times);
    printf("number of times upgrade success : %d\r\n", numTimesSuccess);
    printf("number of times upgrade failed : %d\r\n", numTimesFailed);
    printf("---------------------------------------\r\n");
    return 0;
}


int  main(int argc , char *argv[])
{
    tinyros::init("canUpTest");
    ros::init(argc, argv, "canUpTest");
    ros::Time::init();

    int opt=0;
    int isAll = -1;
    bool isPressureTest = false;
    char board_name[50] = {0};
    std::string boardName = "";
    int totalTimesUpgrade = 0;
    bool isPressureTestFinish = false;
    //int numTimesUpgraded = 0;

    while((opt = getopt_long(argc,argv,"n:l",long_option,NULL))!=-1)
    {
        switch(opt)
        {
            case 'a':
                printf("if it is all mcu upgraded\r\n");
                isAll = atoi(optarg);
                printf("is all upgraded is set to %d\r\n",isAll);
            break;
            case 'b':
                if(optarg == NULL)
                {
                    printf("optarg is NULL!!!!\r\n");
                    return -1;
                }
                else
                {  
                    printf("board name:%s \r\n",optarg);
                    printf("string length = %d\r\n",(int)(strlen(optarg)));
                    strncpy(board_name, optarg, strlen(optarg));
                    board_name[strlen(optarg)] = '\0';
                    printf("board name in string : %s\r\n",board_name);
                }
            break;
            case 'p':
                printf("pressure test is set to true\r\n");
                isPressureTest = true;
                
            break;
            case 't':
                totalTimesUpgrade = atoi(optarg);
                printf("total number of times for pressure test = %d\r\n", totalTimesUpgrade);
	    break;
            default:
                printf("unknown argument\r\n");
            break;
        }
    }

    //signal(SIGPIPE, SIG_IGN);
    //curl_global_init(CURL_GLOBAL_ALL);
    
    //dataRouterJointSet mcuUpgrade_;
    //mcuUpgrade_.init();
    //CanupgradeDebug(&mcuUpgrade_);
    
    //curl_global_cleanup();

    // first we check if it is pressure test
    if(isPressureTest && totalTimesUpgrade != 0)
    {
        printf("&&& pressure test start &&&\r\n");
        doPressureTest(totalTimesUpgrade);
	isPressureTestFinish = true;
    }
    else
    {
        printf("not doing pressure test\r\n");
    }

    if(isPressureTestFinish)
    {
	printf("&&& pressure test end &&&\r\n");
	return 0;
    }

    // first we check if all of the arguments are valid input arguments

    if(isAll == 0 || isAll == 1)
    {
        printf("--all argument ok is all is set to %d\r\n", isAll);
    }
    else
    {
        printf("--all argument is invalid\r\n");
        print_usage();
        return -1;
    }

    boardName = board_name;
    if(boardName == "monitor" || boardName == "chassis_controller" || boardName.empty())
    {
        printf("boardName filed ok , board name should either be monitor, controller or empty, board name = %s\r\n", boardName.c_str());
    }
    else
    {
        printf("board name invalid , board name = %s\r\n", boardName.c_str());
        print_usage();
        return -1;
    }
    
    
    if(isAll)
    {
        printf("upgrade all mcu manually ...\r\n");
        if(!boardName.empty())
        {
            printf("boardName field should be empty\r\n");
            print_usage();
            return -1;
        }
        else
        {
            Firmware::get_instance()->upgradeAllMcuManual();
        }

    }
    else
    {
        printf("upgrade single mcu board manually ...\r\n");
        if(boardName == "monitor" || boardName == "chassis_controller")
        {
            //Firmware::get_instance()->setUpgradeManualCtrl(true);
            //Firmware::get_instance()->upgradeInit();
            Firmware::get_instance()->upgradeSingleMcuManual(boardName);
        }
        else
        {
            printf("board name field invalid , board name : %s\r\n", boardName.c_str());
            print_usage();
            return -1;
        }
    }

    return 0;
}
