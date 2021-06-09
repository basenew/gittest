#ifndef __PLATFORM_UDOCK_DATA_H__
#define __PLATFORM_UDOCK_DATA_H__

#define HAVE_TRY    1

typedef enum dockSDKCMD_{
    CANCEL_DOCK,
    BEGAIN_DOCK,
    PAUSE_DOCK,
    CANCEL_PAUSE,
}dockSDKCMD;

typedef enum UdockMsg_
{
    S_BEGAIN_DOCK = 8001,           //开始自动回充
    S_MOVING  = 8002,               //正在回充运动
    S_BLUETOOTH_PARING = 8003,      //正在蓝牙匹配
    S_DOCKING = 8004,               //正在对桩
    S_SWITCH_ON = 8005,             //行程开关打开
    S_COLLISION_BAR_ON = 8006,      //防撞条打开
    S_CHARGING = 8007,              //正在充电
    S_FAIL_MOVE = 8010,            //回充运动失败
    S_FAIL_LOCALIZITION = 8011,    //回充定位失败
    S_FAIL_BLUETOOTH_PARE = 8012,  //回充蓝牙匹配失败
    S_FAIL_DOCK = 8013,            //回充对桩失败
    S_FAIL_CHARGE = 8014           //回充充电失败

}UdockMsg;

//回充状态
typedef enum dockSDKState_{
    RECEIVE_CMD,    //收到指令开始回充
    BLUETOOTH_PARE, //开始蓝牙匹配
    MOVE_POINT,     //开始运动
    MOVE_POINT_LAST_STEP,//开始对进充电口
    SWITCH_ON,      //行程开关打开
    WAITING_CHARGE, //等待充电
    DOCK_SUCESS,    //对桩成功
    IDLE,           //空闲状态
    BUSY,           //忙碌状态
    PAUSE,          //
    BLUETOOTH_PARE_FAIL,    //蓝牙匹配失败
    CMD_CANCEL_FAIL,        //取消失败
    MOVE_FAIL,              //运动失败
    DOCK_PRECISE_FAIL,      //定位失败
    DOCK_TIME_OUT_FAIL,     //超时失败
    DOCK_FAIL,              //回充失败
    RECEIVE_LEAVE_CMD
}dockSDKState;

typedef struct TrackMovePara_{
    float angle1;
    float distance;
    float angle2;
}TrackMovePara;

#endif
