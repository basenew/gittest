/*
    Udock common by xiangbin.huang
*/
#ifndef __UDOCK_COMMON_H__
#define __UDOCK_COMMON_H__

//#define     _DOCK_1_1_
//#define     _CHASSIS_MARSHELL_

//回充参数
#define DEFAULT_LINE_DISTANCE           2.50    //充电墙壁宽度
#define ERRO_DISTANCE                   0.05    //
#define ERRO_DISTANCE_FIRST_STAGE       0.10

#ifndef _CHASSIS_MARSHELL_
    #define CENTER_DOCK_DISTANCE    0.4     //机器人中心距离充电头距离
#else
    #define CENTER_DOCK_DISTANCE    0.3     //机器人中心距离充电头距离
    #define LAST_MOVE_DISTANCE      1.1f
#endif


#if  (defined _POLICE_) && (defined _CHASSIS_JC_) //公安定制履带版
    #define LIDAR_POSITION          0.218     //雷达安装距离机器人中心位置
    #define POINT4_DISTANCE         (0.8 + 0.18 + CENTER_DOCK_DISTANCE)     //机器人回充点
    #define POINT3_DISTANCE         (0.4 + POINT4_DISTANCE)                 //机器人回充点
    #define POINT2_DISTANCE         (0.4 + POINT3_DISTANCE)                 //机器人回充点前方

#elif (defined _POLICE_) && (defined _CHASSIS_MARSHELL_)//公安定制轮式版
    #define LIDAR_POSITION          0.264     //雷达安装距离机器人中心位置
#ifdef _DOCK_1_1_
    #define POINT4_DISTANCE         (0.8 + 0.18 + CENTER_DOCK_DISTANCE-0.42)     //机器人回充点
#else
    #define POINT4_DISTANCE         (0.8 + 0.18 + CENTER_DOCK_DISTANCE)                 //机器人回充点前方
#endif
    #define POINT3_DISTANCE         (0.4 + POINT4_DISTANCE)                 //机器人回充点
    #define POINT2_DISTANCE         (0.4 + POINT3_DISTANCE)                 //机器人回充点前方
#elif (defined _CHASSIS_JC_)    //履带版
    #define LIDAR_POSITION          0.4     //雷达安装距离机器人中心位置
    #define POINT4_DISTANCE         (0.8 + 0.18 + CENTER_DOCK_DISTANCE)     //机器人回充点
    #define POINT3_DISTANCE         (0.4 + POINT4_DISTANCE)                 //机器人回充点
    #define POINT2_DISTANCE         (0.4 + POINT3_DISTANCE)                 //机器人回充点前方
#elif (defined _CHASSIS_MARSHELL_) //轮式版
    #define LIDAR_POSITION          0.264      //雷达安装距离机器人中心位置
#ifdef _DOCK_1_1_
    #define POINT4_DISTANCE         (0.8 + 0.18 + CENTER_DOCK_DISTANCE-0.42)     //机器人回充点
#else
    #define POINT4_DISTANCE         (0.8 + 0.18 + CENTER_DOCK_DISTANCE)     //机器人回充点               //机器人回充点前方
#endif
    #define POINT3_DISTANCE         (0.4 + POINT4_DISTANCE)                 //机器人回充点
    #define POINT2_DISTANCE         (0.4 + POINT3_DISTANCE)                 //机器人回充点前方
#elif (defined _CHASSIS_4WD_) // four wheel drive //TODO 待测量
    #define LIDAR_POSITION          0.01    //雷达安装距离机器人中心位置
#ifdef _DOCK_1_1_
    #define POINT4_DISTANCE         (0.8 + 0.18 + CENTER_DOCK_DISTANCE-0.42)     //机器人回充点
#else
    #define POINT4_DISTANCE         (0.8 + 0.18 + CENTER_DOCK_DISTANCE)     //机器人回充点               //机器人回充点前方
#endif
    #define POINT3_DISTANCE         (0.4 + POINT4_DISTANCE)                 //机器人回充点
    #define POINT2_DISTANCE         (0.4 + POINT3_DISTANCE)                 //机器人回充点前方
#endif


#define DOCK_OFFSET   (0.42+0.38)

//回充识别参数
#define LIDAR_OFFSET            0.0
#define SIMPLE_ERRO_MAX         24  //degree
#define PRECISE_ERRO_MAX        12  //degree

#ifndef _CHASSIS_MARSHELL_
    #define LIDAR_TOTAL_NUM_VELODYNE    897     //velodyne lidar
    #define LIDAR_NORMAL_MID            449
    #define LIDAR_NORMAL_START      (LIDAR_NORMAL_MID-25)
    #define LIDAR_NORMAL_END        (LIDAR_NORMAL_MID+25)
#else
    #define LIDAR_TOTAL_NUM_VELODYNE    448     //sick lidar
    #define LIDAR_NORMAL_MID            224
    #define LIDAR_NORMAL_START      (LIDAR_NORMAL_MID-20)
    #define LIDAR_NORMAL_END        (LIDAR_NORMAL_MID+20)
#endif

#define M2_PI                       6.28318531f
#define DEBUG_UDOCK                 0       //debug switch
#define USE_GSUDP_DATA              0       //udp lidar data
#define USE_GSROSERIAL_DATA         1       //rosserial data
#define COLLISITION_DETECT          1       //obstacle avoid

//避障参数设置
#if  (defined _POLICE_) && (defined _CHASSIS_JC_) //公安定制履带版
    #define     VEHICLE_WIDTH   0.98f
    #define     VEHICLE_LENGTH  1.332f
    #define     LIDAR_INSTALL_POSITION  0.218f
#elif (defined _POLICE_) && (defined _CHASSIS_MARSHELL_)//公安定制轮式版
    #define     VEHICLE_WIDTH   0.90f
    #define     VEHICLE_LENGTH  1.332f
    #define     LIDAR_INSTALL_POSITION  0.404f
#elif (defined _CHASSIS_JC_)        //履带版
    #define     VEHICLE_WIDTH   0.98f
    #define     VEHICLE_LENGTH  1.332f
    #define     LIDAR_INSTALL_POSITION  0.404f
#elif (defined _CHASSIS_MARSHELL_) //轮式版
    #define     VEHICLE_WIDTH   0.98f
    #define     VEHICLE_LENGTH  1.332f
    #define     LIDAR_INSTALL_POSITION  0.404f
#elif (defined _CHASSIS_4WD_) //四轮四转 //TODO         
    #define     VEHICLE_WIDTH   0.55f
    #define     VEHICLE_LENGTH  0.74f
    #define     LIDAR_INSTALL_POSITION  0.3839f
#endif

#define     OBSTACLE_FRONT_DIS      0.3f
#define     OBSTACLE_FRONT_EXPAND   0.05f

#define     OBSTACLE_FRONT_EXPAND0   0.05f
#define     OBSTACLE_FRONT_EXPAND1   0.06f
#define     OBSTACLE_FRONT_EXPAND2   0.12f

//运动参数
#define     DOCK_LINEAR_SPEED    8
#define     DOCK_ANGULAR_SPEED   8


#endif
/*
    END file Udock algrithm by xiangbin.huang
*/
