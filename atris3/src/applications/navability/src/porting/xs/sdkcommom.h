

#ifndef SDK_COMMOM_H
#define SDK_COMMOM_H

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#	define SDK_DECL_EXPORT __declspec(dllexport)
#	define SDK_DECL_IMPORT __declspec(dllimport)
#else
#	define SDK_DECL_EXPORT
#	define SDK_DECL_IMPORT
#endif

#if defined(XSTSDK_EXPORT)
#  define XS_TECH SDK_DECL_EXPORT
#else
#  define XS_TECH SDK_DECL_IMPORT
#endif





#define Q_DECL_EXPORT
#define Q_DECL_IMPORT

#include <stdlib.h>
#include <vector>
#include <string>
#include <string.h>
#include <stdint.h>
#include <memory>


namespace xs_sdk
{
    enum ErrorCode
    {
        EC_SUCC = 0,
        EC_ERR,
        EC_SESSION_DISCONNECT,
        EC_SESSION_SEND_FAIL,
        EC_NO_IMP,
        EC_BUFF_OVERFLOW,
        EC_DUPLICATED_CID,
        EC_COMPONENT_FALURE,
        EC_MAX
    };

    using namespace std;

    struct SimpleResponse
    {
        int32_t call_id;
        //应答结果
        int32_t result;
        //应答消息
        std::string message;
    };

//    struct StationInfo
//    {
//        string station_id;
//        string station_name;
//        //经度
//        double longitude;
//        //维度
//        double latitude;
//        //方向
//        double azimuth;

//        enum StationType
//        {
//            STATION =0,
//            CHARGE = 1,
//            PATROL = 2,
//            GRID = 10000,
//        };
//        StationType type;
//    };

    struct StationInfo
    {
        string station_id;
        string station_name;
        //经度
        double longitude;
        //维度
        double latitude;
        //方向
        double azimuth;
        enum StationType
        {
            STATION = 0,
            CHARGE = 1,
            PATROL = 2, // 不再使用
            GRID = 0x1000,// 不再使用
            GRID_CHARE = 0x1001, // 不再使用
            GRID_NO_ANGLE = 0x1010, //不再使用
            EXACT_STATION = 3,
        };

        StationType type;
        //楼栋数
        string building; //二级名
        //单元数
        string uint; //三级名
        //层数
        string floor; //四级名
        int32_t region_id; //子地图编码id

        enum StationPositionType
        {
            WGS84 = 0,
            GAUSS = 1,
        };
        std::vector<StationPositionType> position_types;
        double gauss_x;
        double gauss_y;
        //关联id:目前用于关联路网
        int32_t association_id;
    };


    struct TaskConfig
    {
        enum SpeedLevel
        {
            NORMAL = 0,
            LOW = 1,
            HIGH = 2,
        };
        SpeedLevel speed_level;

        enum ObstacleType
        {
            OBSTACLEAVOID = 0,
            OBSTACLESTOP = 1,
        };
        ObstacleType obstacle_type;
    };

    struct GridInfo
    {
        int32_t grid_height;
        int32_t grid_width;
        int32_t origin_x;
        int32_t origin_y;
        int32_t resolution;
    };

    struct MapInfo
    {
        string id;
        string name;
        string create_time;
        GridInfo grid_info;

        enum MapType
        {
            LOCAL_MAP = 0,
            REMOTE_MAP = 1,
        };

        MapType type ;
        //楼栋数
        string building; //二级名
        //单元数
        string uint; //三级名
        //层数
        string floor; //四级名
        int32_t region_id;//子地图编码id
    };

    struct MapperControl
    {
        enum ControlType
        {
            START_GMAPPER = 0,
            STOP_MAPPER = 1,
            SAVE_MAPPER = 2,
            DONOT_SAVE_MAPPER = 3,
            GET_TILE_MAP = 4, // not support
            START_INCREASE_MAPPER = 5,
            GENERATE_MAP = 6,
            ADD_CLOSURE_POINT = 7,
            MATCH_CLOSURE_POINT = 8,
            PAUSE_MAPPERR = 9,
            RESUME_MAPPER = 10,
        };
        ControlType  control_type;
        string map_name; // used when START_GMAPPER
        MapInfo info; //used when MULTI_LAYER_MAPPER
    };

    struct Subscribe
    {
        enum SubscribeType
        {
            subscribe = 0,//订阅
            unSubscribe=1//取消订阅
        };
        //订阅的主题
        int32_t topic;
        SubscribeType subscribe_type;
        //数据发送周期 单位ms
        int32_t cycle;
    };

    struct VehicleState
    {
        enum Shift
        {
            R = 0,
            P = 1,
            N = 2,
            D = 3,
        };

        enum DriveMode
        {
            Standby = 0,
            Manual = 1,
            Auto = 2,
        };

        //当前时间
        int64_t date_time;
        //车辆定位经度
        double longitude;
        //车辆定位纬度
        double latitude;
        //车辆所在海拔高度
        double altitude;
        //前轮摆角度
        double steer_angle;
        //方向角（正东逆时针）
        double azimuth;
        //档位 RPND
        Shift shift;
        //油门开度（0-100）%
        double throttle;
        //刹车力度（0-100）%
        double brake;
        //行驶速度 m/s
        double speed;
        //驾驶模式 待机 手动 自动
        DriveMode drive_mode;
    };

    struct Fuel
    {
        //剩余燃料
        double residual_fuel;
        //电池电压
        double voltage;
        //电流
        double ammeter;
        //电池温度
        double temperature;
        //续航里程
        double endurance;
        bool is_charging;
        //累计里程
        double odometer;
    };

    struct  SwitchControl
    {
        enum Switch
        {
            ON = 0,
            OFF = 1,
        };

        enum SwitchType
        {
            WARNINGLAMP=0,
            MAINLAMP=1,
            SUBLAMP=2,
            FOGLAMP=3,
            WHISTLE=4,
            LEFTTURNLIGHT=5,
            RIGHTTURNLIGHT=6,
        };
        Switch	switch_;
        int64_t delayed;
        SwitchType  switch_type;
    };

    struct TaskState
    {
        int64_t start_time;
        int32_t point_count;
        int32_t current_point_index;
        //起始站点
        StationInfo start_station;
        //结束站点
        StationInfo end_station;
    };

    struct PatrolTaskState
    {
        string name; // patroltask name ongoing
        string id; // patroltask id ongoing
        int32_t task_num;
        int32_t current_task_index; // start from 0
        int32_t loop_num = 5;
        int32_t current_loop_index; // start form 0
        TaskConfig config;
        enum TaskStatus
        {
            STANDBY = 0,//有任务，待接收执行命令
            RUNNING = 1, //正在执行
            PAUSING = 2,//暂停中
            STOPPED = 3, //已停止
        };
        TaskStatus status;
    };

    struct EulerAngle
    {
        double roll;
        double pitch;
        double yaw;
    };

    struct GlobalPose
    {
        //时间戳
        int64_t date_time;

        enum GPSState{
            POWERON=0,// 上电
            INIT=1,//  初始化
            SUCCESS=2,// 成功
        };
        GPSState gps_state;
        int32_t gps_satellite_num;
        int32_t gps_week;
        int64_t gps_millisecond;
        enum PositionStatus
        {
            NONE=0,//未定位
            SINGLE=1,// 单点定位
            PSRDIFF=2,// 差分
            RTKFLOAT=3,// 浮点定位
            RTKFIX=4,// 差分定位
            UNKNOWN=5,// 未知
        };
        PositionStatus position_status;
        EulerAngle gps_angle;
        struct GPSVelocity
        {
            double north_velocity;
            double east_velocity;
            double up_velocity;
        };
        GPSVelocity gps_velocity;
        double lng;
        double lat;
        double altitude;

        EulerAngle gps_angle_dev;
        double lng_dev;
        double lat_dev;
        double gauss_x;
        double gauss_y;
    };

    struct ArriveStation
    {
        int64_t date_time;
        StationInfo station;
    };



    struct Point3D
    {
        double x;
        double y;
        double z;
    };

    struct Turn
    {
        enum Direction
        {
            LEFT=0,
            RIGHT=1,
        };
        //时间
        int64_t date_time;
        //方向
        Direction direction;
        //经度
        double longitude;
        //维度
        double latitude;
        //方向
        double azimuth;
    };

    struct VehicleStart
    {
        int64_t date_time;
        //经度
        double longitude;
        //维度
        double latitude;
        //方向
        double azimuth;
    };

    struct VehicleStop
    {
        //时间
        int64_t date_time;
        //经度
        double longitude;
        //维度
        double latitude;
        //方向
        double azimuth;
    };

    struct Obstacle
    {
        int64_t date_time;
        double longitude;
        double latitude;
        double azimuth;

        struct ObstacleInfo
        {
            enum ObstacleType
            {
                PEDESTRIAN=0,
                VEHICLE=1,
            };
            ObstacleType obstacle_type;
            double x;
            double y;
            double z;
        };
        std::vector<ObstacleInfo>  obstacles;
    };


    struct Fault
    {
        enum FaultModul
        {
            SENSOR=0,//传感器
            CHASSIS=1,//底盘
            APPLICATION=2,//应用
        };

        enum FaultType{
            GEN=0,//产生
            RELIEVE=1,//解除
        };

        enum FaultLevel
        {
            DEBUG=0,//调试
            INFO=1,//消息
            NOTICE = 2,
            WARN=3,//警告
            BUG=4,
            ERROR=5,
            FAULT=6,//故障
            FAILURE=7,//致命
        };

        int64_t date_time;
        string fault_code;
        string fault_message;
        FaultModul fault_module;
        FaultType fault_type;
        FaultLevel fault_level;
        std::vector<uint8_t> fault_data;

    };

    struct TaskControl
    {
        enum ControlType
        {
            RUN = 0,
            SUSPEND = 1,
            CANCEL = 2,
            RESUME = 3,
        };

       ControlType  control_type;
    };

    struct TaskRoute
    {
        int64_t start_time;
        int64_t arrival_time;
        struct Point
        {
            double longitude;
            double latitude;
            double azimuth;
            double grid_x;
            double grid_y;
            double gauss_x; //单位：厘米
            double gauss_y; //单位：厘米
        };
        //当前位置
        Point current_point;
        //路线点集合
        std::vector<Point> route_points;
        //起始站点
        StationInfo start_station;
        //结束站点
        StationInfo end_station;
    };

    struct GPSLocation
    {
        double longitude;
        double latitude;
        double azimuth;
        double altitude;
    };

    struct PathInfo
    {
        string path_id;
        string path_name;
        int32_t point_num;
        GPSLocation start_point;
        GPSLocation end_point;
    };

    struct PatrolInfo
    {
        string task_name;
        string task_id;

        enum TaskType
        {
            STATION_TASK = 0,
            PATH_TASK = 1,
        };
        TaskType task_type;

        struct Task
        {
            StationInfo station;
            PathInfo path;
        }task;

        struct Date
        {
            // Year of date. Must be from 1 to 9999, or 0 if specifying a date without
            // a year.
            int32_t year;
            // Month of year. Must be from 1 to 12, or 0 if specifying a year without a
            // month and day.
            int32_t month;
            // Day of month. Must be from 1 to 31 and valid for the year and month, or 0
            // if specifying a year by itself or a year and month where the day is not
            // significant.
            int32_t day;
            int32_t hour;
            int32_t min;
        };
        Date date;
        enum LoopType
        {
            CYCLE = 0,
            ROUND = 1, // go there and back
        };
        LoopType loop_type;
        int32_t loop_time;
        TaskConfig config;
    };

    struct GroupInfo
    {
        string task_name;
        string task_id;
        struct SubTask
        {
           enum TaskType
            {
                STATION_TASK = 0,
                PATH_TASK = 1,
            };
            TaskType task_type;
            struct Task
            {
                StationInfo station;
                PathInfo path;
            } task;
            TaskConfig config;
            int32_t wait_time; // wait time in min
        };
        std::vector<SubTask> sub_task;

        struct Date
        {
            // Year of date. Must be from 1 to 9999, or 0 if specifying a date without
            // a year.
            int32_t year;
            // Month of year. Must be from 1 to 12, or 0 if specifying a year without a
            // month and day.
            int32_t month;
            // Day of month. Must be from 1 to 31 and valid for the year and month, or 0
            // if specifying a year by itself or a year and month where the day is not
            // significant.
            int32_t day;
            int32_t hour;
            int32_t min;
        };
        Date date;
        int32_t loop_time;
        double last_hours;
        Date end_time;
    };

    struct SiteInfo
    {
        string id;
        string name;
        int32_t zone;
    };

    struct GetTaskObject
    {
        enum TaskObjectType
        {
            STATION_INFO = 0,
            PATH_INFO = 1,
            MAP_POINT_INFO = 2, // not used
            PATROL_INFO = 3,
            GROUP_INFO = 4,
            MAP_INFO = 5,
            SITE_INFO = 6,
        };
        TaskObjectType  object_type;
    };

    struct StationInfoResponse
    {
        std::vector<StationInfo> station;
    };

    struct PathInfoResponse
    {
        std::vector<PathInfo> path;
    };

    struct MapInfoResponse
    {
        std::vector<MapInfo> map;
    };


    struct PatrolInfoResponse
    {
        string name;
        string id;
        std::vector<PatrolInfo> patrol;
    };

    struct GroupInfoResponse
    {
        string name;
        string id;
        std::vector<GroupInfo> patrol;
    };

    struct SiteInfoResponse
    {
        std::vector<SiteInfo> site;
    };

    struct Pose2D
    {
        double x;
        double y;
        double raw;
    };


    struct NavigationPoint
    {
        Pose2D pose;
        GPSLocation GPS_location;
    };

    struct ParameterSetting
    {
        enum SpeedLevel
        {
            NORMAL = 0,
            LOW = 1,
            HIGH = 2,
        };

        struct SpeedSetting
        {
            SpeedLevel level;
            double speed;
        };

        std::vector<SpeedSetting> speed_setting;
    };


    struct LocalisationResponse
    {
        Pose2D pos;

        struct Range
        {
            double min_x;
            double min_y;
            double max_x;
            double max_y;
            int32_t grid_size;
        };
        Range range;
        std::vector<uint8_t> point_cloud;
        std::vector<uint8_t> map_body;
        int32_t quality;
    };

    struct LocalisationResult
    {
        bool result; //定位结果 true 好 false 不好
        Pose2D pose;
    };

    struct BinaryData
    {
        std::vector<uint8_t> binary;
    };

    struct RouteRecordControl
    {
        enum ControlType
        {
            START_RECORD = 0,
            STOP_RECORD = 1,
            SAVE_RECORD = 2,
            PAUSE_RECORD = 3,
            RESUME_RECORD = 4,
        };
        ControlType  control_type;
    };

    struct RouteRecordResponse
    {
        int32_t index;
        Pose2D pose;
        GPSLocation GPS_location;
    };

    struct SyncControl
    {
        enum ControlType
        {
            UPLOAD = 0,
            DOWNLOAD = 1,
            CANCEL = 2,
        };

       ControlType  control_type;

       enum DataType
        {
            //矢量图
            VECTOR = 0,
            //瓦片图
            MAP_TILE = 1,
            HDMAP = 2,
            MAP_3D = 3,
            MAP_2D = 4,
            //轨迹文件
            POSE = 5,
            //路网
            PATH_NET = 6,
        };

       std::vector<DataType> data_types;
    };

    struct SyncProgress
    {
        struct DataProgress
        {
            double total;
            double current;
        };

        std::vector<DataProgress> progress;
        double total;
        double current;
    };

    struct SyncState
    {
        enum State
        {
            ERROR = 0,
            COMPLETE = 1,
        };
        State  state;
        string error_code;
        string error_msg;
    };

    struct MapperResponse
    {
        struct MapRange
        {
            double min_x;
            double min_y;
            double max_x;
            double max_y;
            int32_t grid_size;
        };

        MapRange range;
        std::vector<uint8_t> map_body;
    };

    struct MapperTackPoint
    {
        int32_t index;
        Pose2D pose;
        GPSLocation GPS_location;
    };

    struct MapperProgress
    {
        enum ControlType
        {
            UNKOWN = 0,
            GENERATE_MAP = 6,
        };

        ControlType type;

        enum Result
        {
            FAILED = 0,
            SUCCESS = 1,
            ONGOING = 2,
        };
        Result result;
        string result_msg;
        int32_t progess; // 0~100
    };

    struct GridPoint
    {
        string id;
        string name;
        string map_name;
        Point3D position;
        EulerAngle angle;
    };

    struct GridPointList
    {
        std::vector<GridPoint> point;
    };

    struct MapperCurrent
    {
        enum MapStatus
        {
            NORMAL = 0,
            START_MAP = 1,
            STOP_MAP = 2,
            GENERATE_MAP = 3,
            PAUSE_MAP = 4,
            RESUME_MAP = 5,
        };
        MapStatus  status;
    };

    struct LocalisationCurrent
    {
        enum LocalisationStatus
        {
            NORMAL = 0,
            START_LOCAL = 1,
            STOP_LOCAL = 2,
            PAUSE_LOCAL = 3,
            RESUME_LOCAL = 4,
        };
       LocalisationStatus  status;
    };

    struct RouteRecordCurrent
    {
        enum RouteRecordStatus
        {
            NORMAL = 0,
            START = 1,
            STOP = 2,
            PAUSE = 3,
            RESUME = 4,
        };
       RouteRecordStatus  status;
    };


    struct NavigationPointEx
    {
        int mode;  //模式: 0非路线模式 1路线模式
        int zone;  //条带
        std::string path_name; //路线名称
        std::string point_name; //目的点名
        std::vector<Pose2D> dest_points;
        string point_id;
    };

    struct LocalisationControl
    {
        enum ControlType
        {
            LOCAL_BY_GPS = 0,
            LOCAL_BY_STATION = 1,
            LOCAL_MANUAL = 2,
            LOCAL_CHECK = 3,
            LOCAL_MANUAL_NO_GPS = 4,
            LOCAL_PAUSE = 5,
            LOCAL_RESUME = 6,
            LOCAL_STOP = 7,
        };
        ControlType  control_type;
        StationInfo station;
    };

    struct LocalisationControlEx
    {
        enum ControlType
        {
            LOCAL_BY_GPS = 0,
            LOCAL_BY_STATION = 1,
            LOCAL_MANUAL = 2,
            LOCAL_CHECK = 3,
            LOCAL_MANUAL_NO_GPS = 4,
            LOCAL_PAUSE = 5,
            LOCAL_RESUME = 6,
            LOCAL_STOP = 7,
        };
        ControlType  control_type;  //控制类型
        std::string point_name;  //定位点名称
        Point3D point;  //定位点坐标
    };

    struct LocalisatingControl
    {
        enum ControlType
        {
            LOCAL_PAUSE = 0,
            LOCAL_RESUME = 1,
            LOCAL_STOP = 2,
        };
        ControlType  control_type;  //控制类型
    };

    struct TaskRealStatus
    {
        TaskConfig config; // task config
        enum Status
        {
            STANDBY = 0, //有任务，待接收执行命令
            RUNNING = 1, //正在执行
            PAUSING = 2, //暂停中
            STOPPED = 3, //已停止 此时：code为0，则导航成功; code为1，则导航失败;
        };
        struct RealStatus
        {
            Status status;
            int32_t code; //错误码：0为成功，其他为错误码
            string error_msg ; //错误消息内容

        };
        RealStatus status;
    };

    struct NavigationState
    {
        //当前时间
        int64_t date_time;
        //车辆定位经度
        double longitude;
        //车辆定位纬度
        double latitude;
        //车辆所在海拔高度
        double altitude;
        //方向角（正东逆时针）
        double azimuth;
        //高斯
        double global_x;
        double global_y;
        //当前地图名
        string map_name;
        //当前站点名
        string site_name;
        int32_t zone;
        //实时速度
        double speed;
        double grid_x;
        double grid_y;
        bool position_valid;
        MapInfo map_info;
    };

    struct NavigationSetting
    {
        enum Key
        {
            SAFE_DISTACNE = 0,
            URGENCY_STOP_DISTANCE = 1,
            OBSTACLE_RADIUS = 2,
            HEADBACK_ALLOW = 3, //不需要value
            HEADBACK_NOT_ALLOW = 4, //不需要value
            OBSTACLE_AVOID = 5, //壁障，不需要value
            OBSTACLE_STOP = 6, //停障，不需要value
            CURRENT_SPEED_LOW = 7,//应用低速，不需要value
            CURRENT_SPEED_NORMAL = 8,//应用正常速度，不需要value
            CURRENT_SPEED_HIGH = 9,//应用高速，不需要value
        };

        struct Setting
        {
            //ENUM
            Key key;
            //value 可选
            double value;
        };

        std::vector<Setting> setting;
    };

    struct GeneralKey
    {
        string name;//可表示地图名、站点名等
    };

    struct MapRequest
    {
        enum ControlType
        {
            DOWNLOAD = 0,
            UPLOAD = 1,
        };
        ControlType control_type;
        string address;
        string name;
    };

    struct MapRequestEx
    {
        enum ControlType
        {
            DOWNLOAD = 0,
            UPLOAD = 1,
        };
        ControlType control_type;
        string address;
        MapInfo map_info;

        enum Entity{
            ETY_2D = 0,
            ETY_3D = 1,
            ETY_KEYFRAME = 3,
            ETY_MAPTITLE = 4,
            ETY_PATHNET = 5,
            ETY_POSE = 6,
            ETY_VECTOR = 7,
            ETY_ALL = 8,
        };
        std::vector<Entity> entities;
    };

    struct MapRename
    {
        string old_name;
        string new_name;
    };

    struct StationListUpdate
    {
        MapInfo map_info;
        std::vector<StationInfo> stationinfos;
    };

    struct VecJunction
    {
        int32_t id;				//唯一标识符
        std::vector<Point3D> vertexes;
    };

    struct VecParking
    {
        int32_t id;				//唯一标识符
        int32_t on_road;			//预留
        std::vector<Point3D> vertexes;  //停车位只存在4个点， 在[0][1][2][3]4个点中， [0]和[3]为车位的入口
    };

    struct VecZerbraline
    {
        int32_t id;				//唯一标识符
        int32_t on_road;			//预留
        std::vector<Point3D> vertexes;
    };

    struct VecObstacle
    {
        int32_t id;				//唯一标识符
        int32_t on_road;			//预留
        int32_t attr;				//预留
        int32_t order_num;		//预留
        std::vector<Point3D> vertexes;
    };

    struct VecSlopeArea
    {
        int32_t id = 1;				//唯一标识符
        std::vector<Point3D> vertexes;
    };

    struct VecLanemarking
    {
        int32_t id;			//道路id
        int32_t marking;		//线序
        int32_t attribute;	//线属性
        int32_t type;			//预留
        int32_t function;		//预留
        int32_t direct_l;		//预留
        int32_t direct_r;		//预留
        int32_t direct_s;		//预留
        int32_t direc_u;		//预留
        int32_t min_speed;	//预留
        int32_t max_speed;	//预留
        std::vector<Point3D> vertexes;
    };

    struct VecRoadline
    {
        int32_t id;					//唯一标识符
        int32_t lane;					//关联道路id
        int32_t line;					//起始车道线
        int32_t midlane_id;			//预留
        int32_t direction;			//预留
        int32_t max_speed;			//最大显示  千米/时
        int32_t marking;				//转向枚举
        int32_t follow_gps;			//跟线模式
        int32_t gps_direct;			//跟线方向
        int32_t class_;				//道路类别
        int32_t type;				//道路类型
        std::vector<Point3D> vertexes;
    };

    struct VecDynamicObstacle
    {
        int32_t id;					//唯一ID
        int32_t on_road;				//预留
        int32_t attr;					//预留
        int32_t order_num;			//预留
        int32_t dyn_flag;				//动态障碍标志， 如果为动态障碍填1
        int32_t start_time;			//障碍生效起始时间, 格林威治时间(秒)
        int32_t end_time;				//障碍生效结束时间, 格林威治时间(秒)
        std::vector<Point3D> points;	//障碍几何信息
    };



    struct VecMap
    {
        std::vector<VecJunction> junctions; 			//路口
        std::vector<VecParking> parkings; 			//停车位
        std::vector<VecZerbraline> zerbraline; 		//斑马线
        std::vector<VecObstacle> obstalce; 			//障碍（虚拟墙）
        std::vector<VecSlopeArea> slopeareas; 		//斜铺区域
        std::vector<VecLanemarking> lanemarkings; 	//道路
        std::vector<VecRoadline> roadlines; 			//路网
        std::vector<VecDynamicObstacle> dyc_obstacles; //动态障碍
    };

    struct VecMapRequest
    {
        MapInfo map_desc;	//如果地图名称为空，则获取当前地图矢量地图，反之获取指定地图下的矢量地图
        enum MapType
        {
            JUNCTION = 0, 			//路口
            PARKING= 1,		//停车位
            ZERBRALINE = 2,			//斑马线
            OBSTACLE = 3,			//障碍（虚拟墙）
            SLOPEAREA = 4,			//斜铺区域
            LANEMARKING = 5,		//道路
            ROADLINE = 6,			//路网
            DYNAMIC_OBSTACLE = 7,	//动态障碍
        };
        std::vector<MapType>  types;	//如果types为空，则默认为获取所有类型的矢量地图
    };

    struct TspNavigationRequest
    {
        int32_t zone;							//条带
        Pose2D start;  						//如果为空，则为车辆当前位置
        std::vector<StationInfo> dest_points;  	//目的点集
    };

    struct TspNavigationResponse
    {
        std::vector<StationInfo> ordered_dests;  	//目的点集
    };

    struct VersionResponse
    {
        enum VersionClass
        {
            RELEASE = 0,
            BETA  = 1,
            INSIDE = 2,
            DEBUG = 3,
        };

        enum VersionType
        {
            PROGRAM = 0,
            PARAMETER = 1,
            FIRMWARE = 2,
        };

        struct VersionInfo
        {
            VersionClass class_;
            VersionType type;
            string module_name;//program; parameter; tx2; fpga;
            string version;
            int64_t version_id;
        };

        std::vector<VersionInfo>  local_version;
        std::vector<VersionInfo>  remote_version;
    };

    struct StationLocalisation
    {
        std::string point_name;  //[reserved]定位点名称.
        Point3D point;  //定位点坐标
    };

    struct CommonState
    {
        int32_t state_code;
    };


    struct DynamicObstacleUpdate
    {
        MapInfo map_info;
        std::vector<VecDynamicObstacle> obstacles;
    };

};

#endif
