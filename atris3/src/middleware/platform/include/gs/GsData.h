#ifndef __PLATFORM_GS_DATA__
#define __PLATFORM_GS_DATA__
#include<iostream>
#include<string>
#include<vector>
class GsMap
{
    public:
        std::string name;
        std::string id;
        std::string create_date;
        unsigned int grid_h;
        unsigned int grid_w;
        float origin_x;
        float origin_y;
        double resolution;
};

/**
 * @brief type:璺緞绫诲瀷锛宯ame:璺緞鍚?
 */
class GsPath
{
    public:
        unsigned int type;
        std::string name;
};

typedef struct GsPos_
{
    double x;
    double y;
    double angle;
}GsPos;

typedef struct GsNamePoint_
{
    std::string name;
    GsPos pos;
}GsNamePoint;

struct GsNavPoint
{
    std::string id;
    int ass_id;
    int type;
    GsNamePoint np;
    int mode; //模式: 0非路线模式 1路线模式
    bool operator==(GsNavPoint& rhs) const {
        return (id == rhs.id &&
                ass_id == rhs.ass_id &&
                type == rhs.type);
    };
};
 
typedef struct PointInfo_
{
    std::string pointBaseId;
    std::string recognitionType;
    std::string visiblePicUrl;
    std::string thermometryPicUrl;
    std::string videoUrl;
    std::string audioUrl;
    std::string temperatureFramePoint;
    std::string deviceFramePoint;
    std::string meterType;
    std::string meterResult;
    std::string meterModel;
	std::string meterIndex;
    bool        audioStatus;
}PointInfo;

typedef struct StationInfo_
    {
        std::string station_id;
        std::string station_name;
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
        std::string building; //二级名
        //单元数
        std::string uint; //三级名
        //层数
        std::string floor; //四级名
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
}StationInfo;

struct Cmp_GsnamePt
{
    bool operator()(const GsNamePoint &p1, const GsNamePoint &p2)const
    {
        if (p1.name != p2.name){
            return p1.name < p2.name;
        }

        if (p1.pos.x != p2.pos.x){
            return p1.pos.x < p2.pos.x;
        }

        if (p1.pos.y != p2.pos.y){
            return p1.pos.y < p2.pos.y;
        }

        /*
        if(p1.pos.angle != p2.pos.angle){
            return p1.pos.angle < p2.pos.angle;
        } 
        */
        return false;
    }
};

enum GsLocateType
{
    GS_LOCATE_DIRECT,//鏈哄櫒浜轰綅濮夸笉鍙橈紝涓嶇敤杞湀瀹氫綅
    GS_LOCATE_NORMAL,//鏈哄櫒浜轰綅璧勫彉鍖栦簡锛岄渶瑕佽浆鍦堣繘琛屽畾浣?
    GS_LOCATE_CUSTMOIZED,//鑷畾涔夊畾浣嶏紝浼氳繘琛岃浆鍦?
    GS_LOCATE_DIRECT_CUSTMOIZED//鑷畾涔夊畾浣嶏紝浼氳繘琛岃浆鍦?
};

enum GsPointType
{
    GS_INIT_POINT,
    GS_CHARGE_POINT,
    GS_NAV_POINT,
    GS_RFID_POINT
};

typedef struct LaserRaw_
{
    int stamp;
    std::string frame_id;
    float angle_min, angle_max;
    float angle_increment, range_min, range_max;
    unsigned int range_size, intens_size;
    float laser_range[2048];
    float intensities[2048];
}LaserRaw;
typedef struct GridData_
{
    int x, y;
}GridData;

typedef struct LaserGrid_
{
    int stamp;
    std::string frame_id;
    int grid_width, grid_height;
    float org_x, org_y;
    float resolution;
    unsigned int grid_data_size;
    GridData grid_data[2048];
}LaserGrid;

#endif
