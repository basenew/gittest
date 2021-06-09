#ifndef ATRIS_DEVICE_IMSI_DATA_H_
#define ATRIS_DEVICE_IMSI_DATA_H_

#include <string>
#include <vector>

struct DangerImsiListData {
    std::string imsi;
    int danger_type;
};

struct ImsiDetectionListData {
    struct DangerImsiListData item;
    int detection_timestamp;
};

#define DANGER_TYPE_NORMAL  0
#define DANGER_TYPE_ONE     1
#define DANGER_TYPE_TWO     2
#define DANGER_TYPE_THREE   3
#define DANGER_TYPE_FOUR    4

class ImsiData
{
public:
    static void init();
    static int ParseDangerImsiTXTfile(const std::string &file_path);
    static bool IsImsiInDangerImsiList(DangerImsiListData &list_data);
    static void  UpdateImsiDetectionListDataBase(const ImsiDetectionListData &list_data);
    static int  QueryNumberofDetectionImsi(int cur_page, int num, std::vector<ImsiDetectionListData> &vector_data, int *total_page);
    static int GenerateDetectionImsiTXTFile(std::string &file_name);
private:
    static void ReinitDangerImsiDataBase();
    static void ReinitDetectionImsiDataBase();
    static void UpdateDangerImsiListDataBase(const DangerImsiListData &list_data);
    static int ParseDetectionImsiDataBase(std::vector<ImsiDetectionListData> &vector_data);
};

#endif
