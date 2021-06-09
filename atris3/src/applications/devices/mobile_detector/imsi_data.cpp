#include "imsi_data.h"

#include <unistd.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include "log/log.h"

#include "database/sqliteengine.h"
#include "imemory/atris_imemory_api.h"

#define CREATE_DANGER_IMSI_LIST "CREATE TABLE IF NOT EXISTS [danger_imsi] (" \
        "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
        "[imsi] TEXT NOT NULL," \
        "[danger_type] INTEGER)"

#define CREATE_DETECTION_IMSI_LIST "CREATE TABLE IF NOT EXISTS [detection_imsi] (" \
        "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
        "[imsi] TEXT NOT NULL," \
        "[danger_type] INTEGER," \
        "[detection_timestamp] TEXT NOT NULL)" 


void ImsiData::init()
{
    SqliteEngine::execSQL(CREATE_DANGER_IMSI_LIST,  SQLITE_DANGER_IMSI_FILE);        
    SqliteEngine::execSQL(CREATE_DETECTION_IMSI_LIST,  SQLITE_DETECTION_IMSI_FILE);        
}

void ImsiData::ReinitDangerImsiDataBase()
{
    std::string cmd = "mv " + std::string(SQLITE_DANGER_IMSI_FILE) + " " + SQLITE_DANGER_IMSI_FILE + ".bak";
    system(cmd.c_str());
    SqliteEngine::execSQL(CREATE_DANGER_IMSI_LIST,  SQLITE_DANGER_IMSI_FILE);
}

void ImsiData::ReinitDetectionImsiDataBase()
{
    std::string cmd = "mv " + std::string(SQLITE_DETECTION_IMSI_FILE) + " " + std::string(SQLITE_DETECTION_IMSI_FILE) + ".bak";
    system(cmd.c_str());
    SqliteEngine::execSQL(CREATE_DETECTION_IMSI_LIST,  SQLITE_DETECTION_IMSI_FILE);
}

int ImsiData::ParseDangerImsiTXTfile(const std::string &file_path)
{
    if (access(file_path.c_str(), F_OK) != 0) {
        std::cout << file_path << " is not exsit" << std::endl;
        return -1;
    }
    
    std::ifstream _in_(file_path); 
    if (!_in_.is_open()) {
      log_error("%s Open file failed: %s", __FUNCTION__, file_path.c_str());
      return -1;
    }
    int i = 0;
    std::vector<DangerImsiListData> vec_data;
    DangerImsiListData list_data;
    std::string danger_type = "";
    std::string line;
    size_t index = 0;
    while(getline(_in_, line)){
        index = line.find('\t');
        if(index != std::string::npos){
            list_data.imsi = line.substr(0, index);
            danger_type = line.substr(index+1);
            list_data.danger_type = atoi(danger_type.c_str());
            UpdateDangerImsiListDataBase(list_data);
        }
    }

    return 0;
}

bool ImsiData::IsImsiInDangerImsiList(DangerImsiListData &list_data)
{
    bool ret = false;
    int sqlret = SQLITE_OK;
    
    char **result = nullptr;
    int row = 0;
    int column = 0;
    int danger_type = 0;

    sqlret = SqliteEngine::query("SELECT * FROM danger_imsi WHERE imsi='"+list_data.imsi+"'",
                        &result, &row, &column,  SQLITE_DANGER_IMSI_FILE);
    if(sqlret != SQLITE_OK){
        ReinitDangerImsiDataBase();
        return false;
    }
    if(row <= 0){
        return false;
    }
    danger_type = atoi(result[column+2]);
    if (danger_type == DANGER_TYPE_FOUR) {
        ret = false;
    } else {
        ret = true;
    }
    list_data.danger_type =  danger_type;
    SqliteEngine::freeQuery(result);

    return ret;
}

void ImsiData::UpdateImsiDetectionListDataBase(const ImsiDetectionListData &list_data)
{
    int sqlret = SQLITE_OK;
    int try_times = 5;
    std::stringstream danger_type_stream;
    danger_type_stream << list_data.item.danger_type;
    std::string detection_string = std::to_string(list_data.detection_timestamp);
    do
    {
        sqlret = SqliteEngine::execSQL("INSERT INTO detection_imsi(imsi, danger_type, detection_timestamp) VALUES('"
                            +list_data.item.imsi+"', "
                            +danger_type_stream.str()+",'"
                            +detection_string+"')",
                            SQLITE_DETECTION_IMSI_FILE);
        if(sqlret != SQLITE_OK){
            ReinitDetectionImsiDataBase();
        }
        if(try_times-- <= 0){
            break;
        }
    }while(sqlret != SQLITE_OK);
}

int ImsiData::QueryNumberofDetectionImsi(int cur_page, int num, std::vector<ImsiDetectionListData> &vector_data, int *total_page)
{
    if(num <= 0 || cur_page <= 0){
        return -1;
    }
    char **result = nullptr;
    int row = 0;
    int column = 0;
    int start = 0, number = 0, total = 0;
    int sqlret = SQLITE_OK;

    //SqliteEngine::query("SELECT * FROM detection_imsi", &result, &row, &column, SQLITE_DETECTION_IMSI_FILE);
    sqlret = SqliteEngine::query("SELECT * FROM detection_imsi WHERE danger_type>=1 AND danger_type<=3", 
        &result, &row, &column, SQLITE_DETECTION_IMSI_FILE);
    if(sqlret != SQLITE_OK){
        ReinitDetectionImsiDataBase();
        return -1;
    }
    if (row <= 0) {
        SqliteEngine::freeQuery(result);
        std::cout << "query db is empty" << std::endl;
        return 0;        
    }

    total = (row + num - 1)/num;
    *total_page = total;
    if(cur_page > total){
        return -1;
    }
 
    if(cur_page == total){
        start = 1;
        number = (row-1)%num + 1;
    }else{
        start = num * (total - cur_page -1) + ((row -1)%num + 1) + 1;
        number = num;
    }

    ImsiDetectionListData struct_data;
    for (int i = start; i < start + number; i++) {
        struct_data.item.danger_type =  atoi(result[i*column+2]);
        struct_data.item.imsi = result[i*column+1];
        struct_data.detection_timestamp =  atol(result[i*column+3]);
        vector_data.push_back(struct_data);
    }
    
    SqliteEngine::freeQuery(result);

    return 0;
}

void ImsiData::UpdateDangerImsiListDataBase(const DangerImsiListData &list_data)
{
    char **result = nullptr;
    int row = 0;
    int column = 0;
    std::stringstream danger_type_stream;
    std::string imsi_string = "";
    int sqlret = SQLITE_OK;
   
    imsi_string = list_data.imsi;
    sqlret = SqliteEngine::query("SELECT * FROM danger_imsi WHERE imsi='"+imsi_string+"'",
                        &result, &row, &column, SQLITE_DANGER_IMSI_FILE);
    if(sqlret != SQLITE_OK){
        ReinitDangerImsiDataBase();
    }
    if (row > 0) {
        return;
    }
    danger_type_stream.clear();
    danger_type_stream << list_data.danger_type;
    SqliteEngine::execSQL("INSERT INTO danger_imsi(imsi, danger_type) VALUES('"
                            +imsi_string+"', "
                            +danger_type_stream.str()+")",
                            SQLITE_DANGER_IMSI_FILE);
    SqliteEngine::freeQuery(result);
}

static bool DetectionImsiCompare(const ImsiDetectionListData &data1, const ImsiDetectionListData &data2)
{
    return data1.item.imsi < data2.item.imsi;
}

int ImsiData::ParseDetectionImsiDataBase(std::vector<ImsiDetectionListData> &vector_data)
{
    char **result = nullptr;
    int row = 0;
    int column = 0;
    int sqlret = SQLITE_OK;

    sqlret = SqliteEngine::query("SELECT * FROM detection_imsi", &result, &row, &column, SQLITE_DETECTION_IMSI_FILE);
    if(sqlret != SQLITE_OK){
        ReinitDetectionImsiDataBase();
        return -1;
    }
    if (row <= 0) {
        SqliteEngine::freeQuery(result);
        return -1;        
    }

    ImsiDetectionListData struct_data;

    for (int i = 1; i <= row; i++) {
        struct_data.item.imsi = result[i*column+1];
        struct_data.item.danger_type =  atoi(result[i*column+2]);
        struct_data.detection_timestamp =  atol(result[i*column+3]);
        if(struct_data.item.danger_type == DANGER_TYPE_FOUR){
            continue;
        }
        vector_data.push_back(struct_data);
    }
    
    SqliteEngine::freeQuery(result);

    // SqliteEngine::execSQL("DELETE FROM detection_imsi", SQLITE_DETECTION_IMSI_FILE);

    return 0;
}

static std::string  TimestampToDateTime(long time)
{
        struct tm *t = localtime(&time);
        char dateBuf[128] = "0";
        snprintf(dateBuf, sizeof(dateBuf), "%04d-%02d-%02d %02d:%02d:%02d", t->tm_year+1900,
                 t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);

        std::string date(dateBuf);
        return date;
}

int ImsiData::GenerateDetectionImsiTXTFile(std::string &file_name)
{
    std::vector<ImsiDetectionListData> list_data;
    int ret = ParseDetectionImsiDataBase(list_data);
    if  (ret != 0) {
        return -1;
    }

    sort(list_data.begin(), list_data.end(), DetectionImsiCompare);

    //shm::Robot shmrbt;
    //shm::iMemory_read_Robot(&shmrbt);
    //file_name = "/tmp/"+ std::string(shmrbt.robot.sn) + "_IMSI" + ".txt";
 
    std::ofstream _out_(file_name);
    unsigned int i = 0;
    for (i = 0; i < list_data.size(); i++) {
        _out_ << list_data[i].item.imsi;
        _out_ << '\t';
        _out_ << list_data[i].item.danger_type;
        _out_ << '\t';
        _out_ << TimestampToDateTime(list_data[i].detection_timestamp);
        _out_ << '\n';
    }
    _out_.close();
    
    return 0;
}
