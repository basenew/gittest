/*
    Udock  by xiangbin.huang
*/
#include "udockwheel_obstacle_detect.h"
#include "imemory/atris_imemory_api.h"
#include <vector>

using namespace std;
using namespace std::chrono;


void UdockWheelObstacleDetect::set_wheel_front_detect_area(float angle, float speed, quadrilateralPoint &area)
{
    // 注意：检测多边形底边为激光雷达所在水平线
    float temp = VEHICLE_WIDTH/2 + 0.08f;
    float k =1.8f, v= fabs(speed);
    
    area.A.x = 0;
    area.A.y = temp;
    area.D.x = 0;
    area.D.y = -temp;

    float deleta = atan((0.85f +0.35f)/(0.85f)*tan(angle));
    float cosdeleta = cos(deleta);
    float sindeleta = sin(deleta);
    area.B.x = multilidar_to_collisionbar + k *v *cosdeleta;
    area.B.y = temp + k *v *sindeleta;
    area.C.x = area.B.x;
    area.C.y = -temp + k *v *sindeleta;
//    log_info("[ udock ] front detect area set!");
//    log_info("[ udock ] A.x %f, A.y %f", area.A.x, area.A.y);
//    log_info("[ udock ] B.x %f, B.y %f", area.B.x, area.B.y);
//    log_info("[ udock ] C.x %f, C.y %f", area.C.x, area.C.y);
//    log_info("[ udock ] D.x %f, D.y %f", area.D.x, area.D.y);
}

void UdockWheelObstacleDetect::set_wheel_front_slanted_detect_area(float angle, float speed, quadrilateralPoint &area)
{

    float temp = VEHICLE_WIDTH/2 + 0.08f;
    float k =1.8f, v= fabs(speed);
    area.A.x = 0;
    area.A.y = temp;
    area.D.x = 0;
    area.D.y = -temp;

    float deleta = atan((0.85f +0.35f)/(0.85f)*tan(angle));
    float cosdeleta = cos(deleta);
    float sindeleta = sin(deleta);
    area.B.x = slanted_lidar_to_collisionbar + k *v *cosdeleta;
    area.B.y = temp + k *v *sindeleta;
    area.C.x = area.B.x;
    area.C.y = -temp + k *v *sindeleta;
//    log_info("[ udock ] front detect area set!");
//    log_info("[ udock ] A.x %f, A.y %f", area.A.x, area.A.y);
//    log_info("[ udock ] B.x %f, B.y %f", area.B.x, area.B.y);
//    log_info("[ udock ] C.x %f, C.y %f", area.C.x, area.C.y);
//    log_info("[ udock ] D.x %f, D.y %f", area.D.x, area.D.y);
}

void UdockWheelObstacleDetect::set_wheel_back_detect_area(float angle, float speed, quadrilateralPoint &area) //rad
{

    float temp = VEHICLE_WIDTH/2 + 0.08f;
    float k =1.8f, v= fabs(speed);

    area.A.x = 0;
    area.A.y = temp;
    area.D.x = 0;
    area.D.y = -temp;

    float deleta = -atan((0.35f)/(0.85f)*tan(angle));
    float cosdeleta = cos(deleta);
    float sindeleta = sin(deleta);

    area.B.x = k *v *cosdeleta;
    area.B.y = temp + k *v *sindeleta;
    area.C.x = area.B.x;
    area.C.y = -temp + k *v *sindeleta;

//    log_info("[ udock ] back detect area set!");
//    log_info("[ udock ] A.x %f, A.y %f", area.A.x, area.A.y);
//    log_info("[ udock ] B.x %f, B.y %f", area.B.x, area.B.y);
//    log_info("[ udock ] C.x %f, C.y %f", area.C.x, area.C.y);
//    log_info("[ udock ] D.x %f, D.y %f", area.D.x, area.D.y);
}

bool UdockWheelObstacleDetect::get_back_lidar_filter_data(void)
{
    
    if(!gs_udp->get_lidar_data(scan)){
        if (print_log_flag) log_info("[ udock ] [ obstacle avoid ]no back lidar data ,filtering data not go on...");
        return false;
    }
    ob_det.lidar_data_filter(3.0f, scan, ob_data_back);

    return true;
}

bool UdockWheelObstacleDetect::get_front_lidar_filter_data(void)
{
    static bool guassian_firmware_old = false;
    if(!gs_udp->get_lidar_data_assistant(scan_assistant))
    {
        if (print_log_flag) log_info("[ udock ] [ obstacle avoid ]no velodyne horizontal line data ,filtering data not go on...");
        return false;
    }
    //验证数据有效性
    if (!ob_det.is_lidar_data_valid(scan_assistant, 3.0f * M_PI / 5.0, -3.0f * M_PI / 5.0))
    {
        if (print_log_flag) log_info("[ udock ] [ obstacle avoid ]velodyne horizontal line data invalid,filtering data not go on...");
        return false;
    }

    // 处理水平一线数据    
    ob_det.lidar_data_filter(3.0f, scan_assistant, ob_data_assistant);

    // 处理最下面一线数据,为与老版本兼容，此处若取不到数据不返回失败
    if (!gs_udp->get_lidar_data_bottom_line(scan_bottom_line))
    {
        if (!guassian_firmware_old) 
        {
            log_info("[ udock ] [ obstacle avoid ]no velodyne bottom line data, guassian firmware in old version...");
            guassian_firmware_old = true;
        }
    }
    else
    {
        ob_det.lidar_data_filter_bottom_line(h_bottom_line, scan_bottom_line, ob_data_assistant);
    }

    return true;
}

bool UdockWheelObstacleDetect::get_front_slanted_lidar_filter_data(void)
{
    if(!gs_udp->get_lidar_data_front_slanted(scan_front_slanted)){
        if (print_log_flag) log_info("[ udock ] [ obstacle avoid ]no front slanted lidar data ,filtering data not go on...");
        return false;
    }
    ob_det.lidar_data_filter_front_slanted(h_slanted_lidar, h_cliff_detect, scan_front_slanted, ob_data_front_slanted, flag_cliff);
    return true;
}

bool UdockWheelObstacleDetect::get_front_ultra_sound_data(void)
{
    ob_data_ultrosonic.clear();
    uint16_t ultroso_measurement;
    shm::UltraSound ultroso;
    memset(ultroso.data, 0, sizeof(ultroso.data));
    shm::iMemory_read_UltraSound(&ultroso);
    
    for (int i = 2; i < 4; i++)
    {
        ultroso_measurement = ultroso.data[i];
        if (ultroso_measurement == 0xFFFF)
        {
            log_info("[ udock ] [ obstacle avoid ]ultrosonic measurement invalid...");
            return false;     
        }
        if (ultroso_measurement < 500) // 取低于500测量值
        {
            dataPoint a;
            a.x = multilidar_to_collisionbar + ultroso_measurement / 1000.0;
            a.y = 0.0;//检测到需避障，忽略y坐标
            ob_data_ultrosonic.push_back(a);
        }
    }
    return true;
}

bool UdockWheelObstacleDetect::get_back_ultra_sound_data(void)
{
    ob_data_ultrosonic.clear();
    uint16_t ultroso_measurement;
    shm::UltraSound ultroso;
    memset(ultroso.data, 0, sizeof(ultroso.data));
    shm::iMemory_read_UltraSound(&ultroso);
    
    for (int i = 6; i < 8; i++)
    {
        ultroso_measurement = ultroso.data[i];
        if (ultroso_measurement == 0xFFFF)
        {
            log_info("[ udock ] [ obstacle avoid ]ultrosonic measurement invalid...");
            return false;              
        }
        if (ultroso_measurement < 500) // 取低于500测量值
        {
            dataPoint a;
            a.x = ultroso_measurement / 1000.0;
            a.y = 0.0;//检测到需避障，忽略y坐标
            ob_data_ultrosonic.push_back(a);
        }
    }
    return true;
}

bool UdockWheelObstacleDetect::need_back_move_stop(float angle) //true -need stop
{
    set_wheel_back_detect_area(angle, 0.2f, move_back_linear_point_);
    if(!get_back_lidar_filter_data()){
        return true;
    }
    if(ob_det.lidar_data_in_eare(ob_data_back, move_back_linear_point_) >=2){
        return true;
    }
    return false;
}

bool UdockWheelObstacleDetect::need_front_move_stop(float angle)
{
    set_wheel_front_detect_area(angle, 0.2f, move_front_linear_point_);
    set_wheel_front_slanted_detect_area(angle, 0.2f, move_front_slanted_linear_point_);

    if(!get_front_lidar_filter_data()){
        return true;
    }
    if(!get_front_slanted_lidar_filter_data()){
        return true;
    }

    if(ob_det.lidar_data_in_eare(ob_data_assistant, move_front_linear_point_) >=2){
        return true;
    }
    if (ob_det.lidar_data_in_eare(ob_data_front_slanted, move_front_slanted_linear_point_) >= 2){
        return true;
    }
    return false;
}

#define ALL_SAFE                    0x00
#define SIGLE_LIDAR_DANGEROUS       0x01
#define MULTI_LIDAR_DANGEROUS       0x02
#define ULTROSONIC_DANGEROUS        0x04
#define BOTH_DANGEROUS              0x03


float UdockWheelObstacleDetect::speed_control(float linear_speed)
{
    float default_speed = 0;
    uint8_t state = ALL_SAFE;
    if(ob_det.lidar_data_in_eare(ob_data_assistant, rc_move_front_point_) >= 2){
        state |= MULTI_LIDAR_DANGEROUS;
    }
    if(ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) >= 2){
        state |= SIGLE_LIDAR_DANGEROUS;
    }
    if(ob_det.lidar_data_in_eare(ob_data_ultrosonic, rc_move_front_point_) >= 1){
        state |= ULTROSONIC_DANGEROUS;
    }

    if (state == ALL_SAFE)
        return linear_speed;
    else if (state == SIGLE_LIDAR_DANGEROUS)
    {
        forbid_move_front = true;
        log_info("[ udock ] [ obstacle avoid ] front obstacle detected(only single lidar),  speed to zero and chassis is locked!");     
    }
    else
    {
        if (state & MULTI_LIDAR_DANGEROUS)
        {
            if (print_log_flag)
                log_info("[ udock ] [ obstacle avoid ] front obstacle detected by multi lidar...");
        }
        else if (state & SIGLE_LIDAR_DANGEROUS)
        {
            if (print_log_flag)
                log_info("[ udock ] [ obstacle avoid ] front obstacle detected by single lidar...");
        }
        else if (state & ULTROSONIC_DANGEROUS)
        {
            if (print_log_flag)
                log_info("[ udock ] [ obstacle avoid ] front obstacle detected by ultrasonic sensor...");
        }
    } 
    if (print_log_flag) log_info("[ udock ] [ obstacle avoid ] speed to zero...");
    return default_speed;
}

float UdockWheelObstacleDetect::speed_control(float linear_speed, bool &get_state)
{
    float default_speed = 0;
    uint8_t state = ALL_SAFE;
    if(ob_det.lidar_data_in_eare(ob_data_assistant, rc_move_front_point_) >= 2){
        state |= MULTI_LIDAR_DANGEROUS;
    }
    if(ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) >= 2){
        state |= SIGLE_LIDAR_DANGEROUS;
    }
    if(ob_det.lidar_data_in_eare(ob_data_ultrosonic, rc_move_front_point_) >= 1){
        state |= ULTROSONIC_DANGEROUS;
    }

    if (state == ALL_SAFE)
    {
        get_state = true;
        return linear_speed;
    }
    else if (state == SIGLE_LIDAR_DANGEROUS)
    {
        log_info("[ udock ] [ obstacle avoid ] front obstacle detected(only single lidar),  speed to zero and chassis is locked!");
        forbid_move_front =true;
        get_state = true;
    }
    else
    {
        if (state & MULTI_LIDAR_DANGEROUS)
        {
            if (print_log_flag) log_info("[ udock ] [ obstacle avoid ] front obstacle detected by multi lidar");
        }
        else if (state & SIGLE_LIDAR_DANGEROUS)
        {
            if (print_log_flag) log_info("[ udock ] [ obstacle avoid ] front obstacle detected by single lidar"); 
        }           
        else if (state & ULTROSONIC_DANGEROUS)
        {
            if (print_log_flag) log_info("[ udock ] [ obstacle avoid ] front obstacle detected by ultrasonic sensor");
        }
        if (print_log_flag) log_info("[ udock ] [ obstacle avoid ] slow down ...");
        get_state = false;
    }
    return default_speed;
}

//--------------------------------------------------------------------------------
float UdockWheelObstacleDetect::get_safe_front_speed_stategy(float linear_speed, float angle)
{
    // 减少打印
    print_log_flag = false;
    count_for_print_log++;
    if (count_for_print_log >= 10)
    {
        print_log_flag = true;
        count_for_print_log = 0;
    }
    bool state;
    float speed;
    linear_speed = fabs(linear_speed);
    if(linear_speed >= SPEED_MAX){
        linear_speed = SPEED_MAX;
    }

    if(!get_front_lidar_filter_data()){
        if (print_log_flag) log_info("[ udock ] [ obstacle avoid ] filter velodyne lidar data failed, output zero speed!");
        // play_tts(TTSStrings::TTS_KEY_RC_MULTI_LIDAR_FAILURE);
        return 0;
    }

    if(!get_front_slanted_lidar_filter_data()){
        if (print_log_flag) log_info("[ udock ] [ obstacle avoid ] filter front slanted lidar data failed, output zero speed!");
        // play_tts(TTSStrings::TTS_KEY_RC_FRONT_SINGLE_LIDAR_FAILURE);
        return 0;
    }

    if(!get_front_ultra_sound_data()){
        log_info("[ udock ] [ obstacle avoid ] get front ultrasonic sensor data failed, output zero speed!");
        return 0;
    }

    if(forbid_move_front){
        return 0;
    }

    // 悬崖检测
    // if (flag_cliff)
    // {
    //     forbid_move_front = true;
    //     // play_tts(TTSStrings::TTS_KEY_RC_CHASSIS_LOCKED_BY_LIDAR);
    //     return 0;        
    // }

    // 作用减速延迟
    deceleration_affect(linear_speed);

    //  速度等级
    static vector<float> speed_level = {SPEED_MIC, SPEED_MIN, SPEED_MID, SPEED_MAX};

    // 生成进行检验的速度等级
    vector<float> speed_check_level;
    for (size_t i = 0; i < speed_level.size(); i++)
    {
        if (linear_speed > speed_level[i])
            speed_check_level.push_back(speed_level[i]);
        else
        {
            speed_check_level.push_back(linear_speed);
            break;
        }        
    }

    // 进行速度检验，输出安全速度等级
    for (auto it = speed_check_level.crbegin(); it != speed_check_level.crend()-1; ++it)
    {
        set_wheel_front_slanted_detect_area(angle, *it, rc_move_front_slanted_point_);
        set_wheel_front_detect_area(angle, *it, rc_move_front_point_);
        speed = speed_control(*it, state);
        if(state ){
            if (speed > 0.0001 && speed < linear_speed - 0.00001)
                deceleration_record(speed);
            return speed;
        }   
    }
    set_wheel_front_slanted_detect_area(angle, speed_check_level[0], rc_move_front_slanted_point_);
    set_wheel_front_detect_area(angle, speed_check_level[0], rc_move_front_point_);
    speed = speed_control(speed_check_level[0]);
    return speed;
}


float UdockWheelObstacleDetect::get_safe_front_speed(float linear_speed, float angle)
{
    float default_speed = 0;
    linear_speed = fabs(linear_speed);
    if(linear_speed >= SPEED_MAX){
        linear_speed = SPEED_MAX;
    }

    if(!get_front_lidar_filter_data()){
        log_info("[ udock ]front lidar data erro!");
        return default_speed;
    }

    if(!get_front_slanted_lidar_filter_data()){
        log_info("[ udock ]front slanted lidar data erro!");
        return default_speed;
    }
//    set_wheel_front_slanted_detect_area(angle, linear_speed, rc_move_front_slanted_point_);
//    if (ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) >= 2)
//    {
//        log_info("[ udock ] obstacle detected by front slanted lidar ........");
//        return default_speed;
//    }

    // mic speed
    if(linear_speed <=SPEED_MIC){
        set_wheel_front_slanted_detect_area(angle, linear_speed, rc_move_front_slanted_point_);
        set_wheel_front_detect_area(angle, linear_speed, rc_move_front_point_);
        if(ob_det.lidar_data_in_eare(ob_data_assistant, rc_move_front_point_) < 2
        && ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) < 2){
            return linear_speed;
        }
        return default_speed;
    }

    // min speed
    if(linear_speed <=SPEED_MIN){
        set_wheel_front_detect_area(angle, linear_speed, rc_move_front_point_);
        set_wheel_front_slanted_detect_area(angle, linear_speed, rc_move_front_slanted_point_);
        if(ob_det.lidar_data_in_eare(ob_data_assistant, rc_move_front_point_) <2
        && ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) < 2){
            return linear_speed;
        }
        set_wheel_front_detect_area(angle, SPEED_MIC, rc_move_front_point_);
        set_wheel_front_slanted_detect_area(angle, SPEED_MIC, rc_move_front_slanted_point_);
        if(ob_det.lidar_data_in_eare(ob_data_assistant, rc_move_front_point_) <2
        && ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) < 2){
            return SPEED_MIC;
        }
        return default_speed;
    }
    // mid speed
    if(linear_speed <=SPEED_MID){
        set_wheel_front_detect_area(angle, linear_speed, rc_move_front_point_);
        set_wheel_front_slanted_detect_area(angle, linear_speed, rc_move_front_slanted_point_);
        if(ob_det.lidar_data_in_eare(ob_data_assistant, rc_move_front_point_) <2
        && ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) < 2){
            return linear_speed;
        }

        set_wheel_front_detect_area(angle, SPEED_MIN, rc_move_front_point_);
        set_wheel_front_slanted_detect_area(angle, SPEED_MIN, rc_move_front_slanted_point_);
        if(ob_det.lidar_data_in_eare(ob_data_assistant, rc_move_front_point_) <2
        && ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) < 2){
            return SPEED_MIN;
        }

        set_wheel_front_detect_area(angle, SPEED_MIC, rc_move_front_point_);
        set_wheel_front_slanted_detect_area(angle, SPEED_MIC, rc_move_front_slanted_point_);
        if(ob_det.lidar_data_in_eare(ob_data_assistant, rc_move_front_point_) <2
        && ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) < 2){
            return SPEED_MIC;
        }
        return default_speed;
    }
    // max speed
    if(linear_speed <=SPEED_MAX){
        set_wheel_front_detect_area(angle, linear_speed, rc_move_front_point_);
        set_wheel_front_slanted_detect_area(angle, linear_speed, rc_move_front_slanted_point_);
        if(ob_det.lidar_data_in_eare(ob_data_assistant, rc_move_front_point_) <2
        && ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) < 2){
            return linear_speed;
        }

        set_wheel_front_detect_area(angle, SPEED_MID, rc_move_front_point_);
        set_wheel_front_slanted_detect_area(angle, SPEED_MID, rc_move_front_slanted_point_);
        if(ob_det.lidar_data_in_eare(ob_data_assistant, rc_move_front_point_) <2
        && ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) < 2){
            return SPEED_MID;
        }

        set_wheel_front_detect_area(angle, SPEED_MIN, rc_move_front_point_);
        set_wheel_front_slanted_detect_area(angle, SPEED_MIN, rc_move_front_slanted_point_);
        if(ob_det.lidar_data_in_eare(ob_data_assistant, rc_move_front_point_) <2
        && ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) < 2){
            return SPEED_MIN;
        }

        set_wheel_front_detect_area(angle, SPEED_MIC, rc_move_front_point_);
        set_wheel_front_slanted_detect_area(angle, SPEED_MIC, rc_move_front_slanted_point_);
        if(ob_det.lidar_data_in_eare(ob_data_assistant, rc_move_front_point_) <2
        && ob_det.lidar_data_in_eare(ob_data_front_slanted, rc_move_front_slanted_point_) < 2){
            return SPEED_MIC;
        }
        return default_speed;
    }
    return default_speed;
}

float UdockWheelObstacleDetect::get_safe_back_speed(float linear_speed, float angle)
{
    // 减少打印
    print_log_flag = false;
    count_for_print_log++;
    if (count_for_print_log >= 10)
    {
        print_log_flag = true;
        count_for_print_log = 0;
    }
    float default_speed = 0;
    linear_speed = fabs(linear_speed);
    if(linear_speed >= 0.4){
        linear_speed = 0.4;
    }

    if(!get_back_lidar_filter_data()){
        if (print_log_flag) log_info("[ udock ] [ obstacle avoid ] filter back lidar data failed, output zero speed!");
        // play_tts(TTSStrings::TTS_KEY_RC_REAR_LIDAR_FAILURE);
        return default_speed;
    }

    if(!get_back_ultra_sound_data()){
        if (print_log_flag) log_info("[ udock ] [ obstacle avoid ] get front ultrasonic sensor data failed, output zero speed!");
        return default_speed;
    }

    //  减速 速度等级
    static vector<float> speed_level = {SPEED_MIC, SPEED_MIN, SPEED_MID, SPEED_MAX};

    // 生成进行检验的速度等级
    vector<float> speed_check_level;
    for (size_t i = 0; i < speed_level.size(); i++)
    {
        if (linear_speed > speed_level[i])
            speed_check_level.push_back(speed_level[i]);
        else
        {
            speed_check_level.push_back(linear_speed);
            break;
        }        
    }

    // 进行速度检验，输出安全速度等级
    for (auto it = speed_check_level.crbegin(); it != speed_check_level.crend(); ++it)
    {
        set_wheel_back_detect_area(angle, *it, rc_move_back_point_);
        if(ob_det.lidar_data_in_eare(ob_data_back, rc_move_back_point_) >= 2)
        {
            if (print_log_flag) log_info("[ udock ] [ obstacle avoid ] obstacle find by back lidar, slow down...");
            continue;
        }
        else if (ob_det.lidar_data_in_eare(ob_data_ultrosonic, rc_move_back_point_) >= 1)
        {
            if (print_log_flag) log_info("[ udock ] [ obstacle avoid ] obstacle find by back ultrasonic, slow down...");
            continue;
        }
        else
        {
            forbid_move_front = false;
            return -(*it);
        }
    }
    if (print_log_flag) log_info("[ udock ] [ obstacle avoid ] speed to zero...");
    return -default_speed;
}

// void UdockWheelObstacleDetect::play_tts(int key, bool force)
// {
//     // 时间记录数组的长度暂定为10；，若增大需要更改
//     if (force)
//     {
//         msg.text = TTSStrings::text(key);
//         aisound_tts_pub_.publish(msg);
//         return;
//     }

//     static time_point<steady_clock, seconds> time_record[10];
//     auto time_now = time_point_cast<seconds>(steady_clock::now());
//     if (duration_cast<seconds>(time_now-time_record[key - TTSStrings::TTS_KEY_RC_CHASSIS_LOCKED_BY_LIDAR]).count() >= 6)//播报时间间隔6s
//     {
//         msg.text = TTSStrings::text(key);
//         aisound_tts_pub_.publish(msg);
//         time_record[key - TTSStrings::TTS_KEY_RC_CHASSIS_LOCKED_BY_LIDAR] = time_now;
//     }
// }

void UdockWheelObstacleDetect::deceleration_record(float speed)
{
    if(speed >= SPEED_MID)
    {
        duration_deceleration_ = std::chrono::milliseconds(1500);
    }
    else if(speed >= SPEED_MIN)
    {
        duration_deceleration_ = std::chrono::milliseconds(900);
    }
    else if(speed >= SPEED_MIC)
    {
        duration_deceleration_ = std::chrono::milliseconds(500);
    }
    time_point_deceleration_start_ = std::chrono::steady_clock::now();
    speed_thresh_ = speed;
    // log_info("[ udock ] [ obstacle avoid ] record speed %f", speed_thresh_);

    return ;
}

void UdockWheelObstacleDetect::deceleration_affect(float& speed)
{
    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_point_deceleration_start_) < duration_deceleration_)
    {
        // log_info("[ udock ] [ obstacle avoid ] deceleration affect %f", speed_thresh_);
        speed = min(speed_thresh_, speed);      
    }
}

/*
    END file Udock obstacle detect by xiangbin.huang
*/
