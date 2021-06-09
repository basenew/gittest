/*
    Udock algrithm by xiangbin.huang
*/

#ifndef __UDOCK_OBSTACLE_DETECT_H__
#define __UDOCK_OBSTACLE_DETECT_H__

#include <vector>
#include <string.h>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include "log/log.h"
#include "udock_common.h"
#include "platform/gs/GsData.h"

using namespace std;
using namespace Eigen;

typedef struct dataPoint_{
    float x;
    float y;
}dataPoint;

#define     SPEED_MIC               0.2f
#define     SPEED_MIC_DURATION      400 //ms
#define     SPEED_MIN   0.4f
#define     SPEED_MIN_DURATION      1000
#define     SPEED_MID   0.7f
#define     SPEED_MID_DURATION      2500
#define     SPEED_MAX   1.0f

typedef struct quadrilateralPoint_{
    dataPoint A;
    dataPoint B;
    dataPoint C;
    dataPoint D;

}quadrilateralPoint;

class UdockObstacleDetect
{
    public:
        UdockObstacleDetect(){

        }
        ~UdockObstacleDetect(){

        }

        //confirm validity of lidar data
         bool is_lidar_data_valid(const LaserRaw &scan, float left_angle, float right_angle)
         {
             float angle_start = scan.angle_min;
             int num_total = 0; 
             int num_inf = 0;
             unsigned int i = 0;
             for (i = 0; i < scan.range_size; i++)
             {
                 if (angle_start < right_angle || angle_start > left_angle)
                 {
                     angle_start += scan.angle_increment;
                     continue;
                 }
                 if(isinf(scan.laser_range[i])) num_inf++;
                 num_total++; 
                 angle_start += scan.angle_increment;
             }
             float inf_rate = float(num_inf)/num_total;
             // log_info("[ udock ] total %d", i);
             // log_info("[ udock ] lidar  data  %d  %d",  num_inf, num_total);
             if (inf_rate > 0.98)
             {
                 log_info("[ udock ] lidar  data invalid (%d %% inf data)...", int(inf_rate*100));
                 return false;
             }
             return true;
         }
 


        //scan data filter
        bool lidar_data_filter(float dis, const LaserRaw &scan, vector<dataPoint> &data_in_range)
        {
            boost::lock_guard<boost::mutex> lock(lidar_filter_lock);
            float angle_start = scan.angle_min;
            data_in_range.clear();

            for(unsigned int i =0; i <scan.range_size; i++){
                if((0.02 <=scan.laser_range[i]) && (scan.laser_range[i] <= dis)){
                    dataPoint tep;
                    tep.x = scan.laser_range[i] * cos(angle_start);
                    tep.y = scan.laser_range[i] * sin(angle_start);
                    data_in_range.push_back(tep);
                }
                angle_start += scan.angle_increment;
            }
            return true;
        }

        bool lidar_data_filter_bottom_line(float h, const LaserRaw &scan, vector<dataPoint> &data_in_range)
        {
            //注意，不要clear data_in_range, 目的是叠加在水平一线数据上
            float angle_start = scan.angle_min;
            static float sinm15d = sin(-15.0/180.0*M_PI);
            static float sin8d = sin(8.0/180.0*M_PI);
            static float cos8d = cos(8.0/180.0*M_PI);
            static float sin_offset_angle[2000]; //激光光线与x轴夹角
            static float cos_offset_angle[2000]; 
            static bool sin_and_cos_of_angle_init = false;
            if (sin_and_cos_of_angle_init == false)
            {
                float cos15d = cos(15.0/180.0*M_PI);
                log_info("[ udock ] initial sin and cos of angle................");
                for (unsigned int i = 0; i < scan.range_size; i++)
                {                    
                    sin_offset_angle[i] = sin(angle_start) * cos15d;
                    cos_offset_angle[i] = cos(angle_start) * cos15d;
                    angle_start += scan.angle_increment;
                }
                sin_and_cos_of_angle_init = true;
                angle_start = scan.angle_min;
            }

            Vector3f v3temp1(0.0, 0.0, 0.0), v3temp2(0.0, 0.0, 0.0); 
            dataPoint tep;
            for (unsigned int i = 0, j = 0; i < scan.range_size; i++)
            {
                if (angle_start < -1.0f * M_PI / 3.0f || angle_start > 1.0f * M_PI / 3.0f)
                {
                    angle_start += scan.angle_increment;
                    continue;
                }
                if ((0.5 < scan.laser_range[i]) && (scan.laser_range[i] < 3))
                {
                    v3temp2 = scan.laser_range[i]*Vector3f(cos_offset_angle[i], sin_offset_angle[i], sinm15d);
                    v3temp1 = v3temp2;
                    v3temp2(0) = cos8d * v3temp1(0) + sin8d * v3temp1(2);
                    v3temp2(2) = -sin8d * v3temp1(0) + cos8d * v3temp1(2);
                    if (v3temp2(2) > -h)
                    {
                        tep.x = v3temp2(0);
                        tep.y = v3temp2(1);
                        data_in_range.push_back(tep);
                    }
                    //j++;
                }
                angle_start += scan.angle_increment;
            }
            return true;
        }

        bool lidar_data_filter_front_slanted(float h_ob, float h_cliff, const LaserRaw &scan, vector<dataPoint> &data_in_range, bool& flag_cliff)
        {
            //log_info("[ udock ] into lidar data filter of front slanted lidar");
            float dip_angle_0 = 0;
            flag_cliff = false;
#ifdef _CHASSIS_MARSHELL_
#ifdef _POLICE_
            dip_angle_0 = 36.0f / 180.0f * M_PI;
#else
            dip_angle_0 = 35.0f / 180.0f * M_PI;
#endif
#else
            dip_angle_0 = 36.0f / 180.0f * M_PI;
#endif
            static float sin_dip_angle_0 = sin(dip_angle_0);
            static float cos_dip_angle_0 = cos(dip_angle_0);


            boost::lock_guard<boost::mutex> lock(lidar_filter_lock_front_slanted);
            float angle_start = scan.angle_min;
            data_in_range.clear();
            data_in_range.reserve(512);

            static float sin_dip_angle[811]; //激光光线与水平面夹角
            //static float cos_dip_angle[811];
            static float sin_offset_angle[811]; //激光光线与x轴夹角
            static float cos_offset_angle[811];
            static float cos_dip_angle_0_multiply_cos_offset[811];
            static bool sin_and_cos_of_angle_init = false;
            if (sin_and_cos_of_angle_init == false)
            {
                log_info("[ udock ] initial sin and cos of angle................");
                for (unsigned int i = 0; i < scan.range_size; i++)
                {
                    sin_offset_angle[i] = sin(angle_start);
                    cos_offset_angle[i] = cos(angle_start);
                    sin_dip_angle[i] = sin_dip_angle_0 * cos_offset_angle[i];
                    //cos_dip_angle[i] = sqrt(1.0 - sin_dip_angle[i] * sin_dip_angle[i]);
                    cos_dip_angle_0_multiply_cos_offset[i] = cos_dip_angle_0 * cos_offset_angle[i];
                    angle_start += scan.angle_increment;
                }
                sin_and_cos_of_angle_init = true;
                angle_start = scan.angle_min;
            }

            for (unsigned int i = 0; i < scan.range_size; i++)
            {
                if (angle_start < -1.0f * M_PI / 3.0f || angle_start > 1.0f * M_PI / 3.0f)
                {
                    angle_start += scan.angle_increment;
                    continue;
                }
                if ((0.05 <= scan.laser_range[i]) && (scan.laser_range[i] * sin_dip_angle[i] <= h_ob))
                {
                    dataPoint tep;
                    tep.x = scan.laser_range[i] * cos_dip_angle_0_multiply_cos_offset[i];
                    tep.y = scan.laser_range[i] * sin_offset_angle[i];
                    
                    //tep.x = scan.laser_range[i] * cos_dip_angle[i] * cos_offset_angle[i];
                    //tep.y = scan.laser_range[i] * cos_dip_angle[i] * sin_offset_angle[i];

                    data_in_range.push_back(tep);
                    //log_info("[ udock ]collect dot..........: %d ", i);
                }
                // else if (scan.laser_range[i] * sin_dip_angle[i] > h_cliff)
                // {
                //     float y = scan.laser_range[i] * sin_offset_angle[i];
                //     //float y = scan.laser_range[i] * cos_dip_angle[i] * sin_offset_angle[i];
                //     if (y > -VEHICLE_WIDTH / 2.0f && y < VEHICLE_WIDTH / 2.0)
                //     {
                //         flag_cliff = true;
                //         log_info("[ udock ] cliff detected height %f ", scan.laser_range[i] * sin_dip_angle[i]);

                //         // 出现悬崖时发出打印原始数据
                //         log_info("[ udock ] raw data");
                //         for (unsigned int j = 0; j < scan.range_size; j++)
                //         {
                //             if ((scan.angle_min + j * scan.angle_increment) < -1.0f * M_PI / 3.0f || (scan.angle_min + j * scan.angle_increment) > 1.0f * M_PI / 3.0f)
                //             {
                //                 continue;
                //             }
                //             log_info("[ udock ] %f", scan.laser_range[j]);
                //         }
                //         return true;
                //     }
                // }
                angle_start += scan.angle_increment;
            }
            return true;
        }

        //eare detect
        int lidar_data_in_eare(const vector<dataPoint> &data_in_range, const quadrilateralPoint &eare)
        {
             boost::lock_guard<boost::mutex> lock(lidar_eare_lock);
             dataPoint A = eare.A;
             dataPoint B = eare.B;
             dataPoint C = eare.C;
             dataPoint D = eare.D;
             int count = 0;
             if(data_in_range.size() <1){
                 return count;
             }

             for(unsigned int i=0; i<data_in_range.size(); i++){
                  float x = data_in_range[i].x;
                  float y = data_in_range[i].y;

                  float a = (B.x - A.x)*(y - A.y) - (B.y - A.y)*(x - A.x);
                  float b = (C.x - B.x)*(y - B.y) - (C.y - B.y)*(x - B.x);
                  float c = (D.x - C.x)*(y - C.y) - (D.y - C.y)*(x - C.x);
                  float d = (A.x - D.x)*(y - D.y) - (A.y - D.y)*(x - D.x);
                  if((a > 0 && b > 0 && c > 0 && d > 0) || (a < 0 && b < 0 && c < 0 && d < 0)) {
                      count++;
                  }
             }
             return count;
        }


    private:

        boost::mutex        lidar_filter_lock;
        boost::mutex        lidar_filter_lock_front_slanted;
        boost::mutex        lidar_eare_lock;

};


#endif
/*
    END file Udock obstacle detect by xiangbin.huang
*/
