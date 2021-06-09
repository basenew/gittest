/*
    Udock algrithm by xiangbin.huang
*/

#ifndef __UDOCK_ALGRITHM_H__
#define __UDOCK_ALGRITHM_H__
#include "udock_common.h"
#include "config/config.h"
#include "platform/gs/GsData.h"
#include <json/json.h>
#include "udock_location_1.h"


typedef struct transformMatris_{
    float parameter1[3];
    float parameter2[3];

}transformMatris;



class UdockTank
{
    public:
        ~UdockTank();
        void get_pose(float *x, float *y);
        float get_lidar_wall_distance(void);
        float get_distance(void);
        float get_angle(void);
        float get_yposition(void);
        float get_xposition(void);
        float get_line_angle(void);
        void  get_dest_point(float *x, float *y);
        void  get_center_point(float *x, float *y);
        void  transform_to_dock(float inx, float iny, float *outx, float *outy);
        float line_angle_to_axisx(float line_angle);
        float line_to_line_angle(float line_angle1, float line_angle2);
        bool axis_transform(LaserRaw *data);
        void point_shadowin_line(float x0, float y0, float a, float b, float *s_x, float *s_y, float *dis);
        void least_squre_method(float *x, float *y, int size, float *k, float *b);
        bool find_wall_angle(float *angle, int point_num);
        void find_point_by_angle(void);
        void find_point_precise(void);
        void lines_angle(void);
        bool caculate_distance(int num);
        float v1_trans_v2_angle(float v1_x, float v1_y, float v2_x, float v2_y);
        // udock1.1
        bool caculate_distance(void);

        void point_distance(float point2_d);
        bool udock_caculate(LaserRaw *data,int point_num, int showd_num);
        bool is_data_valid(const float *x, const float *y, uint16_t len);
        float caculate_last_angle(void); //degree
        void reset_wall_width();

        Config *config;
        float wall_width_;
        bool wall_width_confirmed_;
        
        static UdockTank* get_instance(){
            static UdockTank udock_tank;
            return &udock_tank;
        }

    private:
        UdockTank();
        transformMatris trans_parameter_;

        // udock 1.1
        pointData point_xy_;
        UdockLocation *location_1_;
        //

        float   point_x[LIDAR_TOTAL_NUM_VELODYNE];
        float   point_y[LIDAR_TOTAL_NUM_VELODYNE];
        int     point_num_;

        float   normal_angle_;
        int     simple_point_start;
        int     simple_point_end;
        int     precise_point_start;
        int     precise_point_end;
        float   line_k_;
        float   line_b_;
        float   distance_wall_;
        float   distance_lidar_wall_;
        float   shadow_point_start_x;
        float   shadow_point_start_y;
        float   shadow_point_mid_x;
        float   shadow_point_mid_y;
        float   shadow_point_end_x;
        float   shadow_point_end_y;
        float   point2_x_;
        float   point2_y_;
        float   point2_dis_;
        float   point2_angle_;
        float   trans_angle_;
        float   trans_x_;
        float   trans_y_;
        float   line_angle_;
        float   point2_trans_x_;
        float   point2_trans_y_;
};


#endif
/*
    END file Udock algrithm by xiangbin.huang
*/
