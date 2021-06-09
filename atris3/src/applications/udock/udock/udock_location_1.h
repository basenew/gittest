/*
    Udock algrithm by xiangbin.huang
*/

#ifndef __UDOCK_LOCATION_H__
#define __UDOCK_LOCATION_H__

#include <vector>
#include <stdint.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include "platform/gs/GsData.h"
#include "udock_common.h"
#include "Eigen/Dense"
#include "log/log.h"

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

typedef struct pointData_{
    float angle_increase;
    uint16_t point_size;
    float x[2048];
    float y[2048];
}pointData;

typedef struct dataSegment_{
    uint16_t start;
    uint16_t end;
}dataSegment;

typedef struct pointOne_{
    float x;
    float y;
}pointOne;


class UdockLocation
{
    public:
        ~UdockLocation()
        {

        }
    private:
        void math_dfs(int pos, int cnt, int n, uint16_t k, int a[],bool visited[], std::vector<int> &out)
        {
            if (cnt == k) {
                std::vector<int> vc;
                for (int i = 0; i < n; i++){
                    if (visited[i]){
                        vc.push_back(a[i]-1);
                    }
                }
                if(vc.size() == k){
                    for(int j =0; j <k; j++){
                        out.push_back(vc[j]);
                    }
                }
                return;
            }
            if (pos == n) return;
            if (!visited[pos]) {
                visited[pos] = true;
                math_dfs(pos + 1, cnt + 1, n, k, a,visited, out);
                visited[pos] = false;
            }
            math_dfs(pos + 1, cnt, n, k, a, visited, out);
        }

        void math_cnk(const int n, const uint16_t k, std::vector<int> &out)
        {
            out.clear();
            int *a = new int[n];
            bool *visited = new bool[n];
            for (int i = 0; i < n; i++)
            {
                a[i] = i + 1;
                visited[i] = false;
            }
            math_dfs(0, 0, n, k, a, visited, out);
            delete[] a;
            delete[] visited;
        }

        void math_bubble_sort(float* h, size_t len)
        {
             if(len<=1) return;
             for(size_t i=0;i<len-1;++i)
                 for(size_t j=0;j<len-1-i;++j)
                     if(h[j]>h[j+1])
                        std::swap(h[j],h[j+1]);
            return;
        }

        uint16_t find_min_error_index(const std::vector<float> &erros, float &min_error)
        {
            uint16_t index = 0;
            float   min_erro = erros[0];
            for(size_t i =1; i< erros.size(); i++)
            {
                if(erros[i] < min_erro){
                    index = i;
                    min_erro = erros[i];
                }
            }
            min_error = min_erro;
            return index;
        }

        float p1_p2_distance(const pointOne p1, pointOne p2)
        {
            return sqrt( (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) );
        }

        void data_cluster(const pointData &data, std::vector<int> &out)
        {
            out.clear();
            out.push_back(0);
            for(uint16_t i=0; i<data.point_size-1; i++){
                pointOne p1, p2;
                p1.x = data.x[i]; p1.y = data.y[i];
                p2.x = data.x[i+1]; p2.y = data.y[i+1];
                float th = sqrt(p1.x * p1.x + p1.y * p1.y) * data.angle_increase;
                // 为避免在近处将边缘的离群点去掉了,加上一个最小的阈值限制
                float diff_thresh = std::max(cluster_distance_threshold *th, cluster_min_diff_thresh);
                if(p1_p2_distance(p1, p2) > diff_thresh){
                    out.push_back(i+1);
                }
            }
            if(out.back() != data.point_size-1){
                out.push_back(data.point_size-1);
            }
//            for(uint16_t i=0; i<out.size(); i++){
//                std::cout << out[i] << " ";
//            }
//            std::cout << std::endl;
        }

        void data_cluster_filter(const std::vector<int> &in, std::vector<dataSegment> &out)
        {
            out.clear();
            for(uint16_t i=0; i<in.size()-1; i++){
                if((in[i+1] - in[i]) >cluster_point_size_threshod){
                   dataSegment seg;
                   seg.start = in[i];
                   seg.end = in[i+1] -1;
                   out.push_back(seg);
                }
            }
//            for(uint16_t i=0; i<out.size(); i++){
//                std::cout << out[i].start << " "<< out[i].end << " ";;
//            }
//            std::cout << std::endl;
        }

        void data_cluster_distance_filter(const pointData &data, const std::vector<dataSegment> &in, std::vector<dataSegment> &out)
        {
            out.clear();
            for(uint16_t i=0; i<in.size(); i++){
                dataSegment seg = in[i];
                pointOne p1, p2;
                p1.x = data.x[seg.start];   p1.y = data.y[seg.start];
                p2.x = data.x[seg.end];     p2.y = data.y[seg.end];
                float d = p1_p2_distance(p1, p2);
//                std::cout << d << " ";
                if(cluster_distance_min < d && d < cluster_distance_max){
                    out.push_back(seg);
                }
            }
//            std::cout << std::endl;
        }

        bool find_dock_segment(const pointData &data, const std::vector<dataSegment> &in, std::vector<dataSegment> &out)
        {
            bool res = false;
            out.clear();
            out.resize(3);

            std::vector<pointOne> center;
            center.resize(in.size());
            for(uint16_t i=0; i<in.size(); i++){
                dataSegment seg1 = in[i];
                pointOne point;
                point.x = 0;
                point.y = 0;
                for(uint16_t j =seg1.start; j <=seg1.end; j++){
                    point.x += data.x[j];
                    point.y += data.y[j];
                }
                point.x = point.x/(seg1.end - seg1.start +1);
                point.y = point.y/(seg1.end - seg1.start +1);
                center[i] = point;
            }

            if(in.size() <3){
                log_warn("[udock] segment size <3");
//                std::cout << "segment size <3" << std::endl;
                res = false;
            }
            else if(in.size() == 3){
                float erro = three_cluser_distance_erro(center);
                log_warn("[udock] segment size =3");
//                std::cout << "size 3: erro = " << erro <<std::endl;
                if(erro < error_threshold){
                    out[0] =in[0];
                    out[1] =in[1];
                    out[2] =in[2];
                    res = true;
                }
            }
            else if(in.size() >3){
                log_warn("[udock] segment size =%d", in.size());
                std::vector<int> cnk;
                math_cnk(in.size(), 3, cnk);
                std::vector<pointOne> center_dis;
                std::vector<float> combined_error;
                for(uint16_t j=0; j< cnk.size()/3; j++){
                    pointOne p1;
                    center_dis.resize(3);
                    p1.x = center[cnk[j*3]].x;
                    p1.y = center[cnk[j*3]].y;
                    center_dis[0] = p1;
                    p1.x = center[cnk[j*3+1]].x;
                    p1.y = center[cnk[j*3+1]].y;
                    center_dis[1] = p1;
                    p1.x = center[cnk[j*3+2]].x;
                    p1.y = center[cnk[j*3+2]].y;
                    center_dis[2] = p1;
                    float erro = three_cluser_distance_erro(center_dis);
                    combined_error.push_back(erro);
//                    std::cout << "size "<< cnk.size()/3 << " " <<cnk[j*3] <<" "<<cnk[j*3+1]<<" "<<cnk[j*3+2] << " erro = " << erro <<std::endl;
                }
                float min_error;
                uint16_t min_error_index = find_min_error_index(combined_error, min_error);
                log_info("[udock] min erro :%f", min_error);
                if(min_error < error_threshold){
                    out[0] =in[cnk[min_error_index *3]];
                    out[1] =in[cnk[min_error_index *3 +1]];
                    out[2] =in[cnk[min_error_index *3 +2]];
                    res = true;
                }
            }
            return res;
        }


        float three_cluser_distance_erro(const std::vector<pointOne> &in)
        {
            float d[3];
            if(in.size() !=3){
                return 1000;
            }
            d[0] = p1_p2_distance(in[0], in[1]);
            d[1] = p1_p2_distance(in[2], in[1]);
            d[2] = p1_p2_distance(in[0], in[2]);
            math_bubble_sort(d, 3);
//            std::cout << "bubble_sort: " << d[0] <<" "<< d[1] << " " << d[2] <<std::endl;
            return fabs(d[0] - cluster_distance_p1_p2) + fabs(d[1] - cluster_distance_p1_p2) + fabs(d[2] - cluster_distance_p1_p3);
        }

public:

        static UdockLocation* get_instance()
        {
            static UdockLocation udock_location;
            return &udock_location;
        }

        bool find_dock(const pointData &data)
        {
            std::vector<int> vc_cluser;
            data_cluster(data, vc_cluser);

            std::vector<dataSegment> vc_cluser_seg;
            data_cluster_filter(vc_cluser, vc_cluser_seg);

            std::vector<dataSegment> vc_cluser_seg_dfilter;
            data_cluster_distance_filter(data, vc_cluser_seg, vc_cluser_seg_dfilter);

            std::vector<dataSegment> solution;
            if(!find_dock_segment(data, vc_cluser_seg_dfilter, solution)){
                return false;
            }
            log_info("[udock] segment[0] s:%d, e:%d", solution[0].start, solution[0].end);
            log_info("[udock] segment[1] s:%d, e:%d", solution[1].start, solution[1].end);
            log_info("[udock] segment[2] s:%d, e:%d", solution[2].start, solution[2].end);
            

            int count = 0;
            pointOne aver = {0, 0};
            for(int j=0; j<3; j++){
                aver.x = 0; aver.y = 0;
                // start+1 , end - 1 拟合直线时去掉可能的离群点（离群点通常存在于边缘）
                for(int i=solution[j].start+1; i<= solution[j].end-1; i++){
                    aver.x += data.x[i];
                    aver.y += data.y[i];
                    point_x_poly[count] = data.x[i];
                    point_y_poly[count] = data.y[i];
                    count ++;
                    if(count >=500){
                        log_warn("too .........500");
                        return false;
                    }
                }
                cluster_average[j].x = aver.x/(solution[j].end - solution[j].start +1);
                cluster_average[j].y = aver.y/(solution[j].end - solution[j].start +1);
            }
            least_squre_method(point_x_poly, point_y_poly, count, &k_, &b_);
            point_shadowin_line(cluster_average[0], k_, b_, cluster_average_shadow[0], &cluster_shadow_distance[0]);
            point_shadowin_line(cluster_average[1], k_, b_, cluster_average_shadow[1], &cluster_shadow_distance[1]);
            point_shadowin_line(cluster_average[2], k_, b_, cluster_average_shadow[2], &cluster_shadow_distance[2]);
            return true;
        }

        bool axis_transform(LaserRaw *data)
        {
            double angle_start = data->angle_min;
            lidar_data_points.point_size = data->range_size;
            if(data->range_size != LIDAR_TOTAL_NUM_VELODYNE){
                log_error("[ udock ]!!!!!!erro lidar point number! receive num[%d] right num[%d]", data->range_size ,LIDAR_TOTAL_NUM_VELODYNE);
                return false;
            }
            for(unsigned int i =0; i < data->range_size; i++){

                if(data->laser_range[i] <=10.0){
                    lidar_data_points.x[i] = data->laser_range[i] * sin(angle_start);
                    lidar_data_points.y[i] =  -(data->laser_range[i] * cos(angle_start));
                }
                else{
                    lidar_data_points.x[i] = 0;
                    lidar_data_points.y[i] = 0;
                }
                angle_start += data->angle_increment;
            }
            log_info("[ udock ] %s", __FUNCTION__);
            return true;
        }

        void least_squre_method(float *x, float *y, int size, float *k, float *b)
        {
            MatrixXd Y_MAT(size, 1);
            MatrixXd W_MAT(size, 2);
            MatrixXd WT_MAT(2, size);
            MatrixXd R_MAT(2, 1);
            MatrixXd WTW_MAT(2, 2);
            MatrixXd WTW_INV_MAT(2, 2);

            for(int i =0; i <size; i++){
                Y_MAT(i, 0) = y[i];
                W_MAT(i, 0) = x[i];
                W_MAT(i, 1) = 1;
            }
            WT_MAT = W_MAT.transpose();
            WTW_MAT = WT_MAT * W_MAT;
            WTW_INV_MAT = WTW_MAT.inverse();
            R_MAT = WTW_INV_MAT * WT_MAT * Y_MAT;
            *k = R_MAT(0, 0);
            *b = R_MAT(1, 0);
        }

        void point_shadowin_line(const pointOne &p1, const float a, const float b, pointOne &p1_shadow, float *dis)
        {
            float x0 = p1.x;
            float y0 = p1.y;

            float x, y;
            p1_shadow.x = (x0 + a * y0 - a * b )/(1 + a * a);
            p1_shadow.y = (a * x0 + a *a * y0 + b )/(1 + a * a);
            x = p1_shadow.x;
            y = p1_shadow.y;
            *dis = sqrt((x0 - x) * (x0 - x) + (y0 - y) * (y0 - y));
        }


    private:
        UdockLocation()
        {
            cluster_distance_threshold = 4.0f;
            cluster_min_diff_thresh = 0.04;
            cluster_point_size_threshod = 6;
            cluster_distance_min = 0.13 - 0.05; //0.14687
            cluster_distance_max = 0.13 + 0.05;
            cluster_distance_p1_p2 = 0.21f;
            cluster_distance_p1_p3 = cluster_distance_p1_p2 *2.0f;
            error_threshold = 0.1f;
        }
        float cluster_distance_threshold;
        float cluster_min_diff_thresh;
        uint16_t cluster_point_size_threshod;
        float cluster_distance_min;
        float cluster_distance_max;
        float cluster_distance_p1_p2;
        float cluster_distance_p1_p3;
        float error_threshold;

        pointData lidar_data_points;
        float point_x_poly[500];
        float point_y_poly[500];
    public:
        float k_, b_; //
        pointOne cluster_average[3];
        pointOne cluster_average_shadow[3];
        float cluster_shadow_distance[3];
};


#endif
/*
    END file Udock algrithm by xiangbin.huang
*/
