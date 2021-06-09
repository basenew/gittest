#ifndef _DOCK_RECOGNIZER_H_
#define _DOCK_RECOGNIZER_H_

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <deque>
#include <tf/transform_listener.h>
#include <string>

namespace dock
{
    struct Point
    {
        double x;
        double y;

        Point(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
        double distance_to(const Point &the_other_point) const
        {
            return std::hypot((x - the_other_point.x), (y - the_other_point.y));
        }
    };

    struct Pose
    {
        double x;
        double y;
        double theta;

        Pose(double x_ = 0, double y_ = 0, double theta_ = 0) : x(x_), y(y_), theta(theta_) {}
    };

    class SinCosLookUpTableFor2DScans
    {
    public:
        static SinCosLookUpTableFor2DScans *get_instance()
        {
            static SinCosLookUpTableFor2DScans look_up_table;
            return &look_up_table;
        }

        std::pair<std::vector<double>, std::vector<double>> look_up_sincos(const sensor_msgs::LaserScan::ConstPtr scan_ptr)
        {
            // here do a simple hash operation.
            int key = scan_ptr->angle_min * 102.0 + scan_ptr->angle_increment * 97848.0 + scan_ptr->angle_max * 89.0;
            // std::cout << "SinCosLookUpTableFor2DScans :: look_up_sincos(): the key is " << key << std::endl;

            auto it = sin_cos_data_.find(key);
            if (it != sin_cos_data_.end())
            {
                return it->second;
            }
            else
            {
                std::cout << "SinCosLookUpTableFor2DScans :: look_up_sincos(): generate new table..." << std::endl;
                std::vector<double> sins(scan_ptr->ranges.size(), 0.0);
                std::vector<double> coss(scan_ptr->ranges.size(), 0.0);
                float angle_start = scan_ptr->angle_min;
                for (int i = 0; i < scan_ptr->ranges.size(); i++)
                {
                    sins[i] = sin(angle_start);
                    coss[i] = cos(angle_start);
                    angle_start = angle_start + scan_ptr->angle_increment;
                }

                std::pair<std::vector<double>, std::vector<double>> res = std::make_pair(sins, coss);
                sin_cos_data_.insert({key, res});
                return res;
            }
        }
        std::pair<std::vector<double>, std::vector<double>> look_up_sincos(const double angle_min, const double angle_max, const double angle_increment)
        {
            // here do a simple hash operation.
            int key = angle_min * 102.0 + angle_increment * 97848.0 + angle_max * 89.0;
            // std::cout << "SinCosLookUpTableFor2DScans :: look_up_sincos(): the key is " << key << std::endl;

            auto it = sin_cos_data_.find(key);
            if (it != sin_cos_data_.end())
            {
                return it->second;
            }
            else
            {
                std::cout << "SinCosLookUpTableFor2DScans :: look_up_sincos(): generate new table..." << std::endl;
                int length = (angle_max - angle_min) / angle_increment + 1;
                std::vector<double> sins(length, 0.0);
                std::vector<double> coss(length, 0.0);
                float angle_start = angle_min;
                for (int i = 0; i < length; i++)
                {
                    sins[i] = sin(angle_start);
                    coss[i] = cos(angle_start);
                    angle_start = angle_start + angle_increment;
                }

                std::pair<std::vector<double>, std::vector<double>> res = std::make_pair(sins, coss);
                sin_cos_data_.insert({key, res});
                return res;
            }
        }

    private:
        SinCosLookUpTableFor2DScans()
        {
        }

        std::map<int, std::pair<std::vector<double>, std::vector<double>>> sin_cos_data_;
    };

    class DockRecognizerLine
    {
    public:
        struct Cluster
        {
            int start_idx;
            int end_idx;

            Cluster(int start = 0, int end = 0) : start_idx(start), end_idx(end) {}

            int size()
            {
                return end_idx - start_idx + 1;
            }

            Point get_midpoint(std::vector<Point> &points)
            {
                int center_index = (start_idx + end_idx) / 2;
                return points[center_index];
            }
        };

        struct LineFeature
        {
            // ax + by + c = 0
            double a;
            double b;
            double c;
            double length;
            Point center;
            Point endpoint_left;
            Point endpoint_right;
        };

        struct DockFeature
        {
            Pose pose;
            Point start_point;
            Point end_point;

            DockFeature() {}
            DockFeature(Pose pose_, Point start_point_, Point end_point_) : pose(pose_), start_point(start_point_), end_point(end_point_)
            {
            }
        };

        DockRecognizerLine()
        {
            sins_coss_value_initialized_ = false;
            sin_cos_lookup_table_ptr_ = SinCosLookUpTableFor2DScans::get_instance();
        }
        // 得到激光雷达坐标系下的充电桩位姿， start_point 和 end_point 分别为充电桩轮廓首末点坐标
        bool get_dock_pose(const sensor_msgs::LaserScan::ConstPtr &scan_ptr, Pose &dock_pose, Point &start_point, Point &end_point)
        {
            ROS_INFO_STREAM("               ");
            ROS_INFO_STREAM("caculate start-----------------------------");
            scan_ptr_ = scan_ptr;
            if (sins_coss_value_initialized_ == false)
            {
                sins_coss_value_ = sin_cos_lookup_table_ptr_->look_up_sincos(scan_ptr);
                init_cluster_size_min(scan_ptr);
                if (!query_baselink_to_laser_transform(scan_ptr))
                    return false;
                sins_coss_value_initialized_ = true;
            }
            calculate_points_coordinate();
            collect_clusters();
            find_cluster_group_could_be_dock();
            return calc_dock_pose(dock_pose, start_point, end_point);
        }

        // 得到激光雷达坐标系下的充电桩位姿,不返回充电桩轮廓首末点坐标
        bool get_dock_pose(const sensor_msgs::LaserScan::ConstPtr &scan_ptr, Pose &dock_pose)
        {
            Point start_point, end_point;
            return get_dock_pose(scan_ptr, dock_pose, start_point, end_point);
        }

        // 以充电桩为坐标系，对机器人定位， Pose 为得到机器人的坐标, start_point 和 end_point 分别为充电桩首末点坐标（在雷达坐标系下）
        bool localize_in_dock_frame(const sensor_msgs::LaserScan::ConstPtr &scan_ptr, Pose &robot_pose, Point &start_point, Point &end_point)
        {
            Pose dock_pose;
            if (!get_dock_pose(scan_ptr, dock_pose, start_point, end_point))
            {
                return false;
            }
            Eigen::Matrix3f laser_pose_inv;
            laser_pose_inv << std::cos(dock_pose.theta), -std::sin(dock_pose.theta), dock_pose.x,
                std::sin(dock_pose.theta), std::cos(dock_pose.theta), dock_pose.y,
                0, 0, 1.0;

            Eigen::Matrix3f laser_pose = laser_pose_inv.inverse();

            tf::Matrix3x3 rotation_matrix(laser_pose(0, 0), laser_pose(0, 1), 0,
                                          laser_pose(1, 0), laser_pose(1, 1), 0,
                                          0, 0, 1);
            tf::Vector3 trans_vec(laser_pose(0, 2), laser_pose(1, 2), 0);
            tf::Transform laser_transform_in_dock_frame(rotation_matrix, trans_vec);
            tf::Transform robot_transform_in_dock_frame = laser_transform_in_dock_frame * base_link_to_laser_transform_.inverse();

            robot_pose.x = robot_transform_in_dock_frame.getOrigin().getX();
            robot_pose.y = robot_transform_in_dock_frame.getOrigin().getY();

            double raw, pitch, yaw;
            robot_transform_in_dock_frame.getBasis().getRPY(raw, pitch, yaw);
            robot_pose.theta = yaw;

            return true;
        }

        // 以充电桩为坐标系，对机器人定位， Pose 为得到机器人的位姿，不返回充电桩轮廓首末点坐标
        /* 
             -------
           /         \
                |-->y    
                v
                x
    */
        bool localize_in_dock_frame(const sensor_msgs::LaserScan::ConstPtr &scan_ptr, Pose &robot_pose)
        {
            Point start_point, end_point;
            return localize_in_dock_frame(scan_ptr, robot_pose, start_point, end_point);
        }

        // 使用多帧数据进行定位
        bool localize_in_dock_frame(const std::deque<sensor_msgs::LaserScan::ConstPtr> &scan_ptr_deque, Pose &robot_pose)
        {
            if (scan_ptr_deque.size() == 0)
                return false;
            robot_pose = Pose();
            Point start_point, end_point;
            Pose robot_pose_temp;
            int n = scan_ptr_deque.size();
            for (int i = 0; i < n; i++)
            {
                if (!localize_in_dock_frame(scan_ptr_deque[i], robot_pose_temp, start_point, end_point))
                {
                    return false;
                }
                robot_pose.x += robot_pose_temp.x;
                robot_pose.y += robot_pose_temp.y;
                robot_pose.theta += robot_pose_temp.theta;
            }
            robot_pose.x = robot_pose.x / n;
            robot_pose.y = robot_pose.y / n;
            robot_pose.theta = robot_pose.theta / n;
            return true;
        }

    private:
        // 初始化分割时一个cluster内的点数最小值
        void init_cluster_size_min(const sensor_msgs::LaserScan::ConstPtr &scan_ptr)
        {
            double delta_max = scan_ptr->angle_increment * dock_recognize_distance_max_;
            int point_num = dock_lineseg_length_ / delta_max;
            cluster_size_min_ = point_num * 0.8;
            ROS_INFO_STREAM("cluster_size_min_ now set to " << cluster_size_min_);
        }

        void calculate_points_coordinate()
        {
            points_.clear();
            points_index_.clear();
            for (int i = 0; i < scan_ptr_->ranges.size(); i++)
            {
                if (std::isfinite(scan_ptr_->ranges[i]))
                {
                    Point point;
                    point.x = scan_ptr_->ranges[i] * sins_coss_value_.second[i];
                    point.y = scan_ptr_->ranges[i] * sins_coss_value_.first[i];
                    points_.push_back(point);
                    points_index_.push_back(i);
                }
            }

            ROS_INFO_STREAM("original data num : " << scan_ptr_->ranges.size());
            ROS_INFO_STREAM("finite   data num : " << points_.size());
        }

        // 聚类
        void collect_clusters()
        {
            clusters_.clear();

            // 找到第一个断点
            int first_breakpoint = 0;
            for (int i = 0; i < points_.size() - 1; i++)
            {
                if (points_[i].distance_to(points_[i + 1]) > 4.0 * scan_ptr_->ranges[points_index_[i]] * scan_ptr_->angle_increment)
                {
                    first_breakpoint = i + 1;
                    break;
                }
            }

            // 将断点前的部分挪至后段
            points_.insert(points_.end(), points_.begin(), points_.begin() + first_breakpoint - 1);
            points_index_.insert(points_index_.end(), points_index_.begin(), points_index_.begin() + first_breakpoint - 1);
            // 插入一个较远点  便于后续聚类处理
            points_.emplace_back(100.0, 100.0);
            // 聚类
            for (int i = first_breakpoint; i < points_.size() - 1;)
            {
                int j = i + 1;
                for (; j < points_.size(); j++)
                {
                    if (points_[j].distance_to(points_[j - 1]) > 4.0 * scan_ptr_->ranges[points_index_[j - 1]] * scan_ptr_->angle_increment)
                    {
                        clusters_.emplace_back(i, j - 1);
                        break;
                    }
                }
                i = j;
            }
        }

        // 依据 cluster 之间的距离以及cluster的长度 找到可能是充电桩的cluster
        void find_cluster_group_could_be_dock()
        {
            cluster_groups_.clear();
            ROS_INFO_STREAM("find cluster group could be dock");
            // 首先根据cluster内首末点距离过滤掉一部分cluster
            ROS_INFO_STREAM("original cluster num " << clusters_.size());
            std::vector<Cluster> clusters_temp;
            for (auto &cluster : clusters_)
            {
                if (cluster.size() > cluster_size_min_ && cluster.size() < cluster_size_max_ &&
                    points_[cluster.start_idx].distance_to(Point()) + points_[cluster.end_idx].distance_to(Point()) < 2 * dock_recognize_distance_max_ &&
                    std::fabs(points_[cluster.start_idx].distance_to(points_[cluster.end_idx]) - dock_lineseg_length_) < 1 * dock_lineseg_length_check_thresh_)
                {
                    clusters_temp.push_back(cluster);
                }
            }

            clusters_ = clusters_temp;
            ROS_INFO_STREAM("after filtered, cluster num " << clusters_.size());
            // 获取组合数
            if (clusters_.size() < 3) 
            {
                ROS_INFO_STREAM("cluster not enough,  ");
                return;
            }
            std::vector<int> cluster_seqs(clusters_.size());
            std::iota(cluster_seqs.begin(), cluster_seqs.end(), 0);

            std::vector<int> temp_group(3);
            std::vector<std::vector<int>> groups_temp;

            combination(cluster_seqs, temp_group, groups_temp, 0, 3, 3);
            ROS_INFO_STREAM("original group num: " << groups_temp.size());

            // 选择可能组合
            for (std::vector<int> &group : groups_temp)
            {
                // ROS_INFO_STREAM("group index: " << group[0] << " " << group[1] << " " << group[2]);
                if (group[1] - group[0] < 10 && group[2] - group[1] < 10 && 
                    fabs(clusters_[group[0]].get_midpoint(points_).distance_to(clusters_[group[1]].get_midpoint(points_)) - dock_lineseg_dis_) < dock_lineseg_dis_check_thresh_ &&
                    fabs(clusters_[group[2]].get_midpoint(points_).distance_to(clusters_[group[1]].get_midpoint(points_)) - dock_lineseg_dis_) < dock_lineseg_dis_check_thresh_ &&
                    fabs(clusters_[group[0]].get_midpoint(points_).distance_to(clusters_[group[2]].get_midpoint(points_)) - 2 * dock_lineseg_dis_) < dock_lineseg_dis_check_thresh_)
                {
                    int average_cluster_point_num = (clusters_[group[0]].size() + clusters_[group[1]].size() + clusters_[group[2]].size()) / 3;

                    if (abs(average_cluster_point_num - clusters_[group[0]].size()) < average_cluster_point_num / 3 &&
                        abs(average_cluster_point_num - clusters_[group[1]].size()) < average_cluster_point_num / 3 &&
                        abs(average_cluster_point_num - clusters_[group[2]].size()) < average_cluster_point_num / 3)
                    {
                        cluster_groups_.push_back(group);
                    }
                }
            }
            ROS_INFO_STREAM("possible group num: " << cluster_groups_.size());
        }


        // 求全排列
        void combination(std::vector<int> &a, std::vector<int> &b, std::vector<std::vector<int>> &res, int l, int m, int M)
        {
            //b用于临时存储结果。len(b)==M；l为左侧游标，初始值取0；M是取出个数；m用于指示递归深度，初始值取M）
            int N = a.size();
            if (m == 0)
            {
                res.push_back(b);
                return;
            }
            for (int i = l; i < N; i++)
            {
                b[M - m] = a[i];
                combination(a, b, res, i + 1, m - 1, M);
            }
        }

        bool line_fit(const std::vector<Point> &points, LineFeature &line, double &fitness)
        {
            int size = points.size();
            if (size < 2)
            {
                return false;
            }

            double x_mean = 0;
            double y_mean = 0;
            for (int i = 0; i < size; i++)
            {
                x_mean += points[i].x;
                y_mean += points[i].y;
            }
            x_mean /= size;
            y_mean /= size;

            double Dxx = 0, Dxy = 0, Dyy = 0;

            for (int i = 0; i < size; i++)
            {
                Dxx += (points[i].x - x_mean) * (points[i].x - x_mean);
                Dxy += (points[i].x - x_mean) * (points[i].y - y_mean);
                Dyy += (points[i].y - y_mean) * (points[i].y - y_mean);
            }
            double lambda = ((Dxx + Dyy) - sqrt((Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy)) / 2.0;
            double den = sqrt(Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx));
            line.a = Dxy / den;
            line.b = (lambda - Dxx) / den;
            line.c = -line.a * x_mean - line.b * y_mean;

            double k = -line.a / line.b;
            double b = -line.c / line.b;
            double k_2_a_1 = k * k + 1.0;

            line.endpoint_left.x = (k * points.front().y - k * b + points.front().x) / k_2_a_1;
            line.endpoint_left.y = k * line.endpoint_left.x + b;

            line.endpoint_right.x = (k * points.back().y - k * b + points.back().x) / k_2_a_1;
            line.endpoint_right.y = k * line.endpoint_right.x + b;

            line.length = line.endpoint_left.distance_to(line.endpoint_right);

            line.center.x = (line.endpoint_left.x + line.endpoint_right.x) / 2.0;
            line.center.y = (line.endpoint_left.y + line.endpoint_right.y) / 2.0;

            double sqrt_a2_b2 = std::sqrt(line.a * line.a + line.b * line.b);
            double dis;
            int num_fit = 0;
            for (int i = 0; i < size; i++)
            {
                dis = (line.a * points[i].x + line.b * points[i].y + line.c) / sqrt_a2_b2;
                if (dis < line_fit_dis_thresh_) 
                {
                    num_fit ++;
                }
            }
            fitness = double(num_fit)/size;
            return true;
        }


        bool calc_dock_pose(DockFeature& dock_feature)
        {
            std::vector<Point> points_in_a_group;
            DockFeature dock_feature_temp;
            dock_feature_candidates_.clear();
            for (auto cluster_group : cluster_groups_) // 对于所有可能的聚类组
            {
                points_in_a_group.clear();
                points_in_a_group.reserve(64);
                for (int i = 0; i < 3; i++)
                {
                    for (int j = clusters_[cluster_group[i]].start_idx; j < clusters_[cluster_group[i]].end_idx; j++)
                    {
                        points_in_a_group.push_back(points_[j]);
                    }
                }

                // 把一个组内所有点拟合成一条直线
                LineFeature line;
                double fitness;
                line_fit(points_in_a_group, line, fitness);

                // 选取满足条件的作为候选
                if (fabs(line.length - dock_width) < dock_width_check_thresh_ && fitness > fitness_thresh)
                {
                    dock_feature_temp.start_point = line.endpoint_left;
                    dock_feature_temp.end_point = line.endpoint_right;
                    dock_feature_temp.pose.x = line.center.x;
                    dock_feature_temp.pose.y = line.center.y;

                    double k_v = line.b / line.a;
                    double b_v = line.center.y - k_v * line.center.x;

                    double xx1 = line.center.x + 1;
                    double yy1 = k_v * xx1 + b_v;
                    double xx2 = line.center.x - 1;
                    double yy2 = k_v * xx2 + b_v;

                    double dot_product1 = (xx1 - line.center.x) * line.center.x + (yy1 - line.center.y) * line.center.y;
                    if (dot_product1 > 0)
                    {
                        dock_feature_temp.pose.theta = std::atan2(yy1 - line.center.y, xx1 - line.center.x);
                    }
                    else
                    {
                        dock_feature_temp.pose.theta = std::atan2(yy2 - line.center.y, xx2 - line.center.x);
                    }
                    dock_feature_candidates_.push_back(dock_feature_temp);
                    ROS_INFO_STREAM("dock_width " << line.length << " fitness " << fitness);
                }         
            }
            if (dock_feature_candidates_.size() == 0)
            {
                ROS_INFO_STREAM("no dock pose candidates.");
                return false;
            }

            // 按照远近排序
            if (dock_feature_candidates_.size() > 1)
            {
                std::sort(dock_feature_candidates_.begin(), dock_feature_candidates_.end(), [](const DockFeature &feature1, const DockFeature &feature2) {
                    return (std::pow(feature1.pose.x, 2) + std::pow(feature1.pose.y, 2) < std::pow(feature2.pose.x, 2) + std::pow(feature2.pose.y, 2));
                });
                ROS_INFO_STREAM("totally " << dock_feature_candidates_.size() << " dock pose candidates, choose the nearest one.");
            }
            else
            {
                ROS_INFO_STREAM("only one dock pose candidate.");
            }

            dock_feature = dock_feature_candidates_[0];

            return true;
        }


        template <typename T>
        int sgn(T val)
        {
            return (T(0) < val) - (val < T(0));
        }


        // 计算回充桩位姿
        bool calc_dock_pose(Pose &dock_pose, Point &start_point, Point &end_point)
        {
            DockFeature dock_feature;
            if (!calc_dock_pose(dock_feature)) return false;
            dock_pose = dock_feature.pose;
            start_point = dock_feature.start_point;
            end_point = dock_feature.end_point;
            return true;
        }

        bool query_baselink_to_laser_transform(const sensor_msgs::LaserScan::ConstPtr &scan_ptr)
        {
            int query_time = 0;
            while (ros::ok())
            {
                query_time++;
                try
                {
                    tf_listener_.lookupTransform(baselink_frame_id_, scan_ptr->header.frame_id,
                                                 ros::Time(0), base_link_to_laser_transform_);
                }
                catch (tf::TransformException &ex)
                {
                    ROS_ERROR("%s", ex.what());
                    if (query_time > 5)
                    {
                        ROS_ERROR_STREAM("laser frame lookup failed");
                        return false;
                    }
                    ros::Duration(1.0).sleep();
                    continue;
                }
                break;
            }
            return true;
        }

        tf::TransformListener tf_listener_;
        std::vector<Point> points_;
        std::vector<int> points_index_;
        std::vector<Cluster> clusters_;
        std::vector<std::vector<int>> cluster_groups_;                              // 认为可能为充电桩的聚类        
        std::vector<DockFeature> dock_feature_candidates_;                          // 候选充电桩位姿，从中挑选出最近者返回
        sensor_msgs::LaserScan::ConstPtr scan_ptr_;
        SinCosLookUpTableFor2DScans *sin_cos_lookup_table_ptr_;
        bool sins_coss_value_initialized_;
        std::pair<std::vector<double>, std::vector<double>> sins_coss_value_;


        double dock_lineseg_length_ = 0.012;                                        // 充电桩实线段长度  0.093
        double dock_lineseg_length_check_thresh_ = 0.5;                            // 充电桩实线段长度检测阈值
        double dock_linegap_length_ = 0.06;                                         // 充电桩虚线段长度
        double dock_lineseg_dis_ = 0.153;     // 两个实线段之间的距离
        double dock_lineseg_dis_check_thresh_ = 0.06;                               // 两个实线段之间的距离检测阈值
        double dock_width = 0.4;    // 充电桩宽度
        double dock_width_check_thresh_ = 0.06;                                     // 充电桩宽度检测阈值
        double line_fit_dis_thresh_ = 0.01;                                         // 直线拟合的距离阈值（计算fitness用）


        double fitness_thresh = 0.80; //吻合度阈值
        int cluster_size_min_ = 5;                                                  //一个集群类点数最小阈值
        int cluster_size_max_ = 100;                                                //一个集群类点数最大阈值
        double dock_recognize_distance_max_ = 1.8;                                  //充电桩最远检测距离
        std::string baselink_frame_id_ = "bottom_laser";
        tf::StampedTransform base_link_to_laser_transform_;
    };
} // namespace dock

#endif