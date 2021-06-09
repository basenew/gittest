
#include "udock2/udock.h"
#include "ros/time.h"
#include "ros/ros.h"


using namespace dock;

void udockMagnager::active_call_back()
{
    ROS_INFO("ACTIVE");
}

void udockMagnager::done_callback(const actionlib::SimpleClientGoalState& state, const u_msgs::MoveToResultConstPtr& result)
{
    switch(state.state_)
    {
        case actionlib::SimpleClientGoalState::SUCCEEDED:
            udock_move_state_ = MOVE_FINISH;
            ROS_INFO("SUCCEEDED!!!!");
            break;
        case actionlib::SimpleClientGoalState::ABORTED:
            udock_move_state_ = MOVE_FAILED;
            ROS_INFO("ABORTED!!!!");
            break;
        default:
            break;
    }
}

void udockMagnager::feedback_callback(const u_msgs::MoveToFeedbackConstPtr& feedback)
{
    // ROS_INFO("THE NUMBER RIGHT NOM IS: %d", feedback -> complete_percent);
}

double udockMagnager::v1_trans_v2_angle(Vect v1, Vect v2)
{
    double d1 = sqrt(v1.x * v1.x + v1.y * v1.y);
    double d2 = sqrt(v2.x * v2.x + v2.y * v2.y);
    v1.x = v1.x/d1;
    v1.y = v1.y/d1;
    v2.x = v2.x/d2;
    v2.y = v2.y/d2;
    double tmp = v2.x * v2.x + v2.y * v2.y;
    double sin_theta = (v2.y * v1.x - v1.y * v2.x)/tmp;
    double cos_theta = (v1.x * v2.x + v1.y * v2.y)/tmp;
    return atan2(sin_theta, cos_theta);
}

void udockMagnager::theta_trans_vect(const Pose &p, Vect &v)
{
    v.x = cos(p.theta);
    v.y = sin(p.theta);
}

double udockMagnager::planning_angle(void)
{
    Vect temp[2];
    theta_trans_vect(robot_current_pose_, temp[0]);
    ROS_INFO("V0: %f %f" , temp[0].x, temp[0].y);
    theta_trans_vect(move_last_point_pose_, temp[1]);
    ROS_INFO("V2: %f %f" , temp[1].x, temp[1].y);
    return v1_trans_v2_angle(temp[0], temp[1]);
}

bool udockMagnager::planning_path(void)
{
    Vect temp[3];
    theta_trans_vect(robot_current_pose_, temp[0]);
    ROS_INFO("V0: %f %f" , temp[0].x, temp[0].y);

    temp[1].x = move_point_pose_.x - robot_current_pose_.x;
    temp[1].y = move_point_pose_.y - robot_current_pose_.y;
    ROS_INFO("V1: %f %f" , temp[1].x, temp[1].y);

    theta_trans_vect(move_point_pose_, temp[2]);
    ROS_INFO("V2: %f %f" , temp[2].x, temp[2].y);

    move_para_.angle1 = v1_trans_v2_angle(temp[0], temp[1]);
    move_para_.dist = sqrt(temp[1].x * temp[1].x + temp[1].y * temp[1].y);
    move_para_.angle2 = v1_trans_v2_angle(temp[1], temp[2]);

    ROS_INFO("-----planning %f, %f, %f", move_para_.angle1, move_para_.dist, move_para_.angle2);
    return true;
}

bool udockMagnager::find_charge_station(void)
{
    u_msgs::MoveToGoal goal;

    goal.type = LOCOMOTION_ROTATE;
    goal.velocity = 20* DEGREE2DRAD;
    goal.distance = 6 * M_PI;
    udock_move_state_ = MOVE_START;
    move_to_client_->sendGoal(goal, boost::bind(&udockMagnager::done_callback, this, _1, _2), 
                                    boost::bind(&udockMagnager::active_call_back, this), 
                                    boost::bind(&udockMagnager::feedback_callback, this, _1));
    uint32_t time_count = 0;
    while (1)
    {
        scan_data_mutex_.lock();
        if (dock_recongnizer_.localize_in_dock_frame(scan_, robot_current_pose_))
        {
            sleep(1);
            move_to_client_->cancelGoal();
            scan_data_mutex_.unlock();
            return true;
        }
        time_count++;
        scan_data_mutex_.unlock();
        if(time_count >= goal.distance/goal.velocity){
            move_to_client_->cancelGoal();
            break;
        }
        if(get_dock_state() == IDLE){
            ROS_INFO("udock_state == IDLE %s", __FUNCTION__);
            return false;
        }
        ROS_INFO("localize_in_dock_frame try!");
        sleep(1);
    }
    return false;
}

bool udockMagnager::move_to(const u_msgs::MoveToGoal &goal)
{
    udock_move_state_ = MOVE_START;
    move_to_client_->sendGoal(goal, boost::bind(&udockMagnager::done_callback, this, _1, _2), 
                                    boost::bind(&udockMagnager::active_call_back, this), 
                                    boost::bind(&udockMagnager::feedback_callback, this, _1));
    while(1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if(udock_move_state_ == MOVE_FAILED){
            return false;
        }
        else if(udock_move_state_ == MOVE_FINISH){
            break;
        }
        if(get_dock_state() == IDLE){
            ROS_INFO("udock_state == IDLE %s", __FUNCTION__);
            return false;
        }
    }
    return true;
}

bool udockMagnager::wait_for_charge_msg(void)
{
    int count = 0;
    while (1)
    {
        sleep(1);
        if(get_dock_state() == IDLE){
            ROS_INFO("udock_state == IDLE %s", __FUNCTION__);
            return false;
        }

        if (charge_state_ == 2)
        {
            set_dock_state(dock::SUCCESS);
            ROS_INFO("-----chargeing success!");
            break;
        }
        else{
            count++;
            if(count >10){
                if (try_count_)
                {
                    try_count_--;
                    ROS_INFO("-----docking try! %d", try_count_);
                    u_msgs::MoveToGoal goal;
                    goal.type = LOCOMOTION_MOVE;
                    goal.velocity = 0.08;
                    goal.distance = 0.3;
                    if(move_to(goal)){
                        set_dock_state(dock::FIND_SATION);
                        break;
                    }
                }
                set_dock_state(dock::IDLE);
                publish_dock_state(3);
                ROS_INFO("-----chargeing fail!");
                break;
            }
        }
    }
    return true;
}

bool udockMagnager::move_to_point(void)
{
    u_msgs::MoveToGoal goal;

    goal.type = LOCOMOTION_ROTATE;
    goal.velocity = 16* DEGREE2DRAD;
    goal.distance = move_para_.angle1;
    if(!move_to(goal)){
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    goal.type = LOCOMOTION_MOVE;
    goal.velocity = 0.05;
    goal.distance = move_para_.dist;
    if(!move_to(goal)){
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    goal.type = LOCOMOTION_ROTATE;
    goal.velocity = 16* DEGREE2DRAD;
    goal.distance = move_para_.angle2;
    if(!move_to(goal)){
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    return true;
}

bool udockMagnager::trun_back(void)
{
    u_msgs::MoveToGoal goal;
    goal.type = LOCOMOTION_ROTATE;
    goal.velocity = 8* DEGREE2DRAD;
    goal.distance = planning_angle();

    if(!move_to(goal)){
        return false;
    }

    return true;
}

void udockMagnager::laser_scan_callback(const sensor_msgs::LaserScanConstPtr& msg_ptr)
{
    std::lock_guard<std::mutex> lg(scan_data_mutex_);
    scan_ = msg_ptr;

    static int scan_ptr_index = 0;
    scan_ptr_index++;
    if (scan_ptr_index == 3)
    {
        scan_ptr_deque_.push_back(scan_);
        if (scan_ptr_deque_.size() > 4)
        {
            scan_ptr_deque_.pop_front();
        }
        scan_ptr_index = 0;
    }
}

void udockMagnager::charge_info_callback(const u_msgs::ChargeSource &msg)
{
    ROS_INFO("charge %d", msg.charge_source);
    charge_state_ = msg.charge_source;
}

void udockMagnager::average_filter(void)
{
    std::vector<float> scan_temp;
    scan_temp.clear();
    for (int i = 0; i < scan_->ranges.size(); i++)
    {
        scan_temp.push_back(0);
    }

    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j<scan_->ranges.size(); j++){
            scan_temp[j] += scan_->ranges[j];
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    laser_scan_.header = scan_->header;
    laser_scan_.angle_increment = scan_->angle_increment;
    laser_scan_.angle_max = scan_->angle_max;
    laser_scan_.angle_min = scan_->angle_min;
    laser_scan_.range_min = scan_->range_min;
    laser_scan_.angle_max = scan_->range_max;
    laser_scan_.scan_time = scan_->scan_time;

    laser_scan_.ranges.resize(scan_->ranges.size());
    for (int i = 0; i < scan_->ranges.size(); i++){
        laser_scan_.ranges[i] = scan_temp[i]/10.0f;
    }

}

int udockMagnager::is_on_target(void)
{
    if(fabs(robot_current_pose_.x - move_point_pose_.x) >0.03){
        return dock::X_ERROR;
    }
    if(fabs(robot_current_pose_.y - move_point_pose_.y) >0.02){
        return dock::Y_ERROR;
    }
    // if(fabs(robot_current_pose_.theta - move_point_pose_.theta > (2*M_PI/180))){
    //     return dock::THETA_ERROR;
    // }
    ROS_INFO("Arrive Point");
    return dock::ON_TARGET;
}

bool udockMagnager::try_localizition(void)
{
    int count = 0;
    bool rotate_try = true;
    while (1)
    {
        sleep(2);
        scan_data_mutex_.lock();
        if (dock_recongnizer_.localize_in_dock_frame(scan_ptr_deque_, robot_current_pose_))
        {
            scan_data_mutex_.unlock();
            return true;
        }
        else{
            scan_data_mutex_.unlock();
            count++;
            sleep(1);
        }

        if(count >=5 && !rotate_try){
            return false;
        }
        else if(count >=5){
            if(!find_charge_station()){
                return false;
            }
            else{
                count = 0;
                rotate_try = false;
            }
        }
    }
}
// 空闲 dock_state = 0;
// 回充过程中 dock_state = 1;
// 回充成功 dock_state = 2;
// 回充失败 dock_state = 3;
void udockMagnager::publish_dock_state(int state)
{
    std_msgs::Int32 msg;
    msg.data = state;
    charge_state_pub_.publish(msg);
    if (state != 0 && state != 1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        charge_state_pub_.publish(msg);
    }

    if(state == 2){
        feedback_to_commander(session_id_, E_ACTION_FINISHED, "finish");
    }
    else if(state == 3){
        feedback_to_commander(session_id_, E_ACTION_FAILED, "failed");
    }
}

void udockMagnager::set_dock_state(const int state)
{
    std::lock_guard<std::mutex> lck(udock_state_mutex_);
    udock_state_ = state;
}

int udockMagnager::get_dock_state(void)
{
    return udock_state_;
}

void udockMagnager::dock_thread_run(void)
{
    sleep(3);
    ROS_INFO("%s start!!!", __FUNCTION__);
    ros::Rate circle_rate(50);
    while (1)
    {
        switch (get_dock_state())
        {
            case dock::RECEIVE_CMD:{
                ROS_INFO("RECEIVE_CMD");
                set_dock_state(dock::FIND_SATION);
                publish_dock_state(1);
                break;
            }
            case dock::FIND_SATION:{
                if(!find_charge_station()){
                    ROS_INFO("can't find charge station!");
                    set_dock_state(dock::IDLE);
                    publish_dock_state(3);
                    break;
                }
                ROS_INFO("-----find charge station!");
                set_dock_state(dock::LOCALIZITION);
                break;
            }
            case dock::LOCALIZITION:
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                scan_data_mutex_.lock();
                if(ros::Time::now().toSec() -scan_->header.stamp.toSec() >2)
                {
                    scan_data_mutex_.unlock();
                    ROS_INFO("scan time too old!");
                    set_dock_state(dock::IDLE);
                    publish_dock_state(3);
                    break;
                }
                scan_data_mutex_.unlock();

                if (!try_localizition())
                {
                    ROS_INFO("localize_in_dock_frame fail!");
                    set_dock_state(dock::IDLE);
                    publish_dock_state(3);
                    break;
                }
                ROS_INFO_STREAM("localize successfully, the pose is  [" << robot_current_pose_.x << ", " \
                    << robot_current_pose_.y << ", " << robot_current_pose_.theta << "] .");

                switch(is_on_target()){
                    case dock::ON_TARGET:{
                        ROS_INFO("-----on target!");
                        set_dock_state(dock::TURN_BACK);
                        break;
                    }
                    case dock::X_ERROR:
                    case dock::Y_ERROR:
                        set_dock_state(dock::PLANNING);
                        break;
                    default:
                        break;
                }
                break;
            }
            case dock::PLANNING:
                if(!planning_path()){
                    set_dock_state(dock::IDLE);
                    publish_dock_state(3);
                    break;
                }
                ROS_INFO("-----planning!");
                set_dock_state(dock::MOVE_TO_POINT);
                break;
            case dock::MOVE_TO_POINT:
                if(!move_to_point()){
                    ROS_INFO("move fail!");
                    set_dock_state(dock::IDLE);
                    publish_dock_state(3);
                    break;
                }
                ROS_INFO("-----move point success!");
                set_dock_state(dock::LOCALIZITION);
                break;
            case dock::TURN_BACK:
                if(!trun_back()){
                    ROS_INFO("turn fail!");
                    set_dock_state(dock::IDLE);
                    publish_dock_state(3);
                    break;
                }
                ROS_INFO("-----turn back success!");
                set_dock_state(dock::DOCKING);
                break;
            case dock::DOCKING:
            {
                ROS_INFO("-----docking now!");
                u_msgs::MoveToGoal goal;
                goal.type = LOCOMOTION_MOVE;
                goal.velocity = 0.05;
                goal.distance = -0.235;
                if(move_to(goal)){
                    set_dock_state(dock::CAHRGING);
                    ROS_INFO("-----docking success!");
                }
                else{
                    set_dock_state(dock::IDLE);
                    publish_dock_state(3);
                    ROS_INFO("docking fail!");
                }
                break;
            }
            case dock::CAHRGING:
            {
                sleep(2);
                wait_for_charge_msg();
                break;
            }
            case dock::SUCCESS:
                ROS_INFO("-----finish!");
                publish_dock_state(2);
                set_dock_state(dock::IDLE);
                break;
            case dock::IDLE:
                publish_dock_state(0);
                break;
            default:
                break;
        }
        udock_state_mutex_.unlock();
        circle_rate.sleep();
    }
}

void udockMagnager::cmd_receive_thread_run(void)
{
    const char *dir_path    = "/home/cruiser/testudock";
    ROS_INFO("%s start!!!", __FUNCTION__);
    while (1)
    {
        std::unique_lock<std::mutex> ulck(udock_state_mutex_);
        if(access(dir_path, F_OK) == 0)
        {
            if(get_dock_state() == dock::IDLE){
                set_dock_state(dock::RECEIVE_CMD);
                try_count_ = 0;
            }
            remove(dir_path);
        }
        ulck.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

}

void udockMagnager::feedback_to_commander(int sessionid, int status, std::string msg)
{
    std::lock_guard<std::mutex> lck(feedback_mutex_);
    Json::FastWriter jwriter;
    Json::Value root;
    Json::Value content;

    root["title"] =Json::Value("notify_feedback");
    content["timestamp"] =Json::Value(ros::Time::now().toNSec()/1000000);
    content["id"] =Json::Value(sessionid);
    content["code"] =Json::Value(status);
    content["msg"] =Json::Value(msg);
    root["content"] =Json::Value(content);

    std_msgs::String pub;
    pub.data = jwriter.write(root);
    pub_notify_msg_.publish(pub);
    ROS_INFO_STREAM("[Response cmd]"  <<pub.data <<" " <<__FUNCTION__);
}

bool udockMagnager::check_command_title(const std::string &title)
{
  if(title == "request_dock_on"){
    return true;
  }

  if(title == "request_cancel_dock"){
    return true;
  }

  return false;
}

void udockMagnager::command_callback(const std_msgs::String &msgs)
{
  Json::Reader reader;
  Json::Value root;
  
  try {
    if(!reader.parse(msgs.data, root)){
      ROS_INFO_STREAM("json file parse fail");
      return;
    }
    std::string title = root["title"].isNull() ? "" : root["title"].asString();
    if(!check_command_title(title)){
        return;
    }
    ROS_INFO_STREAM("[udock command]: " << msgs.data);
    Json::Value content = root["content"];
    session_id_ = content["id"].isNull() ? 0 : root["id"].asInt64();

    if(title == "request_dock_on"){
        if(get_dock_state() == dock::IDLE){
            set_dock_state(dock::RECEIVE_CMD);
            try_count_ = 0;
            feedback_to_commander(session_id_, E_ACTION_START, "start");
        }
        else{
            ROS_INFO_STREAM("dock busy!");
            feedback_to_commander(session_id_, E_ACTION_FAILED, "dock_busy");
        }
    }
    else if(title == "request_cancel_dock"){
        set_dock_state(dock::IDLE);
        ROS_INFO("cancel dock!");
        feedback_to_commander(session_id_, E_ACTION_FINISHED, "finish");

        u_msgs::MoveToGoal goal;
        goal.type = LOCOMOTION_STOP;
        goal.velocity = 0;
        goal.distance = 0;
        udock_move_state_ = MOVE_START;
        move_to_client_->sendGoal(goal, boost::bind(&udockMagnager::done_callback, this, _1, _2), 
                                        boost::bind(&udockMagnager::active_call_back, this), 
                                        boost::bind(&udockMagnager::feedback_callback, this, _1));
    }
  }
  catch (const std::exception& e){
    ROS_ERROR_STREAM(__func__<<" Unhandled exception:" << e.what());
    return;
  }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "udock_manager");
    ros::NodeHandle nh;

    udockMagnager udock_manager(nh);

    std::thread dock_thread(&udockMagnager::dock_thread_run, &udock_manager);
    std::thread cmd_receive_thread(&udockMagnager::cmd_receive_thread_run, &udock_manager);

    dock_thread.detach();
    cmd_receive_thread.detach();
    ros::spin();

    return 0;
}



