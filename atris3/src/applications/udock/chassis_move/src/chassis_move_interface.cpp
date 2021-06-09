/*

*/
#include "chassis_move_interface.h"

/*-----------------------------------------------------------------------------------------------------------*/
void ChassisMoveInterface::feedback_to_commander(int sessionid, int status, std::string msg)
{
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

void ChassisMoveInterface::feedback_to_commander_state(int sessionid, int status)
{
	Json::FastWriter jwriter;
  Json::Value root;
	Json::Value content;

  root["title"] =Json::Value("response_get_locomotion");
  content["timestamp"] =Json::Value(ros::Time::now().toNSec()/1000000);
  content["id"] =Json::Value(sessionid);
  content["status"] =Json::Value(status); //0:idle 1:locomotioning -1 failed
  root["content"] =Json::Value(content);

  std_msgs::String pub;
	pub.data = jwriter.write(root);
	pub_notify_msg_.publish(pub);
  ROS_INFO_STREAM("[Response cmd]" <<pub.data <<" " <<__FUNCTION__);
}

void ChassisMoveInterface::feedback_odom(int sessionid, float x, float y, float theta)
{
	Json::FastWriter jwriter;
  Json::Value root;
	Json::Value content;

  root["title"]     = Json::Value("response_get_odm_info");
  root["sessionid"] = Json::Value(sessionid);
  content["x"]    = Json::Value(x);
  content["y"]    = Json::Value(y);
  content["theta"]    = Json::Value(theta);
  root["content"]   = Json::Value(content);

  std_msgs::String pub;
	pub.data = jwriter.write(root);
	pub_notify_msg_.publish(pub);
  ROS_INFO_STREAM("[Response odom cmd]"  << pub.data );
}

bool check_command_title(const std::string &title)
{
  if(title == "request_locomotion"){
    return true;
  }

  if(title == "request_get_locomotion"){
    return true;
  }

  return false;
}

bool ChassisMoveInterface::move_to(const u_msgs::MoveToGoal &goal)
{
    std::lock_guard<std::mutex> lck(move_state_mutex_);
    move_state_ = MOVE_START;
    move_to_client_->sendGoal(goal, boost::bind(&ChassisMoveInterface::done_callback, this, _1, _2), 
                                    boost::bind(&ChassisMoveInterface::active_call_back, this), 
                                    boost::bind(&ChassisMoveInterface::feedback_callback, this, _1));
    feedback_to_commander(sessionid_, E_ACTION_START , "start");
    return true;
}

void ChassisMoveInterface::emergency_callback(const u_msgs::Diagnostics& msgIn)
{
    if(msgIn.key == EMERGENCY_STOP_ID){
        if(msgIn.level == 2){
            chassis_emstop_state_ = true;
            ROS_INFO("%s emergency true",__FUNCTION__);
        }
        else{
            chassis_emstop_state_ = false;
            ROS_INFO("%s emergency false",__FUNCTION__);
        }
    }
}

void ChassisMoveInterface::geomagn_callback(const u_msgs::Geomagnetism &msgIn)
{
    if(msgIn.trigger == true)
    {
        magn_trigger_state_ = true;
    }
}

void ChassisMoveInterface::command_callback(const std_msgs::String &msgs)
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
    ROS_INFO_STREAM("[chassis_move command]: " << msgs.data);

    Json::Value content = root["content"];
    sessionid_old_ = sessionid_;
    sessionid_ = content["id"].asInt64();
    //获取运动状态
    if(title == "request_get_locomotion"){
      if(move_state_ == MOVE_START) feedback_to_commander_state(sessionid_, 1);
      else feedback_to_commander_state(sessionid_, 0);
      return;
    }

    if(content["action"].isNull() || content["displacement"].isNull() || content["speed"].isNull()){
      ROS_INFO_STREAM("request_erro param");
      return;
    }
    //执行运动指令
    int action = content["action"].asInt();
    float displacement = content["displacement"].asFloat();
    float speed = content["speed"].asFloat();
    
    if(chassis_emstop_state_ || magn_trigger_state_){
      if(action == LOCOMOTION_MOVE || action == LOCOMOTION_ROTATE){
        feedback_to_commander(sessionid_, E_ACTION_EMERGENCY_STOPPED , FAIL_REASON_EMSTOP);
        return;
      }
    }

    if(move_state_ == MOVE_START){
      feedback_to_commander(sessionid_old_, E_ACTION_CANCEL , FAIL_CANCEL);
    }

    switch(action){
      case LOCOMOTION_STOP:{
        u_msgs::MoveToGoal goal;
        goal.type = LOCOMOTION_STOP;
        goal.velocity = 0;
        goal.distance = 0;
        move_to(goal);
      }
      break;
      case LOCOMOTION_MOVE:{
        u_msgs::MoveToGoal goal;
        goal.type = LOCOMOTION_MOVE;
        goal.velocity = speed;
        goal.distance = displacement;
        move_to(goal);
      }
      break;
      case LOCOMOTION_ROTATE:{
        u_msgs::MoveToGoal goal;
        goal.type = LOCOMOTION_ROTATE;
        goal.velocity = speed;
        goal.distance = displacement;
        move_to(goal);
      }
      break;
      default:
        ROS_INFO_STREAM("request_erro action");
        break;
    }
  }
  catch (const std::exception& e){
    ROS_ERROR_STREAM(__func__<<" Unhandled exception:" << e.what());
    return;
  }
}

void ChassisMoveInterface::active_call_back()
{
    ROS_INFO("start action");
}

void ChassisMoveInterface::done_callback(const actionlib::SimpleClientGoalState& state, const u_msgs::MoveToResultConstPtr& result)
{
    std::lock_guard<std::mutex> lck(move_state_mutex_);
    switch(state.state_)
    {
        case actionlib::SimpleClientGoalState::SUCCEEDED:
            move_state_ = MOVE_FINISH;
            feedback_to_commander(sessionid_, E_ACTION_FINISHED , "finished");
            ROS_INFO("succeeded!");
            break;
        case actionlib::SimpleClientGoalState::ABORTED:
            move_state_ = MOVE_FAILED;
            if(result->fail_reason == FAIL_REASON_OBSTACLE){
              feedback_to_commander(sessionid_, E_ACTION_BE_IMPEDED , FAIL_REASON_OBSTACLE);
            }
            else if(result->fail_reason == FAIL_REASON_EMSTOP){
              feedback_to_commander(sessionid_, E_ACTION_EMERGENCY_STOPPED , FAIL_REASON_EMSTOP);
            }
            else if(result->fail_reason == FAIL_CANCEL){
              feedback_to_commander(sessionid_, E_ACTION_CANCEL , FAIL_CANCEL);
            }
            else{
              ROS_INFO("erro failed reason!");
            }
            ROS_INFO("aborted!");
            break;
        case actionlib::SimpleClientGoalState::PREEMPTED:
          feedback_to_commander(sessionid_, E_ACTION_CANCEL , FAIL_CANCEL);
          ROS_INFO("failed reason cancel!");
          break;
        default:
            break;
    }
}

void ChassisMoveInterface::feedback_callback(const u_msgs::MoveToFeedbackConstPtr& feedback)
{
  static int percent = 0;
  if( abs((int)feedback->complete_percent - percent) >=10){
    ROS_INFO("the action run: %d percent", feedback->complete_percent);
    percent = feedback -> complete_percent;
  }
}

//------------------main------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "chassis_move_to_interface");
  ros::NodeHandle nh;
  ChassisMoveInterface move_interface(&nh);
  ROS_INFO("chassis_move_to interface start!");
  ROS_INFO("spin---------------------------!");
      ros::MultiThreadedSpinner s(10);
    ros::spin(s);;
  ROS_INFO("exit!");
  return 1;
}
