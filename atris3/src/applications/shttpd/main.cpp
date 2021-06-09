#define HFS_PATH_ROOT "/home/atris/atris_app/bin/dist"
#define HFS_PATH_DIR  "/home/atris/atris_app/bin/dist/hfs"
#define SQLITE_ROSSHTTPD_FILE   "/userdata/impdata/rosshttpd.db"
#define ROSSHTTPD_TABLE_COLUMN  3
#define ROSSHTTPD_TABLE "CREATE TABLE IF NOT EXISTS [rosshttpd] (" \
        "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
        "[user] TEXT NOT NULL," \
        "[pwd] TEXT NOT NULL)"

#include "protocol_lws_minimal.cpp"

static void initDatabaseTable() {
  char **result;
  int row, column;

retry:
  SqliteEngine::execSQL(ROSSHTTPD_TABLE, SQLITE_ROSSHTTPD_FILE);
  SqliteEngine::query("SELECT * FROM rosshttpd", &result, &row, &column, SQLITE_ROSSHTTPD_FILE);
  if (row <= 0) {
    SqliteEngine::execSQL("INSERT INTO rosshttpd(user, pwd) VALUES('admin', '576fcf46b1')", SQLITE_ROSSHTTPD_FILE);
  } else {
    if (column != ROSSHTTPD_TABLE_COLUMN) {
      SqliteEngine::execSQL("DROP TABLE rosshttpd", SQLITE_ROSSHTTPD_FILE);
      goto retry;
    }
  }

  SqliteEngine::freeQuery(result);
}

static bool authentication(const std::string& user, const std::string& pwd) {
  char **result;
  int row, column;
  SqliteEngine::query("SELECT user,pwd FROM rosshttpd", &result, &row, &column, SQLITE_ROSSHTTPD_FILE);
  std::string u = row > 0 ? result[column] : "";
  std::string p = row > 0 ? result[column + 1] : "";
  SqliteEngine::freeQuery(result);
  if (u == user && p == pwd) {
    return true;
  }
  return false;
}

static void
show_login(struct shttpd_arg *arg) {
  shttpd_printf(arg, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  shttpd_printf(arg, "<html>");
  shttpd_printf(arg, "<head>");
  shttpd_printf(arg, "<meta charset='UTF-8'>");
  shttpd_printf(arg, "<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
  shttpd_printf(arg, "<meta http-equiv='X-UA-Compatible' content='IE=edge,chrome=1'>");
  shttpd_printf(arg, "<meta http-equiv='Expires' content='0'>");
  shttpd_printf(arg, "<meta http-equiv='Pragma' content='no-cache'>");
  shttpd_printf(arg, "<meta http-equiv='Cache-control' content='no-cache'>");
  shttpd_printf(arg, "<meta http-equiv='Cache' content='no-cache'>");
  shttpd_printf(arg, "<link type='text/css' rel='stylesheet' href='index.css?v=%llu'/>", (uint64_t)(ros::Time::now().toSec() * 1000));
  shttpd_printf(arg, "<script type='text/javascript' src='index.js?v=%llu'></script>", (uint64_t)(ros::Time::now().toSec() * 1000));
  shttpd_printf(arg, "</head>");
  shttpd_printf(arg, "<body>");
  shttpd_printf(arg, "<div style='position: absolute; left: 50%; top: 20%; margin-left:-175px;'>");
  shttpd_printf(arg, "<h1 style='text-align:center'>Welcome to Robot</h1>");
  shttpd_printf(arg, "<form method='POST' action='main_page' style='display:inline-block;' enctype='text/plain'>");
  shttpd_printf(arg, "<fieldset style='border: solid 1px #aaa;border-radius: 5px;padding:10px;width: 350px;'>");
  shttpd_printf(arg, "<div style='text-align: center;'><label>账号:<input type='text' name='login_user' maxlength='15' "
                     "placeholder='请输入登录账号' style='width:200px; margin: 5px;'/></label></div>");
  shttpd_printf(arg, "<div style='text-align: center;'><label>密码:<input type='password' name='login_pwd' maxlength='15' "
                     "placeholder='请输入登录密码' style='width:200px; margin: 5px;'/></label></div>");
  shttpd_printf(arg, "<div style='text-align: center;'><input style='width:80px;margin: 10px;' type='submit' value='登录'/></div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "</form>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</body>");
  shttpd_printf(arg, "</html>");
  arg->flags |= SHTTPD_END_OF_OUTPUT;
}

static void
show_login_error(struct shttpd_arg *arg, const char* error) {
  shttpd_printf(arg, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  shttpd_printf(arg, "<html>");
  shttpd_printf(arg, "<head>");
  shttpd_printf(arg, "<meta charset='UTF-8'>");
  shttpd_printf(arg, "<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
  shttpd_printf(arg, "<meta http-equiv='X-UA-Compatible' content='IE=edge,chrome=1'>");
  shttpd_printf(arg, "<meta http-equiv='Expires' content='0'>");
  shttpd_printf(arg, "<meta http-equiv='Pragma' content='no-cache'>");
  shttpd_printf(arg, "<meta http-equiv='Cache-control' content='no-cache'>");
  shttpd_printf(arg, "<meta http-equiv='Cache' content='no-cache'>");
  shttpd_printf(arg, "<link type='text/css' rel='stylesheet' href='index.css?v=%llu'/>", (uint64_t)(ros::Time::now().toSec() * 1000));
  shttpd_printf(arg, "<script type='text/javascript' src='index.js?v=%llu'></script>", (uint64_t)(ros::Time::now().toSec() * 1000));
  shttpd_printf(arg, "</head>");
  shttpd_printf(arg, "<body>");
  shttpd_printf(arg, "<div style='position: absolute; left: 50%; top: 20%; margin-left:-175px;'>");
  shttpd_printf(arg, "<h1 style='text-align:center'>Welcome to Robot</h1>");
  shttpd_printf(arg, "<form method='POST' action='main_page' style='display:inline-block;' enctype='text/plain'>");
  shttpd_printf(arg, "<fieldset style='border: solid 1px #aaa;border-radius: 5px;padding:10px;width: 350px;'>");
  shttpd_printf(arg, "<div style='text-align: center;'><label>账号:<input type='text' name='login_user' maxlength='15' "
                     "placeholder='请输入登录账号' style='width:200px; margin: 5px;'/></label></div>");
  shttpd_printf(arg, "<div style='text-align: center;'><label>密码:<input type='password' name='login_pwd' maxlength='15' "
                     "placeholder='请输入登录密码' style='width:200px; margin: 5px;'/></label></div>");
  shttpd_printf(arg, "<div style='text-align: center;'><font color='red'>%s</font></div>", error);
  shttpd_printf(arg, "<div style='text-align: center;'><input style='width:80px;margin: 10px;' type='submit' value='登录'/></div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "</form>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</body>");
  shttpd_printf(arg, "</html>");
  arg->flags |= SHTTPD_END_OF_OUTPUT;
}

static void
show_main_page(struct shttpd_arg *arg)
{
  const char* crequest_method = shttpd_get_env(arg, "REQUEST_METHOD");
  std::string request_method = crequest_method ? std::string(crequest_method) : "";

  if (!strcmp(request_method.c_str(), "POST")) {
    /* If not all data is POSTed, wait for the rest */
    if (arg->flags & SHTTPD_MORE_POST_DATA) {
      return;
    }

    if (!arg->in.buf || arg->in.len <= 0)
      return;

    std::string login_user, login_pwd;
    std::string in_buf = std::string(arg->in.buf, arg->in.len);
    std::size_t user_pos = in_buf.find("login_user=");
    std::size_t pwd_pos = in_buf.find("login_pwd=");
    log_info("%s in_buf(%s)", __FUNCTION__, in_buf.c_str());
    if (user_pos == std::string::npos || pwd_pos == std::string::npos) {
      show_login_error(arg, "登录参数缺失");
      return;
    }
    if (user_pos < pwd_pos) {
      login_user = in_buf.substr(user_pos+11, pwd_pos-(user_pos+11));
      login_pwd = in_buf.substr(pwd_pos+10);
    } else {
      login_user = in_buf.substr(user_pos+11);
      login_pwd = in_buf.substr(pwd_pos+10, user_pos-(pwd_pos+10));
    }
    login_user.erase(login_user.find_last_not_of(" \t\f\v\n\r") + 1);
    login_user.erase(0, login_user.find_first_not_of(" \t\f\v\n\r"));
    login_pwd.erase(login_pwd.find_last_not_of(" \t\f\v\n\r") + 1);
    login_pwd.erase(0, login_pwd.find_first_not_of(" \t\f\v\n\r"));
    log_info("%s login_user(%s), login_pwd(%s)", __FUNCTION__, login_user.c_str(), login_pwd.c_str());
    if(login_user.empty() || login_pwd.empty()) {
      show_login_error(arg, "账号和密码不能为空");
      return;
    }
    
    if (!authentication(login_user, login_pwd)) {
      show_login_error(arg, "账号和密码错误");
      return;
    }
  } else {
    show_login_error(arg, "请输入账号和密码登录");
    return;
  }

  shttpd_printf(arg, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n");
  shttpd_printf(arg, "<html>");
  shttpd_printf(arg, "<head>");
  shttpd_printf(arg, "<meta charset='UTF-8'>");
  shttpd_printf(arg, "<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
  shttpd_printf(arg, "<meta http-equiv='Expires' content='0'>");
  shttpd_printf(arg, "<meta http-equiv='Pragma' content='no-cache'>");
  shttpd_printf(arg, "<meta http-equiv='Cache-control' content='no-cache'>");
  shttpd_printf(arg, "<meta http-equiv='Cache' content='no-cache'>");
  shttpd_printf(arg, "<link type='text/css' rel='stylesheet' href='index.css?v=%llu'/>", (uint64_t)(ros::Time::now().toSec() * 1000));
  shttpd_printf(arg, "<script type='text/javascript' src='index.js?v=%llu'></script>", (uint64_t)(ros::Time::now().toSec() * 1000));
  shttpd_printf(arg, "<script type='text/javascript' src='diagnostics.js?v=%llu'></script>", (uint64_t)(ros::Time::now().toSec() * 1000));
  shttpd_printf(arg, "<script type='text/javascript' src='protocol.js?v=%llu'></script>", (uint64_t)(ros::Time::now().toSec() * 1000));
  shttpd_printf(arg, "<script type='text/javascript' src='nipplejs.js?v=%llu'></script>", (uint64_t)(ros::Time::now().toSec() * 1000));
  shttpd_printf(arg, "</head>");
  shttpd_printf(arg, "<body>");
  shttpd_printf(arg, "<div align='center'>");
  shttpd_printf(arg, "<div id='tab'>");
  shttpd_printf(arg, "<h1 style='padding:40px'>Welcome to Robot</h1>");
  shttpd_printf(arg, "<div id='tab-header'>");
  shttpd_printf(arg, "<ul>");
  shttpd_printf(arg, "<li class='selected'>本体管控</li>");
  shttpd_printf(arg, "<li>参数配置</li>");
  shttpd_printf(arg, "<li>诊断信息</li>");
  shttpd_printf(arg, "</ul>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='tab-content'>");
  shttpd_printf(arg, "<div class='dom' style='display: block;'>");  /* dom { */
  shttpd_printf(arg, "<div style='display:inline-block; text-align:center; padding: 20px'>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<div style='text-align: left;'><legend><label><input id='openjoystick' type='checkbox'/>开启操纵杆</label></legend></div>");
  shttpd_printf(arg, "<div align='center'>");
  shttpd_printf(arg, "<div id='joystick_zone' style='position: relative; width: 300px; height: 300px;'></div>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<script type='text/javascript' src='joystick.js?v=%llu'></script>", (uint64_t)(ros::Time::now().toSec() * 1000));
  shttpd_printf(arg, "</fieldset>");

  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>指示灯设置</legend>");
  shttpd_printf(arg, "<div>");
  shttpd_printf(arg, "<label>指示灯样式:   <select id='indicator_style' onchange='request_set_indicator_light_style(this.value,this.options[this.selectedIndex].text)'>");
  shttpd_printf(arg, "<option value=0>静止</option>");
  shttpd_printf(arg, "<option value=1>呼吸</option>");
  shttpd_printf(arg, "<option value=2>频闪</option>");
  shttpd_printf(arg, "</select>");
  shttpd_printf(arg, "</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div>");
  shttpd_printf(arg, "<label>指示灯颜色:   <select id='indicator_color' onchange='request_set_indicator_light_color(this.value,this.options[this.selectedIndex].text)'>");
  shttpd_printf(arg, "<option value=1>红色</option>");
  shttpd_printf(arg, "<option value=2>橙色</option>");
  shttpd_printf(arg, "<option value=3>绿色</option>");
  shttpd_printf(arg, "<option value=4>青色</option>");
  shttpd_printf(arg, "<option value=5>蓝色</option>");
  shttpd_printf(arg, "<option value=6>紫色</option>");
  shttpd_printf(arg, "<option value=7>白色</option>");
  shttpd_printf(arg, "</select>");
  shttpd_printf(arg, "</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>指示灯亮度:");
  shttpd_printf(arg, "<input type='range' id='brightness' value='50' min='0' max='100' step='1' onchange='brightness_output.value=this.value;request_set_indicator_brightness(this.value)'/>");
  shttpd_printf(arg, "</input>");
  shttpd_printf(arg, "<output id='brightness_output' for='brightness'>50</output>");
  shttpd_printf(arg, "<span>%</span>");
  shttpd_printf(arg, "</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>指示灯释放控制权测试: </label>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='发送释放指示灯控制权' onclick='request_release_indicator_light_control()'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");

  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>回充单元测试</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>导航运动指示: </label>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='运动停止成功' onclick='request_send_robot_in_position(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='运动停止失败' onclick='request_send_robot_in_position(0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>机器人下桩测试: </label>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='发送机器人下桩' onclick='request_send_robot_leave_pile()'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");

  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>机器人告警模拟</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>事件告警模拟测试开关: </label>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='模拟测试开关打开' onclick='request_set_robot_event_simulation(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='模拟测试开关关闭' onclick='request_set_robot_event_simulation(0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>bms告警模拟: </label>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='bms告警产生' onclick='request_set_robot_warning(1,1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='bms告警消失' onclick='request_set_robot_warning(1,0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>驱动器告警模拟: </label>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='驱动器告警产生' onclick='request_set_robot_warning(2,1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='驱动器告警消失' onclick='request_set_robot_warning(2,0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>设备告警模拟: </label>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='设备云台告警产生' onclick='request_set_robot_warning(3,1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='设备云台告警消失' onclick='request_set_robot_warning(3,0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>ccb告警模拟: </label>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='ccb告警产生' onclick='request_set_robot_warning(4,1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='ccb告警消失' onclick='request_set_robot_warning(4,0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>sys告警模拟: </label>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='sys告警产生' onclick='request_set_robot_warning(5,1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='sys告警消失' onclick='request_set_robot_warning(5,0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");

  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>机器人模式单元测试</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='进入待机' onclick='request_robot_mode_standby()'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='待机唤醒' onclick='request_robot_wake_up_from_standby()'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='设置运输模式' onclick='request_set_robot_transport_mode(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='解除运输模式' onclick='request_set_robot_transport_mode(0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");

  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>宣传播报</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<form method='post' enctype='multipart/form-data' action='/hfs/upload'>");
  shttpd_printf(arg, "<input type='file' name='file' id='robot_music_upload_file' accept='.mp3'/>");
  shttpd_printf(arg, "<input type='submit' value='上传' onclick='set_robot_music_url()'>");
  shttpd_printf(arg, "</form>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input type='text' id='robot_music_url' placeholder='请输入宣传播报地址' style='width:400px; margin: 5px;'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='同步' onclick='request_music_transport(document.getElementById(\"robot_music_url\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='开始播放' onclick='request_roshttpd_music_play()'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='暂停播放' onclick='request_music_play(\"pause\")'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='恢复播放' onclick='request_music_play(\"resume\")'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='停止播放' onclick='request_music_play(\"stop\")'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>巡逻控制</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='速度低' onclick='request_set_speed(0.2)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='速度中' onclick='request_set_speed(0.4)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='速度高' onclick='request_set_speed(0.6)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='暂停巡逻' onclick='request_nav_switch_with_shttpd(0)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='恢复巡逻' onclick='request_nav_switch_with_shttpd(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='停止巡逻' onclick='request_nav_switch_with_shttpd(2)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input type='text' id='map_name' placeholder='请输入地图名称' style='width:200px; margin: 5px;'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='设置地图' onclick='request_set_map(document.getElementById(\"map_name\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='一键返回' onclick='request_nav_return(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='取消一键返回' onclick='request_nav_return(0)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='一键充电' onclick='request_nav_recharge(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='取消一键充电' onclick='request_nav_recharge(0)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='一键下桩' onclick='request_nav_leave_pile(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='取消一键下桩' onclick='request_nav_leave_pile(0)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='紧急刹车' onclick='request_emergency_stop(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='解除刹车' onclick='request_emergency_stop(0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='前进' onclick='request_robot_move_control(0)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='后退' onclick='request_robot_move_control(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='左转' onclick='request_robot_move_control(2)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='右转' onclick='request_robot_move_control(3)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>音量控制</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='音量+' onclick='request_set_volume(10)'/>");
  shttpd_printf(arg, "<input type='text' id='global_volume' readonly='readonly' style='width: 40px; height: 30px; text-align: center'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='音量-' onclick='request_set_volume(-10)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='打开静音' onclick='request_set_mute(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='关闭静音' onclick='request_set_mute(0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>云台控制</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='上仰' onclick='request_ptz_move_control(0)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='下俯' onclick='request_ptz_move_control(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='左转' onclick='request_ptz_move_control(2)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='右转' onclick='request_ptz_move_control(3)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='停止' onclick='request_ptz_move_control(4)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='拍照' onclick='request_ptz_capture()'/>");
  // shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='开始录像' onclick='request_ptz_record(0)'/>");
  // shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='停止录像' onclick='request_ptz_record(1)'/>");
  // shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='取消录像' onclick='request_ptz_record(2)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "start:<input type='text' id='start_point_x1' placeholder='x1' style='width:60px; margin: 1px;'/>");
  shttpd_printf(arg, "<input type='text' id='start_point_y1' placeholder='y1' style='width:60px; margin: 1px;'/>");
  shttpd_printf(arg, "end:<input type='text' id='end_point_x2' placeholder='x2' style='width:60px; margin: 1px;'/>");
  shttpd_printf(arg, "<input type='text' id='end_point_y2' placeholder='y2' style='width:60px; margin: 1px;'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='请求' onclick='request_ptz_ir_temperature_info(document.getElementById(\"start_point_x1\").value,document.getElementById(\"start_point_y1\").value,document.getElementById(\"end_point_x2\").value,document.getElementById(\"end_point_y2\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "温度:<input type='text' id='termperature_max' placeholder='max' style='width:60px; margin: 1px;'/>");
  shttpd_printf(arg, "<input type='text' id='termperature_min' placeholder='min' style='width:60px; margin: 1px;'/>");
  shttpd_printf(arg, "<input type='text' id='termperature_ave' placeholder='ave' style='width:60px; margin: 1px;'/>");
  shttpd_printf(arg, "<input type='text' id='termperature_diff' placeholder='diff' style='width:60px; margin: 1px;'/>");
  shttpd_printf(arg, "</div>");


  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "转速:<input type='text' id='ptz_h_speed' placeholder='水平' style='width:60px; margin: 1px;'/>");
  shttpd_printf(arg, "<input type='text' id='ptz_v_speed' placeholder='垂直' style='width:60px; margin: 1px;'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='设置' onclick='request_ptz_speed(document.getElementById(\"ptz_h_speed\").value,document.getElementById(\"ptz_v_speed\").value)'/>");
  shttpd_printf(arg, "旋转:<input type='text' id='ptz_hv_angle' placeholder='角度' style='width:60px; margin: 1px;'/>");
  shttpd_printf(arg, "<label>方向:<select id='direction_type_select' style='margin: 5px;' onchange='direction_type_select(this.options[this.options.selectedIndex].value,document.getElementById(\"ptz_hv_angle\").value)'>"
                     "<option id='horizontal_angle' value='0'>水平</option>"
                     "<option id='vertical_angle' value='1'>垂直</option>"
                     "</select></label>");
  shttpd_printf(arg, "</div>");

  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "video:<input style='margin: 5px;' type='button' value='start' onclick='request_ptz_record(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='stop' onclick='request_ptz_record(0)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='cancel' onclick='request_ptz_record(2)'/>");
  shttpd_printf(arg, "<div style='text-align: left;'>");

  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "audio:<input style='margin: 5px;' type='button' value='start' onclick='request_sound_record(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='stop' onclick='request_sound_record(0)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='cancel' onclick='request_sound_record(2)'/>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='ptz:' onclick='request_ptz_ptzf(0)'/>");
 
  shttpd_printf(arg, " P:<input type='text' id='pan'  placeholder='P' readOnly='true' style='width:60px; margin: 5px;'/>");
  shttpd_printf(arg, " T:<input type='text' id='tilt'   placeholder='T' readOnly='true' style='width:60px; margin: 5px;'/>");
  shttpd_printf(arg, " Zoom:<input type='text' id='zoom'   placeholder='Zoom' readOnly='true' style='width:60px; margin: 5px;'/>");
  // shttpd_printf(arg, "</div>");
  // shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, " Focus:<input type='text' id='focus'   placeholder='Focus' readOnly='true' style='width:60px; margin: 5px;'/>");
  shttpd_printf(arg, "</div>");

  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='focus +' onclick='request_ptz_focus_control(0)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='focus -' onclick='request_ptz_focus_control(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='雨刮开' onclick='request_switch_ptz_wiper(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='雨刮关' onclick='request_switch_ptz_wiper(0)'/>");

  shttpd_printf(arg, "<div style='text-align: left;'>");
  // shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='zoom +' onclick='request_ptz_zoom_control(0)'/>");
  // shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='zoom -' onclick='request_ptz_zoom_control(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='补光开' onclick='request_switch_ptz_light(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='补光关' onclick='request_switch_ptz_light(0)'/>");

  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='拍照' onclick='request_ptz_capture()'/>");
  shttpd_printf(arg, " 可见光:<input type='text' id='visible_light_url'  placeholder='可见光url' readOnly='true' style='width:370px; margin: 5px;'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, " 红外光:<input type='text' id='infrared_url'   placeholder='红外光url' readOnly='true' style='width:370px; margin: 5px;'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg,"zoom: 0:<input id='range_zoom_value' type='range' value='1' min='1' max='32' step='1' onchange='changeZoom(document.getElementById(\"range_zoom_value\").value)'/>:32");


  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>TTS播报</legend>");
  shttpd_printf(arg, "<div style='text-align: center;'>");
  shttpd_printf(arg, "<textarea id='tts_play_text' placeholder='请输入要播报的文字'></textarea>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='播报' onclick='request_tts_play_start(document.getElementById(\"tts_play_text\").value)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='停止' onclick='request_tts_play_stop()'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='男声' onclick='request_set_tts_speaker(0)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='女声' onclick='request_set_tts_speaker(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='中文' onclick='request_set_tts_language(0)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='英文' onclick='request_set_tts_language(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='启用' onclick='request_tts_enable(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='禁用' onclick='request_tts_enable(0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>强声驱散</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='打开' onclick='request_sonic_disperse(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='关闭' onclick='request_sonic_disperse(0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>中框风扇</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='三挡' onclick='request_set_fan_middle(3)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='二档' onclick='request_set_fan_middle(2)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='一档' onclick='request_set_fan_middle(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='关闭' onclick='request_set_fan_middle(0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>底框风扇</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='三挡' onclick='request_set_fan_bottom(3)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='二档' onclick='request_set_fan_bottom(2)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='一档' onclick='request_set_fan_bottom(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='关闭' onclick='request_set_fan_bottom(0)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>IMSI</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<form method='post' enctype='multipart/form-data' action='/hfs/upload'>");
  shttpd_printf(arg, "<input type='file' name='file' id='robot_upload_imsi_file' accept='.txt'/>");
  shttpd_printf(arg, "<input type='submit' value='上传' onclick='set_imsi_file_url()'>");
  shttpd_printf(arg, "</form>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input type='text' id='robot_imsi_url' placeholder='请输入IMSI底库文件地址' style='width:400px; margin: 5px;'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='同步' onclick='request_imsi_file_transport(document.getElementById(\"robot_imsi_url\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input type='text' id='robot_imsi_detection_file_url' readOnly='true' style='width:370px; margin: 5px;'/>");
  shttpd_printf(arg, "<input  style='margin: 5px;' type='button' value='抓取识别IMSI' onclick='request_get_detection_imsi_file_url()'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend>机器人日志</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input type='text' id='robot_log_url' readOnly='true' style='width:370px; margin: 5px;'/>");
  shttpd_printf(arg, "<input  style='margin: 5px;' type='button' value='抓取日志' onclick='request_mcb_log()'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>软件升级</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<form method='post' enctype='multipart/form-data' action='/hfs/upload'>");
  shttpd_printf(arg, "<input type='file' name='file' id='robot_upgrade_upload_file' accept='.tar.gz'/>");
  shttpd_printf(arg, "<input type='submit' value='上传' onclick='set_robot_upgrade_url()'>");
  shttpd_printf(arg, "</form>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input type='text' id='robot_upgrade_url' placeholder='请输入离线升级包地址' style='width:400px; margin: 5px;'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='升级' onclick='request_sw_upgrade(document.getElementById(\"robot_upgrade_url\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<table border='0' cellspacing='5'>");
  shttpd_printf(arg, "<style type='text/css'>");
  shttpd_printf(arg, "td.upgrade_progress {padding: 0px 60px 0px 0px}");
  shttpd_printf(arg, "td.pure_text {padding: 0px 0px 0px 0px}");
  shttpd_printf(arg, "</style>");
  shttpd_printf(arg, "<td class='upgrade_progress' style='display:inline-block;width: 380px;height: 25px;'>");
  //shttpd_printf(arg, "<textarea style='resize:none;width: 250px;height: 25px;'></textarea>");
  shttpd_printf(arg, "<input type='text' id='upgrade_progress_textarea' readOnly='true' style='width:370px; margin: 5px;'/>");
  shttpd_printf(arg, "</td>");
  shttpd_printf(arg, "<td class='pure_text' style='display:inline-block;'>");
  shttpd_printf(arg, "<span>升级进度提示</span>");
  shttpd_printf(arg, "</td>");
  shttpd_printf(arg, "</table>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</div>"); /* } dom */
  shttpd_printf(arg, "<div class='dom' style='display: none;'>"); /* dom { */
  shttpd_printf(arg, "<form method='POST' style='display:inline-block; text-align:center; padding: 20px' enctype='text/plain'>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>自动回充</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>回充类型:<select id='global_dock_type_select' style='margin: 5px;' onchange='onchange_dock_type(this.options[this.options.selectedIndex].value)'>"
                     "<option id='global_dock_1_0' value='1'>桩回充</option>"
                     "<option id='global_dock_0_0' value='0'>房回充</option>"
                     "</select></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>回充房宽:<input type='text' id='global_dock_wall_width' style='width:250px; margin: 5px;'/>厘米</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_udock("
                     "document.getElementById(\"global_dock_wall_width\").value,"
                     "document.getElementById(\"global_dock_type_select\").options[document.getElementById(\"global_dock_type_select\").options.selectedIndex].value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>路由器</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>路由类型:<select id='global_router_type_select' style='margin: 5px;'><option id='global_router_type_gosuncn' value='gosuncn'>高新兴</option>"
                     "<option id='global_router_type_hongdian' value='hongdian'>宏电</option></select></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>路由地址:<input type='text' id='global_router_ip' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>登录账号:<input type='text' id='global_router_user' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>登录密码:<input type='text' id='global_router_psw' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_router("
                     "document.getElementById(\"global_router_type_select\").options[document.getElementById(\"global_router_type_select\").options.selectedIndex].value,"
                     "document.getElementById(\"global_router_ip\").value, document.getElementById(\"global_router_user\").value, document.getElementById(\"global_router_psw\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>设备地址</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>云台相机:<input type='text' id='global_ptz_ip' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>云台盒子:<input type='text' id='global_ptz_box_ip' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>四路相机:<input type='text' id='global_nvr_ip' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_devices_ip("
                     "document.getElementById(\"global_ptz_ip\").value, document.getElementById(\"global_ptz_box_ip\").value, document.getElementById(\"global_nvr_ip\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>信令服务</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>服务地址:<input type='text' id='global_mqtt_host' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>服务端口:<input type='text' id='global_mqtt_port' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>登录账号:<input type='text' id='global_mqtt_username' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>登录密码:<input type='text' id='global_mqtt_password' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>信令加密:<input type='text' id='global_mqtt_encrypt' style='width:250px; margin: 5px;'/>(加密=1, 明文=0)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>心跳间隔:<input type='text' id='global_mqtt_keepalive' style='width:250px; margin: 5px;'/>秒</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_mqtt("
                     "document.getElementById(\"global_mqtt_host\").value, document.getElementById(\"global_mqtt_port\").value, "
                     "document.getElementById(\"global_mqtt_username\").value, document.getElementById(\"global_mqtt_password\").value, "
                     "document.getElementById(\"global_mqtt_encrypt\").value, document.getElementById(\"global_mqtt_keepalive\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>影子服务器</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>topic url:<input type='text' id='remote_mqtt_topic_url' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>服务地址:<input type='text' id='remote_mqtt_host' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>服务端口:<input type='text' id='remote_mqtt_port' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>登录账号:<input type='text' id='remote_mqtt_username' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>登录密码:<input type='text' id='remote_mqtt_password' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>信令加密:<input type='text' id='remote_mqtt_encrypt' style='width:250px; margin: 5px;'/>(加密=1, 明文=0)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>心跳间隔:<input type='text' id='remote_mqtt_keepalive' style='width:250px; margin: 5px;'/>秒</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_remote_mqtt("
                     "document.getElementById(\"remote_mqtt_topic_url\").value,"
                     "document.getElementById(\"remote_mqtt_host\").value, document.getElementById(\"remote_mqtt_port\").value, "
                     "document.getElementById(\"remote_mqtt_username\").value, document.getElementById(\"remote_mqtt_password\").value, "
                     "document.getElementById(\"remote_mqtt_encrypt\").value, document.getElementById(\"remote_mqtt_keepalive\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>ABI服务</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>鉴权地址:<input type='text' id='global_abi_base_url' style='width:380px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>设备地址:<input type='text' id='global_abi_robotinfo_url' style='width:380px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>心跳地址:<input type='text' id='global_abi_timestamp_url' style='width:380px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>事件地址:<input type='text' id='global_abi_post_event_url' style='width:380px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>开启服务:<input type='text' id='global_abi_enable' style='width:250px; margin: 5px;'/>(开启=1, 禁用=0)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_abi("
                     "document.getElementById(\"global_abi_base_url\").value, document.getElementById(\"global_abi_robotinfo_url\").value, "
                     "document.getElementById(\"global_abi_timestamp_url\").value, document.getElementById(\"global_abi_post_event_url\").value, document.getElementById(\"global_abi_enable\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>文件服务</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>服务类型:<select id='global_hfs_type_select' style='margin: 5px;' onchange='onchange_hfs_type(this.options[this.options.selectedIndex].value)'><option id='global_hfs_type_qiniu' value='qiniu'>qiniu</option>"
                     "<option id='global_hfs_type_hfs' value='hfs'>hfs</option></select></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>服务地址:<input type='text' id='global_hfs_url' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_hfs("
                     "document.getElementById(\"global_hfs_type_select\").options[document.getElementById(\"global_hfs_type_select\").options.selectedIndex].value, document.getElementById(\"global_hfs_url\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>NTP服务</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>服务地址:<input type='text' id='global_ntp_server_list' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>时间时区:<input type='text' id='global_timezone' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_ntp("
                     "document.getElementById(\"global_ntp_server_list\").value, document.getElementById(\"global_timezone\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>voip参数设置</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>自动设置:<input type='text' id='global_voip_auto_setting' style='width:250px; margin: 5px;'/>(自动=1, 手动=0)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>服务地址:<input type='text' id='global_voip_sip_server' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_voip_params("
                     "document.getElementById(\"global_voip_auto_setting\").value, document.getElementById(\"global_voip_sip_server\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>视觉服务参数设置</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>服务地址:<input type='text' id='global_vision_host' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>服务端口:<input type='text' id='global_vision_port' style='width:250px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_vision("
                     "document.getElementById(\"global_vision_host\").value, document.getElementById(\"global_vision_port\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>遇障参数设置</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>遇障导航策略:<input type='text' id='global_obstacle_mode' style='width:50px; margin: 5px;'/>(不制动停障=0,制动停障=1,绕障=2)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>停障超时时长:<input type='text' id='global_obstacle_timeout' style='width:50px; margin: 5px;'/>(5s~1000s)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>停障超时机制:<input type='text' id='global_obstacle_timeout_strategy' style='width:50; margin: 5px;'/>(停止当前任务=0,重新规划导航路线=1)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_obstacle("
                     "document.getElementById(\"global_obstacle_mode\").value,document.getElementById(\"global_obstacle_timeout\").value,document.getElementById(\"global_obstacle_timeout_strategy\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");

  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>导航参数设置</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>obs_mode<input type='text' id='global_nav_obs_mode' style='width:250px; margin: 5px;'/>(default:0)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>cycle_enable<input type='text' id='global_nav_cycle_enable' style='width:250px; margin: 5px;'/>(开启=1,关闭=0)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>cycle_dst<input type='text' id='global_nav_cycle_dst' style='width:250px; margin: 5px;'/>(m)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>回充路径<input type='text' id='global_nav_without_charge_path' style='width:250px; margin: 5px;'/>(不使用=1, 使用=0)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>融合路径<input type='text' id='global_nav_without_merge_path' style='width:250px; margin: 5px;'/>(不使用=1, 使用=0)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>任务启动超时时间<input type='text' id='global_nav_task_start_timeout' style='width:250px; margin: 5px;'/>(s)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>任务运行超时时间<input type='text' id='global_nav_task_running_timeout' style='width:250px; margin: 5px;'/>(s)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>低电量回充<input type='text' id='global_nav_auto_charge_self' style='width:250px; margin: 5px;'/>(开启=1, 关闭=0)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>低电量回充触发电量<input type='text' id='global_nav_auto_charge_battery' style='width:200px; margin: 5px;'/>(default:30)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>防跌落<input type='text' id='global_nav_anti_drop_enable' style='width:200px; margin: 5px;'/>(开启=1, 关闭=0)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>TSP最优路径<input type='text' id='global_nav_tsp_enable' style='width:200px; margin: 5px;'/>(开启=1, 关闭=0)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_nav("
                     "document.getElementById(\"global_nav_obs_mode\").value, "
                     "document.getElementById(\"global_nav_cycle_dst\").value, "
                     "document.getElementById(\"global_nav_cycle_enable\").value, "
                     "document.getElementById(\"global_nav_without_charge_path\").value, "
                     "document.getElementById(\"global_nav_task_start_timeout\").value, "
                     "document.getElementById(\"global_nav_task_running_timeout\").value, "
                     "document.getElementById(\"global_nav_without_merge_path\").value, "
                     "document.getElementById(\"global_nav_auto_charge_self\").value, "
                     "document.getElementById(\"global_nav_auto_charge_battery\").value, "
                     "document.getElementById(\"global_nav_anti_drop_enable\").value, "
                     "document.getElementById(\"global_nav_tsp_enable\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>手机侦码识别</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>开启爆闪灯:<input type='text' id='global_red_blue_light_imsi_detection' style='width:250px; margin: 5px;'/>(开启=1,关闭=0)</label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='保存修改' onclick='request_rosshttpd_cfg_open_light_imsi_detection("
                     "document.getElementById(\"global_red_blue_light_imsi_detection\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>默认设置</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='恢复出厂设置' onclick='request_reset_robot(1)'/>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='恢复出厂并清除数据' onclick='request_reset_robot(2)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "<fieldset>");
  shttpd_printf(arg, "<legend style='margin: 5px;'>登录密码</legend>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>旧密码:<input type='password' id='pwd_old' maxlength='15' placeholder='请输入密码' style='width:200px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<label>新密码:<input type='text' id='pwd_new' maxlength='15' placeholder='请输入新密码' style='width:200px; margin: 5px;'/></label>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div style='text-align: left;'>");
  shttpd_printf(arg, "<input style='margin: 5px;' type='button' value='修改密码' onclick='request_rosshttpd_pwd("
                     "document.getElementById(\"pwd_old\").value, document.getElementById(\"pwd_new\").value)'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</fieldset>");
  shttpd_printf(arg, "</form>");
  shttpd_printf(arg, "</div>"); /* } dom */
  shttpd_printf(arg, "<div class='dom' style='padding: 20px; text-align: left;display: none;'>"); /* dom { */
  shttpd_printf(arg, "<div style='text-align: right;'>");
  shttpd_printf(arg, "<input type='button' value='刷新' onclick='request_robot_info()'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_base' style='display: none;'>");
  shttpd_printf(arg, "<h3>基本信息</h3>");
  shttpd_printf(arg, "<dd id='robot_info_base_sn'/>");
  shttpd_printf(arg, "<dd id='robot_info_base_upgrade_status'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_imx'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_chassis_software_ver'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_chassis_core_hw_ver'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_chassis_base_hw_ver'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_monitor_sw_ver'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_monitor_core_hw_ver'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_monitor_base_hw_ver'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_bms_hw_ver'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_bms_sw_ver'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_battery_monitor'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_power'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_gs'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_power_iap'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_ultroso_ks106'/>");
  shttpd_printf(arg, "<dd id='robot_info_version_ultroso_ks136'/>");
  
  shttpd_printf(arg, "<dd id='robot_info_base_binded'/>");
  shttpd_printf(arg, "<dd id='robot_info_base_company'/>");
  shttpd_printf(arg, "<dd id='robot_info_base_sn_name'/>");
  shttpd_printf(arg, "<dd id='robot_info_base_dept_name'/>");
  shttpd_printf(arg, "<dd id='robot_info_base_sustation'/>");

  shttpd_printf(arg, "<dd id='robot_info_base_volume_muted'/>");
  shttpd_printf(arg, "<dd id='robot_info_base_volume_value'/>");
  shttpd_printf(arg, "<dd id='robot_info_base_speed'/>");
  shttpd_printf(arg, "<dd id='robot_info_patrol_status'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_main_lidar' style='display: none;'>");
  shttpd_printf(arg, "<h3>主雷达</h3>");
  shttpd_printf(arg, "<dd id='robot_info_main_lidar_status'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_slave_lidar' style='display: none;'>");
  shttpd_printf(arg, "<h3>副雷达</h3>");
  shttpd_printf(arg, "<dd id='robot_info_slave_lidar_status'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_gyro' style='display: none;'>");
  shttpd_printf(arg, "<h3>陀螺仪</h3>");
  shttpd_printf(arg, "<dd id='robot_info_gyro_status'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_odom' style='display: none;'>");
  shttpd_printf(arg, "<h3>里程计</h3>");
  shttpd_printf(arg, "<dd id='robot_info_odom_status'/>");
  shttpd_printf(arg, "<dd id='robot_info_odom_odo'/>");
  shttpd_printf(arg, "<dd id='robot_info_odom_speed_linear'/>");
  shttpd_printf(arg, "<dd id='robot_info_odom_speed_theta'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_ultrasound' style='display: none;'>");
  shttpd_printf(arg, "<h3>超声状态</h3>");
  shttpd_printf(arg, "<dd id='robot_info_ultrasound_status'/>");
  shttpd_printf(arg, "<dd id='robot_info_ultrasound_data'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_gaussian_status' style='display: none;'>");
  shttpd_printf(arg, "<h3>导航盒子</h3>");
  shttpd_printf(arg, "<dd id='robot_info_gaussian_status_data' style='margin: 4px'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_brake' style='display: none;'>");
  shttpd_printf(arg, "<h3>急停状态</h3>");
  shttpd_printf(arg, "<dd id='robot_info_brake_button'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_charge'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_charge_bumper'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_front_bumper'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_tear_bumper'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_front_bumper_type'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_tear_bumper_type'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_source_button'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_source_front_bumper'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_source_tear_bumper'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_source_can'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_source_charge'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_source_charge_bumper'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_source_reboot'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_source_iap'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_control_CEmergency_brake'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_control_CMotor_brake'/>");
  shttpd_printf(arg, "<dd id='robot_info_brake_control_W_software_brake'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_temp_humi' style='display: none;'>");
  shttpd_printf(arg, "<h3>温湿度信息</h3>");
  shttpd_printf(arg, "<dd id='robot_info_temp_humi'/>");
  shttpd_printf(arg, "<dd id='robot_info_temp_humi_12v'/>");
  shttpd_printf(arg, "<dd id='robot_info_temp_humi_24v'/>");
  shttpd_printf(arg, "<dd id='robot_info_temp_humi_humi_env'/>");
  shttpd_printf(arg, "<dd id='robot_info_temp_humi_temp_env'/>");
  shttpd_printf(arg, "<dd id='robot_info_temp_humi_motor_left'/>");
  shttpd_printf(arg, "<dd id='robot_info_temp_humi_motor_right'/>");
  shttpd_printf(arg, "<dd id='robot_info_temp_humi_imx'/>");
  shttpd_printf(arg, "<dd id='robot_info_temp_humi_kuangshi'/>");
  shttpd_printf(arg, "<dd id='robot_info_temp_humi_gaussian'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_chassis_driver' style='display: none;'>");
  shttpd_printf(arg, "<h3>底盘驱动器</h3>");
  shttpd_printf(arg, "<dd id='robot_info_chassis_driver_error'/>");
  shttpd_printf(arg, "<dd id='robot_info_chassis_driver_error1'/>");
  shttpd_printf(arg, "<dd id='robot_info_chassis_driver_type'/>");
  shttpd_printf(arg, "<dd id='robot_info_chassis_driver_bat_current'/>");
  shttpd_printf(arg, "<dd id='robot_info_chassis_driver_bat_voltage'/>");
  shttpd_printf(arg, "<dd id='robot_info_chassis_driver_motor_current'/>");
  shttpd_printf(arg, "<dd id='robot_info_chassis_driver_temp_ic'/>");
  shttpd_printf(arg, "<dd id='robot_info_chassis_driver_temp_motor_left'/>");
  shttpd_printf(arg, "<dd id='robot_info_chassis_driver_temp_motor_right'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_battery' style='display: none;'>");
  shttpd_printf(arg, "<h3>电池信息</h3>");
  shttpd_printf(arg, "<dd id='robot_info_battery_level'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_status'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_current'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_voltage'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_temp_max'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_temp_min'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_charge_cnt'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_charge_cnt'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_discharge_cnt'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_health'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_bat_num'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_relay_status_bit0'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_relay_status_bit1'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_relay_status_bit2'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_relay_status_bit3'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_relay_status_bit4'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_relay_status_bit5'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_charge_status_bit0'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_charge_status_bit1'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_charge_status_bit2'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_charge_status_bit3'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_charge_status_bit4'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_discharge_status_bit0'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_discharge_status_bit1'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_discharge_status_bit2'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_discharge_status_bit3'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_discharge_status_bit4'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_vstatus01'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_vstatus23'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_vstatus45'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_vstatus67'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_vstatus89'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_cstatus01'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_cstatus23'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_cstatus45'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_tstatus01'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_tstatus23'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_tstatus45'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_tstatus67'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_tstatus89'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_tstatus1011'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_alarm01'/>");
  shttpd_printf(arg, "<dd id='robot_info_battery_alarm23'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_sensor_hall' style='display: none;'>");
  shttpd_printf(arg, "<h3>霍尔电流</h3>");
  shttpd_printf(arg, "<dd id='robot_info_sensor_hall1'/>");
  shttpd_printf(arg, "<dd id='robot_info_sensor_hall2'/>");
  shttpd_printf(arg, "<dd id='robot_info_sensor_hall3'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_sensor_liquid' style='display: none;'>");
  shttpd_printf(arg, "<h3>液体传感器</h3>");
  shttpd_printf(arg, "<dd id='robot_info_sensor_liquid_status'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_light' style='display: none;'>");
  shttpd_printf(arg, "<h3>灯组状态</h3>");
  shttpd_printf(arg, "<dd id='robot_info_light_rb_status'/>");
  shttpd_printf(arg, "<dd id='robot_info_light_w_status'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_ppplay' style='display: none;'>");
  shttpd_printf(arg, "<h3>宣传播报</h3>");
  shttpd_printf(arg, "<dd id='robot_info_ppplay_status'/>");
  shttpd_printf(arg, "<dd id='robot_info_ppplay_name'/>");
  shttpd_printf(arg, "<dd id='robot_info_ppplay_interval'/>");
  shttpd_printf(arg, "<dd id='robot_info_ppplay_pts'/>");
  shttpd_printf(arg, "<dd id='robot_info_ppplay_duration'/>");
  shttpd_printf(arg, "</div>");

  shttpd_printf(arg, "<div id='robot_info_anti_drop' style='display: none;'>");
  shttpd_printf(arg, "<h3>防跌落传感器</h3>");
  shttpd_printf(arg, "<dd id='robot_info_anti_drop_status'/>");
  shttpd_printf(arg, "<dd id='robot_info_anti_drop_error'/>");
  shttpd_printf(arg, "</div>");

  shttpd_printf(arg, "<div id='robot_info_4g' style='display: none;'>");
  shttpd_printf(arg, "<h3>无线网络</h3>");
  shttpd_printf(arg, "<dd id='robot_info_dbcom_level'/>");
  shttpd_printf(arg, "<dd id='robot_info_dbcom_signal_value'/>");
  shttpd_printf(arg, "<dd id='robot_info_dbcom_connect_status'/>");
  shttpd_printf(arg, "<dd id='robot_info_dbcom_login_status'/>");
  shttpd_printf(arg, "<dd id='robot_info_4g_level'/>");
  shttpd_printf(arg, "<dd id='robot_info_4g_error'/>");
  shttpd_printf(arg, "<dd id='robot_info_sim_iccid'/>");
  shttpd_printf(arg, "<dd id='robot_info_sim_imsi'/>");
  shttpd_printf(arg, "<dd id='robot_info_sim_cellid'/>");
  shttpd_printf(arg, "<dd id='robot_info_router_imei'/>");
  shttpd_printf(arg, "<dd id='robot_info_router_mac'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_fan_bottom' style='display: none;'>");
  shttpd_printf(arg, "<h3>底框风扇</h3>");
  shttpd_printf(arg, "<dd id='robot_info_fan_bottom_error'/>");
  shttpd_printf(arg, "<dd id='robot_info_fan_bottom_speed_in'/>");
  shttpd_printf(arg, "<dd id='robot_info_fan_bottom_speed_out'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_fan_middle' style='display: none;'>");
  shttpd_printf(arg, "<h3>中框风扇</h3>");
  shttpd_printf(arg, "<dd id='robot_info_fan_middle_error'/>");
  shttpd_printf(arg, "<dd id='robot_info_fan_middle_speed_in'/>");
  shttpd_printf(arg, "<dd id='robot_info_fan_middle_speed_out'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_gps' style='display: none;'>");
  shttpd_printf(arg, "<h3>GPS信息</h3>");
  shttpd_printf(arg, "<dd id='robot_info_gps_error'/>");
  shttpd_printf(arg, "<dd id='robot_info_gps_lati'/>");
  shttpd_printf(arg, "<dd id='robot_info_gps_long'/>");
  shttpd_printf(arg, "<dd id='robot_info_gps_alti'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_voip' style='display: none;'>");
  shttpd_printf(arg, "<h3>语音对讲</h3>");
  shttpd_printf(arg, "<dd id='robot_info_voip_status'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_camera' style='display: none;'>");
  shttpd_printf(arg, "<h3>云台状态</h3>");
  shttpd_printf(arg, "<dd id='robot_info_camera_status'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "<div id='robot_info_disperse' style='display: none;'>");
  shttpd_printf(arg, "<h3>声波驱散</h3>");
  shttpd_printf(arg, "<dd id='robot_info_disperse_status'/>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</div>"); /* } dom */
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</div>");
  shttpd_printf(arg, "</body>");
  shttpd_printf(arg, "</html>");
  arg->flags |= SHTTPD_END_OF_OUTPUT;
}

/*
 * This callback function is used to show how to handle 404 error
 */
static void
show_404(struct shttpd_arg *arg)
{
  shttpd_printf(arg, "%s", "HTTP/1.1 200 OK\r\n");
  shttpd_printf(arg, "%s", "Content-Type: text/plain\r\n\r\n");
  shttpd_printf(arg, "%s", "Oops. File not found! ");
  shttpd_printf(arg, "%s", "This is a custom error handler.");
  arg->flags |= SHTTPD_END_OF_OUTPUT;
}

static void
voip_outgoing_call(struct shttpd_arg *arg)
{
  shttpd_printf(arg, "%s", "HTTP/1.1 200 OK\r\n");
  shttpd_printf(arg, "%s", "Content-Type: text/plain\r\n\r\n");
  shttpd_printf(arg, "%s", "voip outgoing call");
  atris_msgs::SignalMessage message;
  message.title = "notice_voip_outgoing_call";
  shttpd_req_pub.publish(message);

  arg->flags |= SHTTPD_END_OF_OUTPUT;
}

static void voip_call_established(struct shttpd_arg *arg)
{
  shttpd_printf(arg, "%s", "HTTP/1.1 200 OK\r\n");
  shttpd_printf(arg, "%s", "Content-Type: text/plain\r\n\r\n");
  shttpd_printf(arg, "%s", "voip call established");
  atris_msgs::SignalMessage message;
  message.title = "notice_voip_call_established";
  shttpd_req_pub.publish(message);

  arg->flags |= SHTTPD_END_OF_OUTPUT;
}

static void voip_call_terminated(struct shttpd_arg *arg)
{
  shttpd_printf(arg, "%s", "HTTP/1.1 200 OK\r\n");
  shttpd_printf(arg, "%s", "Content-Type: text/plain\r\n\r\n");
  shttpd_printf(arg, "%s", " voip call terminated");
  atris_msgs::SignalMessage message;
  message.title = "notice_voip_call_terminated";
  shttpd_req_pub.publish(message);

  arg->flags |= SHTTPD_END_OF_OUTPUT;
}

static char* 
binstrstr(const char * haystack, size_t hsize, 
  const char* needle, size_t nsize) {
  size_t p;
  if (haystack == NULL) return NULL;
  if (needle == NULL) return NULL;
  if (hsize < nsize) return NULL;
  for (p = 0; p <= (hsize - nsize); ++p) {
    if (memcmp(haystack + p, needle, nsize) == 0) {
      return (char*) (haystack + p);
    }
  }
  return NULL;
} 

static void
show_hfs(struct shttpd_arg *arg) {
  struct state {
    size_t  cl;    /* Content-Length  */
    size_t  nread;    /* Number of bytes read  */
    FILE  *fp;
    std::string filename;
  } *hfs_state;

  do {
    char* in_buf = arg->in.buf;
    int in_len = arg->in.len;
    
    /* If the connection was broken prematurely, cleanup */
    if (arg->flags & SHTTPD_CONNECTION_ERROR && arg->state) {
      log_error("%s SHTTPD_CONNECTION_ERROR", __FUNCTION__);
      (void) fclose(((struct state *) arg->state)->fp);
      free(arg->state);
      arg->state = NULL;
      break;
    }

    const char  *s;
    if ((s = shttpd_get_header(arg, "Content-Length")) == NULL) {
      log_error("%s Length Required", __FUNCTION__);
      shttpd_printf(arg, "HTTP/1.0 411 Length Required\r\nContent-Type: text/html\r\n\r\n");
      shttpd_printf(arg, "{\"url\":\"\",\"message\":\"Length Required\",\"status\":\"no\"}");
      arg->flags |= SHTTPD_END_OF_OUTPUT;
      break;
    }
    
    const char* ct;
    if ((ct = shttpd_get_header(arg, "Content-Type")) == NULL) {
      log_error("%s Precondition Required", __FUNCTION__);
      shttpd_printf(arg, "HTTP/1.0 412 Precondition Required\r\nContent-Type: text/html\r\n\r\n");
      shttpd_printf(arg, "{\"url\":\"\",\"message\":\"Precondition Required\",\"status\":\"no\"}");
      arg->flags |= SHTTPD_END_OF_OUTPUT;
      break;
    }

    char multipart[256] = { 0 };
    if (sscanf(ct, "multipart/form-data; boundary=%250c", multipart) != 1) {
      log_error("%s Boundary Required", __FUNCTION__);
      shttpd_printf(arg, "HTTP/1.0 413 Boundary Required\r\nContent-Type: text/html\r\n\r\n");
      shttpd_printf(arg, "{\"url\":\"\",\"message\":\"Boundary Required\",\"status\":\"no\"}");
      arg->flags |= SHTTPD_END_OF_OUTPUT;
      break;
    }
    
    hfs_state = (struct state*)arg->state;

    if (!hfs_state) {
      const char* str_crlf = "\r\n\r\n";
      const char* str_dis = "Content-Disposition: form-data;";
      char *start = binstrstr(arg->in.buf, arg->in.len, multipart, strlen(multipart));
      char *crlf = binstrstr(arg->in.buf, arg->in.len, str_crlf, strlen(str_crlf));
      char *dis = binstrstr(arg->in.buf, arg->in.len, str_dis, strlen(str_dis));
      
      char key[256] = { 0 };
      char filename[256] = { 0 };
      if (start && crlf && dis) {
        sscanf(dis, "Content-Disposition: form-data; name=\"%255[^\"]\"; filename=\"%255[^\"]\"", key, filename);
        if (!(*filename)) {
          log_error("%s filename Required", __FUNCTION__);
          shttpd_printf(arg, "HTTP/1.0 414 Filename Required\r\nContent-Type: text/html\r\n\r\n");
          shttpd_printf(arg, "{\"url\":\"\",\"message\":\"Filename Required\",\"status\":\"no\"}");
          arg->flags |= SHTTPD_END_OF_OUTPUT;
          break;
        }
      } else {
        if (arg->in.len > 1024) {
          log_error("%s Content-Disposition Required", __FUNCTION__);
          shttpd_printf(arg, "HTTP/1.0 415 Disposition Required\r\nContent-Type: text/html\r\n\r\n");
          shttpd_printf(arg, "{\"url\":\"\",\"message\":\"Disposition Required\",\"status\":\"no\"}");
          arg->flags |= SHTTPD_END_OF_OUTPUT;
        } 
        break;
      }

      if(access(HFS_PATH_ROOT, F_OK) !=0) {
        Utils::createDir(HFS_PATH_ROOT);
      }

      if(access(HFS_PATH_DIR, F_OK) !=0 ) {
        FILE *fp = NULL;
        if((fp = popen("ln -s /userdata/tmp " HFS_PATH_DIR, "r"))) {
          pclose(fp);
        }
      }

      std::string path = std::string(HFS_PATH_DIR) + std::string("/") + std::string(filename);
      hfs_state = (struct state*)calloc(1, sizeof(*hfs_state));
      arg->state = (void*)hfs_state;
      hfs_state->cl = strtoul(s, NULL, 10);
      hfs_state->fp = fopen(path.c_str(), "wb+");
      hfs_state->filename = std::string(filename);
      if (!hfs_state->fp) {
        log_error("%s Open file failed: %s", __FUNCTION__, path.c_str());
        shttpd_printf(arg, "HTTP/1.0 416 Openfile Failed %s\r\nContent-Type: text/html\r\n\r\n", path.c_str());
        shttpd_printf(arg, "{\"url\":\"\",\"message\":\"Openfile Failed %s\",\"status\":\"no\"}", path.c_str());
        arg->flags |= SHTTPD_END_OF_OUTPUT;
        free(arg->state);
        arg->state = NULL;
        break;
      }
      
      log_info("%s Data stream started", __FUNCTION__);
      in_buf = (char *)(crlf + 4);
      in_len = arg->in.len - (crlf - arg->in.buf + 4);
    }

    char *end = binstrstr(in_buf, in_len, multipart, strlen(multipart));
    if (end) {
      if ((end-in_buf-4) > 0) {
        (void) fwrite(in_buf, end-in_buf-4, 1, hfs_state->fp);
      }
    } else {
      (void) fwrite(in_buf, in_len, 1, hfs_state->fp);
    }
    
    hfs_state->nread += arg->in.len;
    
    /* Tell SHTTPD we have processed all data */
    arg->in.num_bytes = arg->in.len;

    /* Data stream finished? Close the file, and free the state */
    if (hfs_state->nread >= hfs_state->cl) {
      log_info("%s Data stream finished", __FUNCTION__);
      shttpd_printf(arg, "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
      shttpd_printf(arg, "{\"url\":\"http://10.20.18.2/hfs/%s\",\"message\":\"\",\"status\":\"ok\",", hfs_state->filename.c_str());
      shttpd_printf(arg, "\"data\":{\"url\":\"http://10.20.18.2/hfs/%s\",\"message\":\"\",\"status\":\"ok\"}}", hfs_state->filename.c_str());
      (void) fclose(hfs_state->fp);
      free(arg->state);
      arg->state = NULL;
      arg->flags |= SHTTPD_END_OF_OUTPUT;
    }
  } while(0);
}


static void
show_atris_post(struct shttpd_arg *arg) {
  Json::FastWriter jwriter; Json::Value reponse;
  std::string request_method = shttpd_get_env(arg, "REQUEST_METHOD") ? shttpd_get_env(arg, "REQUEST_METHOD") : "";
  if (!strcmp(request_method.c_str(), "POST")) {
    /* If not all data is POSTed, wait for the rest */
    if (arg->flags & SHTTPD_MORE_POST_DATA) {
      return;
    }
    if (arg->in.buf && arg->in.len > 0) {
      Json::Reader reader; Json::Value root;
      atris_msgs::SignalMessage msg;
      msg.msg.assign(arg->in.buf, arg->in.len);
      if (reader.parse(msg.msg, root)) {
        if (!root["title"].isNull() && !root["accid"].isNull() && !root["content"].isNull() 
          && !root["content"]["id"].isNull() && !root["content"]["timestamp"].isNull()) {
          log_info("%s %s", __FUNCTION__, msg.msg.c_str());
          msg.msgID = root["content"]["id"].asString();
          msg.timestamp = root["content"]["timestamp"].asInt64();
          msg.title = root["title"].asString();
          msg.account = root["accid"].asString();
          msg.type = "http";
          shttpd_req_pub.publish(msg);
        
          shm::Robot shmrbt;
          shm::iMemory_read_Robot(&shmrbt);
          ros::Time now = ros::Time::now();
          reponse["title"] = "response" + msg.title.substr(7);
          reponse["accid"] = shmrbt.robot.sn;
          reponse["content"]["id"] = msg.msgID;
          reponse["content"]["timestamp"] = (uint64_t)(now.toSec() * 1000);
          reponse["content"]["result"] = "success";
          std::string strreponse = jwriter.write(reponse);
          
          shttpd_printf(arg, "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
          shttpd_printf(arg, "%s",strreponse.c_str());
          arg->flags |= SHTTPD_END_OF_OUTPUT;
          return;
        }
      }
    }
  } else {
    shttpd_printf(arg, "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
    shttpd_printf(arg, "Unsupported request method: \"%s\"", request_method.c_str());
    arg->flags |= SHTTPD_END_OF_OUTPUT;
    return;
  }
          
  shttpd_printf(arg, "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
  shttpd_printf(arg, "Json protocol invalid!!!");
  arg->flags |= SHTTPD_END_OF_OUTPUT;
}

/*
 * Make sure we have ho zombies from CGIs
 */
static volatile int lws_exit_sig = 0;
static void
signal_handler(int sig_num)
{
  switch (sig_num) {
  case SIGCHLD:
    while (waitpid(-1, &sig_num, WNOHANG) > 0) ;
    break;
  case SIGTERM:
    log_error("signal_handler SIGTERM: lws exit!");
    lws_exit_sig = 1;
    break;
  default:
    break;
  }
}

static void *lws_main_loop() {
    do {
        struct lws_context_creation_info ctx_info;
        memset(&ctx_info, 0, sizeof(ctx_info));
        ctx_info.port = 28123;
        ctx_info.protocols = protocols;
        ctx_info.options = LWS_SERVER_OPTION_VALIDATE_UTF8;
        
        struct lws_context *context = lws_create_context(&ctx_info);
        while ( !lws_exit_sig ) {
            lws_service(context, 1000);
        }
        lws_context_destroy(context);
        
        sleep(1);
        
        lws_exit_sig = 0;
        log_warn("lws_main_loop restart.");
    } while(1);

    return NULL;
}

static void http_service_run(int argc, char *argv[]) {
  struct shttpd_ctx *ctx;
retry_shttpd:
  ctx = shttpd_init(argc, argv);
  shttpd_register_uri(ctx, "/", &show_login, NULL);
  shttpd_register_uri(ctx, "/main_page", &show_main_page, NULL);
  /*
  shttpd_register_uri(ctx, "/voip/outgoing_call", &voip_outgoing_call, NULL);
  shttpd_register_uri(ctx, "/voip/call_established", &voip_call_established, NULL);
  shttpd_register_uri(ctx, "/voip/call_terminated", &voip_call_terminated, NULL);
  */
  shttpd_register_uri(ctx, "/hfs/upload", &show_hfs, NULL);
  shttpd_register_uri(ctx, "/atris/post", &show_atris_post, NULL);
  shttpd_handle_error(ctx, 404, show_404, NULL);

  /* Serve connections infinitely until someone kills us */
  for (;;) {
    if (shttpd_poll(ctx, 1000) < 0) {
      break;
    }
  }

  /* Probably unreached, because we will be killed by a signal */
  shttpd_fini(ctx);
  goto retry_shttpd;
}


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "shttpd");
  tinyros::init("shttpd");

  ros::NodeHandle nh;
  shttpd_req_pub = nh.advertise<atris_msgs::SignalMessage>(TOPIC_SIGNAL_REQUEST_MESSAGE, 100);
  shttpd_diag_sub = nh.subscribe(TOPIC_SHTTPD_DIAGNOSTIC_MESSAGE, 100, shttpdResponseCb);
  shttpd_res_sub = nh.subscribe(TOPIC_SIGNAL_RESPONSE_MESSAGE, 100, shttpdResponseCb);

  shm::iMemory_init();
  
  signal(SIGTERM, &signal_handler);
  signal(SIGCHLD, &signal_handler);
  signal(SIGPIPE, SIG_IGN);

  initDatabaseTable();

  std::thread thidws(std::bind(lws_main_loop));
  thidws.detach();

  std::thread thidhttp(std::bind(http_service_run, argc, argv));
  thidhttp.detach();

  ros::MultiThreadedSpinner s(10);
  ros::spin(s);

  return 0;
}
