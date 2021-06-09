///////////////////////////////////////////////////////////
var global_lws_context = null;
var global_lws_lock_reconnect = false;
var global_lws_host_reg = /\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}/;
var global_lws_host_ip = global_lws_host_reg.exec(window.location.href)[0];

///////////////////////////////////////////////////////////

function createUuid() {
    var s = [];
    var hexDigits = "0123456789abcdef";
    for (var i = 0; i < 32; i++) {
        s[i] = hexDigits.substr(Math.floor(Math.random() * 0x10), 1);
    }
    s[14] = "4"; // bits 12-15 of the time_hi_and_version field to 0010
    s[19] = hexDigits.substr((s[19] & 0x3) | 0x8, 1); // bits 6-7 of the clock_seq_hi_and_reserved to 01

    var uuid = s.join("");
    return uuid;
}
function add(x, y) {
    return((x & 0x7FFFFFFF) + (y & 0x7FFFFFFF)) ^ (x & 0x80000000) ^ (y & 0x80000000);
}

function sha1hex(num) {
    var sHEXChars = "0123456789abcdef";
    var str = "";
    for(var j = 7; j >= 0; j--)
        str += sHEXChars.charAt((num >> (j * 4)) & 0x0F);
    return str;
}

function alignSHA1(sIn) {
    var nblk = ((sIn.length + 8) >> 6) + 1,
        blks = new Array(nblk * 16);
    for(var i = 0; i < nblk * 16; i++) blks[i] = 0;
    for(i = 0; i < sIn.length; i++)
        blks[i >> 2] |= sIn.charCodeAt(i) << (24 - (i & 3) * 8);
    blks[i >> 2] |= 0x80 << (24 - (i & 3) * 8);
    blks[nblk * 16 - 1] = sIn.length * 8;
    return blks;
}

function rol(num, cnt) {
    return(num << cnt) | (num >>> (32 - cnt));
}

function ft(t, b, c, d) {
    if(t < 20) return(b & c) | ((~b) & d);
    if(t < 40) return b ^ c ^ d;
    if(t < 60) return(b & c) | (b & d) | (c & d);
    return b ^ c ^ d;
}

function kt(t) {
    return(t < 20) ? 1518500249 : (t < 40) ? 1859775393 :
        (t < 60) ? -1894007588 : -899497514;
}

function sha1(sIn) {
    var x = alignSHA1(sIn);
    var w = new Array(80);
    var a = 1732584193;
    var b = -271733879;
    var c = -1732584194;
    var d = 271733878;
    var e = -1009589776;
    var t;
    for(var i = 0; i < x.length; i += 16) {
        var olda = a;
        var oldb = b;
        var oldc = c;
        var oldd = d;
        var olde = e;
        for(var j = 0; j < 80; j++) {
            if(j < 16) w[j] = x[i + j];
            else w[j] = rol(w[j - 3] ^ w[j - 8] ^ w[j - 14] ^ w[j - 16], 1);
            t = add(add(rol(a, 5), ft(j, b, c, d)), add(add(e, w[j]), kt(j)));
            e = d;
            d = c;
            c = rol(b, 30);
            b = a;
            a = t;
        }
        a = add(a, olda);
        b = add(b, oldb);
        c = add(c, oldc);
        d = add(d, oldd);
        e = add(e, olde);
    }
    var sha1Value = sha1hex(a) + sha1hex(b) + sha1hex(c) + sha1hex(d) + sha1hex(e);
    return sha1Value.toLowerCase();
}

function set_robot_upgrade_url() {
  var filename = document.getElementById("robot_upgrade_upload_file").value;
  var pos1 = filename.lastIndexOf("\\");
  var pos2 = filename.lastIndexOf("/");
  if (pos1 != -1) {
    filename = filename.substring(pos1 + 1);
  } else if (pos2 != -1) {
    filename = filename.substring(pos2 + 1);
  }
  document.getElementById("robot_upgrade_url").value = "http://" + global_lws_host_ip + "/hfs/" + filename;
}

function set_robot_music_url() {
  var filename = document.getElementById("robot_music_upload_file").value;
  var pos1 = filename.lastIndexOf("\\");
  var pos2 = filename.lastIndexOf("/");
  if (pos1 != -1) {
    filename = filename.substring(pos1 + 1);
  } else if (pos2 != -1) {
    filename = filename.substring(pos2 + 1);
  }
  document.getElementById("robot_music_url").value = "http://" + global_lws_host_ip + "/hfs/" + filename;
}

function set_imsi_file_url()
  {
    var filename = document.getElementById("robot_upload_imsi_file").value;
    var pos1 = filename.lastIndexOf("\\");
    var pos2 = filename.lastIndexOf("/");
    if (pos1 != -1) {
      filename = filename.substring(pos1 + 1);
    } else if (pos2 != -1) {
      filename = filename.substring(pos2 + 1);
    }
    document.getElementById("robot_imsi_url").value = "http://" + global_lws_host_ip + "/hfs/" + filename;
}

function do_shttpd_global_init(content) {
  if (content.hasOwnProperty("global_ntp_server_list")) {
    document.getElementById('global_ntp_server_list').value = content["global_ntp_server_list"];
  }
  if (content.hasOwnProperty("global_volume")) {
    document.getElementById('global_volume').value = content["global_volume"];
  }
  if (content.hasOwnProperty("global_dock_wall_width")) {
    document.getElementById('global_dock_wall_width').value = content["global_dock_wall_width"];
  }
  if (content.hasOwnProperty("global_dock_1_0")) {
    document.getElementById("global_dock_wall_width").disabled = (parseInt(content["global_dock_1_0"]) != 0 ? true : false);
    document.getElementById("global_dock_1_0").selected = (parseInt(content["global_dock_1_0"]) != 0 ? true : false);
    document.getElementById("global_dock_0_0").selected = (parseInt(content["global_dock_1_0"]) == 0 ? true : false);
  }
  if (content.hasOwnProperty("global_ptz_ip")) {
    document.getElementById('global_ptz_ip').value = content["global_ptz_ip"];
  }
  if (content.hasOwnProperty("global_ptz_box_ip")) {
    document.getElementById('global_ptz_box_ip').value = content["global_ptz_box_ip"];
  }
  if (content.hasOwnProperty("global_nvr_ip")) {
    document.getElementById('global_nvr_ip').value = content["global_nvr_ip"];
  }
  if (content.hasOwnProperty("global_mqtt_host")) {
    document.getElementById('global_mqtt_host').value = content["global_mqtt_host"];
  }
  if (content.hasOwnProperty("global_mqtt_port")) {
    document.getElementById('global_mqtt_port').value = content["global_mqtt_port"];
  }
  if (content.hasOwnProperty("global_mqtt_username")) {
    document.getElementById('global_mqtt_username').value = content["global_mqtt_username"];
  }
  if (content.hasOwnProperty("global_mqtt_password")) {
    document.getElementById('global_mqtt_password').value = content["global_mqtt_password"];
  }
  if (content.hasOwnProperty("global_mqtt_encrypt")) {
    document.getElementById('global_mqtt_encrypt').value = content["global_mqtt_encrypt"];
  }
  if (content.hasOwnProperty("global_mqtt_keepalive")) {
    document.getElementById('global_mqtt_keepalive').value = content["global_mqtt_keepalive"];
  }

  if (content.hasOwnProperty("global_remote_mqtt_topic_url")) {
    document.getElementById('remote_mqtt_topic_url').value = content["global_remote_mqtt_topic_url"];
  }
  if (content.hasOwnProperty("global_remote_mqtt_host")) {
    document.getElementById('remote_mqtt_host').value = content["global_remote_mqtt_host"];
  }
  if (content.hasOwnProperty("global_remote_mqtt_port")) {
    document.getElementById('remote_mqtt_port').value = content["global_remote_mqtt_port"];
  }
  if (content.hasOwnProperty("global_remote_mqtt_username")) {
    document.getElementById('remote_mqtt_username').value = content["global_remote_mqtt_username"];
  }
  if (content.hasOwnProperty("global_remote_mqtt_password")) {
    document.getElementById('remote_mqtt_password').value = content["global_remote_mqtt_password"];
  }
  if (content.hasOwnProperty("global_remote_mqtt_encrypt")) {
    document.getElementById('remote_mqtt_encrypt').value = content["global_remote_mqtt_encrypt"];
  }
  if (content.hasOwnProperty("global_remote_mqtt_keepalive")) {
    document.getElementById('remote_mqtt_keepalive').value = content["global_remote_mqtt_keepalive"];
  }
  if (content.hasOwnProperty("global_router_type")) {
    document.getElementById("global_router_type_gosuncn").selected = (content["global_router_type"] == "gosuncn" ? true : false);
    document.getElementById("global_router_type_hongdian").selected = (content["global_router_type"] == "hongdian" ? true : false);
  }
  if (content.hasOwnProperty("global_router_ip")) {
    document.getElementById('global_router_ip').value = content["global_router_ip"];
  }
  if (content.hasOwnProperty("global_router_user")) {
    document.getElementById('global_router_user').value = content["global_router_user"];
  }
  if (content.hasOwnProperty("global_router_psw")) {
    document.getElementById('global_router_psw').value = content["global_router_psw"];
  }
  if (content.hasOwnProperty("global_abi_base_url")) {
    document.getElementById('global_abi_base_url').value = content["global_abi_base_url"];
  }
  if (content.hasOwnProperty("global_abi_robotinfo_url")) {
    document.getElementById('global_abi_robotinfo_url').value = content["global_abi_robotinfo_url"];
  }
  if (content.hasOwnProperty("global_abi_timestamp_url")) {
    document.getElementById('global_abi_timestamp_url').value = content["global_abi_timestamp_url"];
  }
  if (content.hasOwnProperty("global_abi_post_event_url")) {
    document.getElementById('global_abi_post_event_url').value = content["global_abi_post_event_url"];
  }
  if (content.hasOwnProperty("global_abi_enable")) {
    document.getElementById('global_abi_enable').value = content["global_abi_enable"];
  }
  if (content.hasOwnProperty("global_hfs_type") && content.hasOwnProperty("global_hfs_url")) {
    document.getElementById("global_hfs_url").value = content["global_hfs_url"];
    document.getElementById("global_hfs_url").disabled = (content["global_hfs_type"] == "qiniu" ? true : false);
    document.getElementById("global_hfs_type_qiniu").selected = (content["global_hfs_type"] == "qiniu" ? true : false);
    document.getElementById("global_hfs_type_hfs").selected = (content["global_hfs_type"] == "hfs" ? true : false);
  }
  if (content.hasOwnProperty("global_timezone")) {
    document.getElementById('global_timezone').value = content["global_timezone"];
  }
  if (content.hasOwnProperty("global_voip_auto_setting")) {
    document.getElementById('global_voip_auto_setting').value = content["global_voip_auto_setting"];
  }
  if (content.hasOwnProperty("global_voip_sip_server")) {
    document.getElementById('global_voip_sip_server').value = content["global_voip_sip_server"];
  }
  if (content.hasOwnProperty("global_nav_obs_mode")) {
    document.getElementById('global_nav_obs_mode').value = content["global_nav_obs_mode"];
  }
  if (content.hasOwnProperty("global_nav_cycle_enable")) {
    document.getElementById('global_nav_cycle_enable').value = content["global_nav_cycle_enable"];
  }
  if (content.hasOwnProperty("global_nav_cycle_dst")) {
    document.getElementById('global_nav_cycle_dst').value = content["global_nav_cycle_dst"];
  }
  if (content.hasOwnProperty("global_nav_without_charge_path")) {
    document.getElementById('global_nav_without_charge_path').value = content["global_nav_without_charge_path"];
  }
  if (content.hasOwnProperty("global_nav_anti_drop_enable")) {
    document.getElementById('global_nav_anti_drop_enable').value = content["global_nav_anti_drop_enable"];
  }
  if (content.hasOwnProperty("global_nav_tsp_enable")) {
    document.getElementById('global_nav_tsp_enable').value = content["global_nav_tsp_enable"];
  }
  if (content.hasOwnProperty("global_nav_without_merge_path")) {
    document.getElementById('global_nav_without_merge_path').value = content["global_nav_without_merge_path"];
  }
  if (content.hasOwnProperty("global_nav_task_start_timeout")) {
    document.getElementById('global_nav_task_start_timeout').value = content["global_nav_task_start_timeout"];
  }
  if (content.hasOwnProperty("global_nav_task_running_timeout")) {
    document.getElementById('global_nav_task_running_timeout').value = content["global_nav_task_running_timeout"];
  }
  if (content.hasOwnProperty("global_nav_auto_charge_self")) {
    document.getElementById('global_nav_auto_charge_self').value = content["global_nav_auto_charge_self"];
  }
  if (content.hasOwnProperty("global_nav_auto_charge_battery")) {
    document.getElementById('global_nav_auto_charge_battery').value = content["global_nav_auto_charge_battery"];
  }
  if (content.hasOwnProperty("global_red_blue_light_imsi_detection")) {
    document.getElementById('global_red_blue_light_imsi_detection').value = content["global_red_blue_light_imsi_detection"];
  }
  if (content.hasOwnProperty("global_vision_host")) {
    document.getElementById('global_vision_host').value = content["global_vision_host"];
  }
  if (content.hasOwnProperty("global_vision_port")) {
    document.getElementById('global_vision_port').value = content["global_vision_port"];
  }
  if (content.hasOwnProperty("global_obstacle_mode")) {
    document.getElementById('global_obstacle_mode').value = content["global_obstacle_mode"];
  }
  if (content.hasOwnProperty("global_obstacle_timeout")) {
    document.getElementById('global_obstacle_timeout').value = content["global_obstacle_timeout"];
  }
  if (content.hasOwnProperty("global_obstacle_timeout_strategy")) {
    document.getElementById('global_obstacle_timeout_strategy').value = content["global_obstacle_timeout_strategy"];
  }
}

///////////////////////////////////////////////////////////////

function lws_onopen(evt)  {
  console.log("WebSocket connected!");
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_rosshttpd_global_init",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';
  
  global_lws_context.send(jsonStr);
}

function lws_onmessage(evt)  {
  var received_msg = evt.data;
  console.log("ReceivedMsg:" + received_msg);

  var jsonObj = JSON.parse(received_msg);
  if (jsonObj.hasOwnProperty("content") && jsonObj.hasOwnProperty("title")) {
    if (jsonObj["title"] == "notify_robot_info" || jsonObj["title"] == "response_robot_info") {
      do_diagnostics(jsonObj["content"]);
    } else if (jsonObj["title"] == "response_mcb_log") {
      if (jsonObj["content"]["result"] == "success") {
        document.getElementById("robot_log_url").value = jsonObj["content"]["url"];
      } else {
        document.getElementById("robot_log_url").value = "获取日志失败: " + jsonObj["content"]["result"];
      }
    } else if (jsonObj["title"] == "notify_get_detection_imsi_file_url") {
      if (jsonObj["content"]["result"] == "success") {
        document.getElementById("robot_imsi_detection_file_url").value = jsonObj["content"]["url"];
      } else {
        document.getElementById("robot_imsi_detection_file_url").value = "获取下载地址失败: " + jsonObj["content"]["result"];
      }
    } else if (jsonObj["title"] == "response_rosshttpd_global_init") {
      do_shttpd_global_init(jsonObj["content"]);
    } else if (jsonObj["title"] == "response_rosshttpd_pwd") {
      if (jsonObj["content"]["result"] == "success") {
        window.location="http://" + global_lws_host_ip;
        alert("修改密码成功，请重新登录！");
      } else {
        alert("修改密码失败:" + jsonObj["content"]["result"]);
      }
    } else if (jsonObj["title"] == "response_ptz_capture") {
      if (jsonObj["content"]["result"] == "success") {
        document.getElementById("visible_light_url").value = jsonObj["content"]["visible_light_url"];
        document.getElementById("infrared_url").value = jsonObj["content"]["infrared_url"];
      } else {
        document.getElementById("visible_light_url").value = "获取失败: " + jsonObj["content"]["result"];
        document.getElementById("infrared_url").value = "获取失败: " + jsonObj["content"]["result"];
      }
    }else if(jsonObj["title"]=="response_ptz_record"){
      if (jsonObj["content"]["result"] == "success") {
        document.getElementById("visible_light_url").value = jsonObj["content"]["visible_light_video_url"];
        document.getElementById("infrared_url").value = jsonObj["content"]["infrared_video_url"];
      } else {
        document.getElementById("visible_light_url").value = "获取失败: " + jsonObj["content"]["result"];
        document.getElementById("infrared_url").value = "获取失败: " + jsonObj["content"]["result"];
    }}else if(jsonObj["title"]=="response_sound_record"){
      if (jsonObj["content"]["result"] == "success") {
        document.getElementById("visible_light_url").value = jsonObj["content"]["url"];
      } else {
        document.getElementById("visible_light_url").value = "获取失败: " + jsonObj["content"]["result"];
    }}else if (jsonObj["title"] == "response_ptz_param") {
      if (jsonObj["content"]["result"] == "success") {
        document.getElementById("pan").value = jsonObj["content"]["pan"];
        document.getElementById("tilt").value = jsonObj["content"]["tilt"];
        document.getElementById("zoom").value = jsonObj["content"]["zoom"];
        document.getElementById("focus").value = jsonObj["content"]["focus"];
      } else {
        document.getElementById("pan").value = "获取失败: " + jsonObj["content"]["result"];
        document.getElementById("tilt").value = "获取失败: " + jsonObj["content"]["result"];
        document.getElementById("zoom").value = "获取失败: " + jsonObj["content"]["result"];
        document.getElementById("focus").value = "获取失败: " + jsonObj["content"]["result"];
      }        
    }else if(jsonObj["title"] =="response_ptz_ir_temperature_info"){
      document.getElementById("termperature_max").value = jsonObj["content"]["temperature_max"];
      document.getElementById("termperature_min").value = jsonObj["content"]["temperature_min"];
      document.getElementById("termperature_ave").value = jsonObj["content"]["temperature_ave"];
      document.getElementById("termperature_diff").value = jsonObj["content"]["temperature_diff"];
    } else if (jsonObj["title"] == "response_sw_upgrade") {
      if (jsonObj["content"]["status"] == "started") {
        document.getElementById("upgrade_progress_textarea").value = "升级开始";
      }
      else if (jsonObj["content"]["status"] == "downloading")
      {
        if (jsonObj["content"]["result"] == "success")
        {
          var percent = parseFloat(jsonObj["content"]["progress"]);
          document.getElementById("upgrade_progress_textarea").value = "固件下载中: " + percent.toString() +" %";
        }
        else if(jsonObj["content"]["result"] == "fail_downloading")
        {
          document.getElementById("upgrade_progress_textarea").value = "固件下载失败";
        }
      }
      else if (jsonObj["content"]["status"] == "imx_upgrading")
      {
        document.getElementById("upgrade_progress_textarea").value = "主控升级结果: " + jsonObj["content"]["result"];
      }
      else if (jsonObj["content"]["status"] == "sys_monitor_start")
      {
        document.getElementById("upgrade_progress_textarea").value = "系统控制板升级开始";
      }
      else if (jsonObj["content"]["status"] == "chassis_control_start")
      {
        document.getElementById("upgrade_progress_textarea").value = "底盘控制板升级开始";
      }
      else if (jsonObj["content"]["status"] == "sys_monitor_end")
      {
        if (jsonObj["content"]["result"] == "monitor_upgrade_failed")
        {
          document.getElementById("upgrade_progress_textarea").value = "系统控制板升级失败";
        }
        else if (jsonObj["content"]["status"] == "monitor_upgrade_success")
        {
          document.getElementById("upgrade_progress_textarea").value = "系统控制板升级成功";
        }
      }
      else if (jsonObj["content"]["status"] == "chassis_control_end")
      {
        if (jsonObj["content"]["result"] == "chassis_upgrade_failed")
        {
          document.getElementById("upgrade_progress_textarea").value = "底盘控制板升级失败";
        }
        else if (jsonObj["content"]["status"] == "chassis_upgrade_success")
        {
          document.getElementById("upgrade_progress_textarea").value = "底盘控制板升级成功";
        }
      }
      else if (jsonObj["content"]["status"] == "finished" && jsonObj["content"]["result"] == "success")
      {
        document.getElementById("upgrade_progress_textarea").value = "升级成功";
      }
      else if (jsonObj["content"]["status"] == "firmware_upgrading" && jsonObj["content"]["result"] == "fail_firmware_upgrading")
      {
        document.getElementById("upgrade_progress_textarea").value = "固件升级失败";
      }
    } else {
      if (jsonObj["title"].indexOf("response_rosshttpd_cfg_") != -1 
        || jsonObj["title"] == "response_reset_robot") {
        if (jsonObj["content"]["result"] == "success") {
          alert("设置成功，请重启机器人！");
        } else {
          alert("设置失败:" + jsonObj["content"]["result"]);
        }
      }
    }
  }
}

function lws_onerror(evt) {
  console.log("WebSocket onerror!");
  if (global_lws_lock_reconnect) return;
  global_lws_lock_reconnect = true;
  setTimeout(function() {
    global_lws_context = new WebSocket("ws://" + global_lws_host_ip + ":28123");
    global_lws_context.onopen = lws_onopen;
    global_lws_context.onmessage = lws_onmessage;
    global_lws_context.onclose = lws_onclose;
    global_lws_context.onerror = lws_onerror;
    global_lws_lock_reconnect = false;
  }, 1000);
}

function lws_onclose(evt)  {
  console.log("WebSocket colosed->" + evt.code + ":" + evt.reason);
  if (evt.reason == "kickout") {
    window.location="http://" + global_lws_host_ip;
    alert("被踢下线了，请重新登录！");
  } else {
    if (global_lws_lock_reconnect) return;
    global_lws_lock_reconnect = true;
    setTimeout(function() {
      global_lws_context = new WebSocket("ws://" + global_lws_host_ip + ":28123");
      global_lws_context.onopen = lws_onopen;
      global_lws_context.onmessage = lws_onmessage;
      global_lws_context.onclose = lws_onclose;
      global_lws_context.onerror = lws_onerror;
      global_lws_lock_reconnect = false;
    }, 1000);
  }
}

global_lws_context = new WebSocket("ws://" + global_lws_host_ip + ":28123");
                
global_lws_context.onopen = lws_onopen;

global_lws_context.onmessage = lws_onmessage;

global_lws_context.onclose = lws_onclose;

global_lws_context.onerror = lws_onerror;

////////////////////////////////////////////////////////////

function request_set_robot_transport_mode(on_off) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_robot_transport_mode",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "switch": '+on_off+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_robot_mode_standby() {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_robot_standby_mode",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    
    global_lws_context.send(jsonStr);
}

function request_robot_wake_up_from_standby() {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_robot_wake_up",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    
    global_lws_context.send(jsonStr);
}


function request_set_indicator_light_style(style, text) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_set_indicator_style",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "style": '+ style +'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    
    global_lws_context.send(jsonStr);
}

function request_set_indicator_light_color(color, text) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_set_indicator_color",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "color": '+ color +'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    
    global_lws_context.send(jsonStr);
}

function request_set_indicator_brightness(brightness, text) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_set_indicator_brightness",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "brightness": '+ brightness +'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    
    global_lws_context.send(jsonStr);
}

function request_release_indicator_light_control() {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_release_indicator_light_control",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    
    global_lws_context.send(jsonStr);
}

function request_set_robot_event_simulation(on_off){
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_set_robot_event_simulation",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "on_off": '+ on_off +'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    
    global_lws_context.send(jsonStr);
}

function request_set_robot_warning(type, on_off){
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_set_robot_warning",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "type": '+type+',\n';
    jsonStr += '        "on_off": '+ on_off +'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    
    global_lws_context.send(jsonStr);
}

function request_send_robot_in_position(result) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_send_robot_in_position",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "action": '+ result +'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    
    global_lws_context.send(jsonStr);
}

function request_send_robot_leave_pile() {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_send_leave_pile",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    
    global_lws_context.send(jsonStr);
}


function request_switch_light(lamp, on_off) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_switch_light",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "lamp": '+ lamp +',\n';
    jsonStr += '        "switch": '+ on_off +'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    
    global_lws_context.send(jsonStr);
}

function request_sonic_disperse(on_off) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_sonic_disperse",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "switch": '+ on_off +'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_music_play(action) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_music_play",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "action": "'+action+'"\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_roshttpd_music_play() {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_roshttpd_music_play",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_set_speed(speed) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_set_speed",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "speed": '+ speed +'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_set_volume(diff) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var volume = parseInt(document.getElementById('global_volume').value) + diff;
    if (volume >= 100) volume = 100;
    if (volume <= 0) volume = 0;
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_set_volume",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "volume": '+volume+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
    document.getElementById('global_volume').value = volume;
}

function request_set_mute(muted) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_set_mute",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "muted": '+muted+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_emergency_stop(on_off) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_emergency_stop",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "switch": '+on_off+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_start_patrol() {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_switch_patrol",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "startTime": 0,\n';
    jsonStr += '        "patrolInterval": 10,\n';
    jsonStr += '        "patrolCount": 9999,\n';
    jsonStr += '        "switch": 1\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_nav_switch_with_shttpd(state) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_nav_switch_with_shttpd",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "switch": '+ state +'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_nav_return(state) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_return",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "switch": '+ state +'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_nav_recharge(state) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_recharge",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "switch": '+ state +'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_nav_leave_pile(state) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_leave_pile",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "switch": '+ state +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}

function request_robot_move_control(op) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_robot_move_control",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "operation": '+ op +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}


function request_start_rotate_camera() {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_start_rotate_camera",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "interval": 3000,\n';
    jsonStr += '        "verticalangel": 15,\n';
    jsonStr += '        "horizontalangel": 15,\n';
    jsonStr += '        "defaultpoint": 1\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_stop_rotate_camera() {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_stop_rotate_camera",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_ptz_move_control(direction) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_ptz_move_control",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "direction": '+ direction +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}

function request_ptz_capture() {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_ptz_capture",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}

function request_ptz_record(on_off) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_ptz_record",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "switch": '+ on_off +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}


function request_sound_record(on_off) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_sound_record",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "switch": '+ on_off +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}

function request_ptz_focus_control(action) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_ptz_focus_control",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "operation": '+ action +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}

function request_ptz_zoom_control(action) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_ptz_zoom_control",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "operation": '+ action +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}

function request_switch_ptz_wiper(on_off) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_switch_ptz_wiper",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "switch": '+ on_off +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}


function request_switch_ptz_light(on_off) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_switch_ptz_light",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "switch": '+ on_off +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}

function request_ptz_ir_temperature_info(start_point_x1,start_point_y1,end_point_x2,end_point_y2){

  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr  = '{\n';
      jsonStr += '    "title": "request_ptz_ir_temperature_info",\n';
      jsonStr += '    "accid": "admin",\n';
      jsonStr += '    "content":\n';
              jsonStr += '    {\n';
              jsonStr += '        "id": "'+id+'",\n';
              jsonStr += '        "timestamp": '+timestamp+',\n';
              jsonStr += '        "start_point":{\n';
              jsonStr += '                  "x": '+ start_point_x1 +',\n';
              jsonStr += '                  "y": '+ start_point_y1 +'\n';
              jsonStr += '         },\n';
              jsonStr += '        "end_point":{\n';
              jsonStr += '                  "x": '+ end_point_x2 +',\n';
              jsonStr += '                  "y": '+ end_point_y2 +'\n';
              jsonStr += '        }\n';
              jsonStr += '    }\n';
      jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}


function direction_type_select(direction,angle) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_ptz_hv_angle",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "direction": '+ direction +',\n';
  jsonStr += '        "angle": '+ angle +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}


function request_ptz_zoom(zoom) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_ptz_zoom_value",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "zoom": '+ zoom +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}



function request_ptz_ptzf(on_off) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_ptz_param",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "switch": '+ on_off +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}





function request_ptz_speed(h_speed,v_speed) {
  console.log("damon");
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_ptz_hv_speed",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "h_speed": '+ h_speed +',\n';
  jsonStr += '        "v_speed": '+ v_speed +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}


function changeZoom(value) {
  console.log(value.value);
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_ptz_zoom_value",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "zoom": '+ value +'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}


function request_set_fan_middle(speed) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_set_fan_middle",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "speed": '+speed+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_set_fan_bottom(speed) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_set_fan_bottom",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "speed": '+speed+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_mcb_log() {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_mcb_log",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_reset_robot(clear) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var password = sha1("123456");
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_reset_robot",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "token": "",\n';
    jsonStr += '        "bindmode": 1,\n';
    jsonStr += '        "clear": '+clear+',\n';
    jsonStr += '        "account": "admin",\n';
    jsonStr += '        "pwd": "'+password+'"\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_sw_upgrade(url) {
    var id = createUuid();
    var logserial = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_sw_upgrade",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "log_serial": "'+logserial+'",\n';
    jsonStr += '        "url": "'+url.replace(/(^\s*)|(\s*$)/g, "")+'"\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_joystick_move(v_linear, v_angular) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_joystick_move",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "v_linear": '+v_linear+',\n';
    jsonStr += '        "v_angular": '+v_angular+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    if (document.getElementById("openjoystick").checked) {
      global_lws_context.send(jsonStr);
    }
}

function request_rosshttpd_pwd(pwd_old, pwd_new) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_rosshttpd_pwd",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "old": "'+pwd_old+'",\n';
    jsonStr += '        "new": "'+pwd_new+'"\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_set_map(map_name) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_set_map",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "name": "'+map_name+'"\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';

  global_lws_context.send(jsonStr);
}

function request_set_tts_language(language) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_set_tts_language",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "language": '+language+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_set_tts_speaker(speaker) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_set_tts_speaker",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "speaker": '+speaker+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_tts_enable(enable) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_tts_enable",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "enable": '+enable+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_tts_play_stop() {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_tts_play_stop",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_tts_play_start(text) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_tts_play_start",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "text": "'+text+'",\n';
    jsonStr += '        "loop": 1,\n';
    jsonStr += '        "interval": 10\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
}

function request_robot_info() {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_robot_info",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
}

function request_rosshttpd_cfg_devices_ip(ptz_ip, ptz_box_ip, nvr_ip) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_rosshttpd_cfg_devices_ip",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "ptz_ip": "'+ptz_ip+'",\n';
    jsonStr += '        "ptz_box_ip": "'+ptz_box_ip+'",\n';
    jsonStr += '        "nvr_ip": "'+nvr_ip+'"\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
}

function onchange_dock_type(value) {
  if (value != "0") {
    document.getElementById("global_dock_wall_width").disabled = true;
  } else {
    document.getElementById("global_dock_wall_width").disabled = false;
  }
}

function request_rosshttpd_cfg_udock(dock_wall_width, dock_1_0) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_rosshttpd_cfg_udock",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "dock_wall_width": '+dock_wall_width+',\n';
    jsonStr += '        "dock_1_0": '+dock_1_0+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
}

function request_rosshttpd_cfg_mqtt(host, port, username, password, encrypt, keepalive) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_rosshttpd_cfg_mqtt",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "host": "'+host+'",\n';
    jsonStr += '        "port": '+port+',\n';
    jsonStr += '        "username": "'+username+'",\n';
    jsonStr += '        "password": "'+password+'",\n';
    jsonStr += '        "encrypt": '+encrypt+',\n';
    jsonStr += '        "keepalive": '+keepalive+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
}

function request_rosshttpd_cfg_remote_mqtt(topic_url, host, port, username, password, encrypt, keepalive) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_rosshttpd_cfg_remote_mqtt",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "topic_url": "'+topic_url+'",\n';
  jsonStr += '        "host": "'+host+'",\n';
  jsonStr += '        "port": '+port+',\n';
  jsonStr += '        "username": "'+username+'",\n';
  jsonStr += '        "password": "'+password+'",\n';
  jsonStr += '        "encrypt": '+encrypt+',\n';
  jsonStr += '        "keepalive": '+keepalive+'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';
  global_lws_context.send(jsonStr);
}

function request_rosshttpd_cfg_router(type, ip, username, password) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_rosshttpd_cfg_router",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "type": "'+type+'",\n';
    jsonStr += '        "ip": "'+ip+'",\n';
    jsonStr += '        "username": "'+username+'",\n';
    jsonStr += '        "password": "'+password+'"\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
}

function request_rosshttpd_cfg_ntp(ntp_server, timezone) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_rosshttpd_cfg_ntp",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "ntp_server": "'+ntp_server+'",\n';
    jsonStr += '        "timezone": '+timezone+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
}

function request_rosshttpd_cfg_default() {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_rosshttpd_cfg_default",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
}

function onchange_hfs_type(value) {
  if (value != "qiniu") {
    document.getElementById("global_hfs_url").disabled = false;
  } else {
    document.getElementById("global_hfs_url").disabled = true;
  }
}

function request_rosshttpd_cfg_hfs(hfs_type, hfs_url) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_rosshttpd_cfg_hfs",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "hfs_type": "'+hfs_type+'",\n';
    jsonStr += '        "hfs_url": "'+hfs_url+'"\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
}

function request_rosshttpd_cfg_abi(base_url, robotinfo_url, timestamp_url, event_url, abi_enable) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_rosshttpd_cfg_abi",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "base_url": "'+base_url+'",\n';
    jsonStr += '        "timestamp_url": "'+timestamp_url+'",\n';
    jsonStr += '        "event_url": "'+event_url+'",\n';
    jsonStr += '        "abi_enable": '+abi_enable+',\n';
    jsonStr += '        "robotinfo_url": "'+robotinfo_url+'"\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
}

function request_rosshttpd_cfg_voip_params(auto_setting, sip_server) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_rosshttpd_cfg_voip_params",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "auto_setting": '+auto_setting+',\n';
    jsonStr += '        "sip_server": "'+sip_server+'"\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
}

function request_rosshttpd_cfg_nav(obs_mode, cycle_dst, cycle_enable, 
                  without_charge_path, task_start_timeout, task_running_timeout, 
                  without_merge_path, auto_charge_self, auto_charge_battery, anti_drop_enable, tsp_enable) {
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_rosshttpd_cfg_nav",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "obs_mode": '+obs_mode+',\n';
    jsonStr += '        "cycle_enable": '+cycle_enable+',\n';
    jsonStr += '        "cycle_dst": "'+cycle_dst+'",\n';
    jsonStr += '        "without_charge_path": '+without_charge_path+',\n';
    jsonStr += '        "task_start_timeout": '+task_start_timeout+',\n';    
    jsonStr += '        "task_running_timeout": '+task_running_timeout+',\n';    
    jsonStr += '        "without_merge_path": '+without_merge_path+',\n';
    jsonStr += '        "auto_charge_self": '+auto_charge_self+',\n';
    jsonStr += '        "auto_charge_battery": '+auto_charge_battery+',\n';
    jsonStr += '        "anti_drop_enable": '+anti_drop_enable+',\n';
    jsonStr += '        "tsp_enable": '+tsp_enable+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
}

function request_rosshttpd_cfg_vision(host, port) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_rosshttpd_cfg_vision",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "vision_host": "'+host+'",\n';
  jsonStr += '        "vision_port": '+port+'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';
  global_lws_context.send(jsonStr);
}

function request_rosshttpd_cfg_obstacle(mode, timeout, timeout_stratagy) {
  var id = createUuid();
  var timestamp = new Date().getTime();
  var jsonStr = '{\n';
  jsonStr += '    "title": "request_rosshttpd_cfg_obstacle",\n';
  jsonStr += '    "accid": "admin",\n';
  jsonStr += '    "content":\n';
  jsonStr += '    {\n';
  jsonStr += '        "id": "'+id+'",\n';
  jsonStr += '        "timestamp": '+timestamp+',\n';
  jsonStr += '        "obstacle_mode": '+mode+',\n';
  jsonStr += '        "obstacle_timeout": '+timeout+',\n';
  jsonStr += '        "obstacle_timeout_strategy": '+timeout_stratagy+'\n';
  jsonStr += '    }\n';
  jsonStr += '}\n';
  global_lws_context.send(jsonStr);
}

function request_rosshttpd_cfg_open_light_imsi_detection(open_light)
{
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_rosshttpd_cfg_open_light_imsi_detection",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+',\n';
    jsonStr += '        "open_light_imsi_detection": '+open_light+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';
    global_lws_context.send(jsonStr);
}

function request_music_transport(url) {
    if (url.length > 0) {
      var id = createUuid();
      var timestamp = new Date().getTime();
      var jsonStr = '{\n';
      jsonStr += '    "title": "request_music_transport",\n';
      jsonStr += '    "accid": "admin",\n';
      jsonStr += '    "content":\n';
      jsonStr += '    {\n';
      jsonStr += '        "id": "'+id+'",\n';
      jsonStr += '        "timestamp": '+timestamp+',\n';
      jsonStr += '        "list": [\n';
      jsonStr += '            {\n';
      jsonStr += '                "name": "'+ url.substring(url.lastIndexOf("/") + 1) +'",\n';
      jsonStr += '                "url": "'+ url +'"\n';
      jsonStr += '            }\n';
      jsonStr += '        ]\n';
      jsonStr += '    }\n';
      jsonStr += '}\n';
      global_lws_context.send(jsonStr);
  }
}

function request_imsi_file_transport(url)
{
    if (url.length > 0) {
        var id = createUuid();
        var timestamp = new Date().getTime();
        var jsonStr = '{\n';
        jsonStr += '    "title": "request_imsi_file_transport",\n';
        jsonStr += '    "accid": "admin",\n';
        jsonStr += '    "content":\n';
        jsonStr += '    {\n';
        jsonStr += '        "id": "'+id+'",\n';
        jsonStr += '        "timestamp": '+timestamp+',\n';
        jsonStr += '        "list": [\n';
        jsonStr += '            {\n';
        jsonStr += '                "name": "'+ url.substring(url.lastIndexOf("/") + 1) +'",\n';
        jsonStr += '                "url": "'+ url +'"\n';
        jsonStr += '            }\n';
        jsonStr += '        ]\n';
        jsonStr += '    }\n';
        jsonStr += '}\n';
        global_lws_context.send(jsonStr);
    } else {
        console.log("url is error\n");
    }
}

function request_get_detection_imsi_file_url()
{
    var id = createUuid();
    var timestamp = new Date().getTime();
    var jsonStr = '{\n';
    jsonStr += '    "title": "request_get_detection_imsi_file_url",\n';
    jsonStr += '    "accid": "admin",\n';
    jsonStr += '    "content":\n';
    jsonStr += '    {\n';
    jsonStr += '        "id": "'+id+'",\n';
    jsonStr += '        "timestamp": '+timestamp+'\n';
    jsonStr += '    }\n';
    jsonStr += '}\n';

    global_lws_context.send(jsonStr);
    console.log("1111222\n");
}
