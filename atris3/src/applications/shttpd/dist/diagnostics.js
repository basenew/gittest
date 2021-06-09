function do_robot_info_base(robot_info) {
  if (robot_info.hasOwnProperty("base")) {
    var base = robot_info["base"];
    if (base.hasOwnProperty("sn")) {
      document.getElementById("robot_info_base_sn").innerText = "- 序列号: " + base["sn"];
      document.getElementById("robot_info_base_sn").style.display = 'block';
    } else {
      document.getElementById("robot_info_base_sn").style.display = 'none';
    }
    if (base.hasOwnProperty("binded")) {
      document.getElementById("robot_info_base_binded").innerText = "- 绑定者: " + base["binded"];
      document.getElementById("robot_info_base_binded").style.display = 'block';
    } else {
      document.getElementById("robot_info_base_binded").style.display = 'none';
    }
    if (base.hasOwnProperty("company"))  {
      document.getElementById("robot_info_base_company").innerText = "- 企业: " + base["company"];
      document.getElementById("robot_info_base_company").style.display = 'block';
    } else {
      document.getElementById("robot_info_base_company").style.display = 'none';
    }
    document.getElementById("robot_info_base").style.display = 'block';
  }
}

function do_robot_info_temp_humi(robot_info) {
  if (robot_info.hasOwnProperty("temp_humi")) {
    var temp_humi = robot_info["temp_humi"];
    if (temp_humi.hasOwnProperty("12v")) {
      document.getElementById("robot_info_temp_humi_12v").innerText = "- 12V供电模块温度: " + temp_humi["12v"].toFixed(1) + " °C";
      document.getElementById("robot_info_temp_humi_12v").style.display = 'block';
    } else {
      document.getElementById("robot_info_temp_humi_12v").style.display = 'none';
    }
    if (temp_humi.hasOwnProperty("24v")) {
      document.getElementById("robot_info_temp_humi_24v").innerText = "- 24V供电模块温度: " + temp_humi["24v"].toFixed(1) + " °C";
      document.getElementById("robot_info_temp_humi_24v").style.display = 'block';
    } else {
      document.getElementById("robot_info_temp_humi_24v").style.display = 'none';
    }
    if (temp_humi.hasOwnProperty("humi_env")) {
      document.getElementById("robot_info_temp_humi_humi_env").innerText = "- 环境湿度: " + temp_humi["humi_env"].toFixed(1) + " %Rh";
      document.getElementById("robot_info_temp_humi_humi_env").style.display = 'block';
    } else {
      document.getElementById("robot_info_temp_humi_humi_env").style.display = 'none';
    }
    if (temp_humi.hasOwnProperty("temp_env")) {
      document.getElementById("robot_info_temp_humi_temp_env").innerText = "- 环境温度: " + temp_humi["temp_env"].toFixed(1) + " °C";
      document.getElementById("robot_info_temp_humi_temp_env").style.display = 'block';
    } else {
      document.getElementById("robot_info_temp_humi_temp_env").style.display = 'none';
    }
    if (temp_humi.hasOwnProperty("motor_left")) {
      document.getElementById("robot_info_temp_humi_motor_left").innerText = "- 左电机温度: " + temp_humi["motor_left"].toFixed(1) + " °C";
      document.getElementById("robot_info_temp_humi_motor_left").style.display = 'block';
    } else {
      document.getElementById("robot_info_temp_humi_motor_left").style.display = 'none';
    }
    if (temp_humi.hasOwnProperty("motor_right")) {
      document.getElementById("robot_info_temp_humi_motor_right").innerText = "- 右电机温度: " + temp_humi["motor_right"].toFixed(1) + " °C";
      document.getElementById("robot_info_temp_humi_motor_right").style.display = 'block';
    } else {
      document.getElementById("robot_info_temp_humi_motor_right").style.display = 'none';
    }
    if (temp_humi.hasOwnProperty("imx")) {
      document.getElementById("robot_info_temp_humi_imx").innerText = "- 主控板温度: " + temp_humi["imx"].toFixed(1) + " °C";
      document.getElementById("robot_info_temp_humi_imx").style.display = 'block';
    } else {
      document.getElementById("robot_info_temp_humi_imx").style.display = 'none';
    }
    if (temp_humi.hasOwnProperty("kuangshi")) {
      document.getElementById("robot_info_temp_humi_kuangshi").innerText = "- 云台盒子温度: " + temp_humi["kuangshi"].toFixed(1) + " °C";
      document.getElementById("robot_info_temp_humi_kuangshi").style.display = 'block';
    } else {
      document.getElementById("robot_info_temp_humi_kuangshi").style.display = 'none';
    }
    if (temp_humi.hasOwnProperty("gaussian")) {
      document.getElementById("robot_info_temp_humi_gaussian").innerText = "- 导航盒子温度: " + temp_humi["gaussian"].toFixed(1) + " °C";
      document.getElementById("robot_info_temp_humi_gaussian").style.display = 'block';
    } else {
      document.getElementById("robot_info_temp_humi_gaussian").style.display = 'none';
    }
    document.getElementById("robot_info_temp_humi").style.display = 'block';
  } else {
    document.getElementById("robot_info_temp_humi").style.display = 'none';
  }
}

function do_robot_info_chassis_driver(robot_info) {
  if (robot_info.hasOwnProperty("chassis_driver")) {
    var chassis_driver = robot_info["chassis_driver"];
    if (chassis_driver.hasOwnProperty("error")) {
      var error = parseInt(chassis_driver["error"]);
      var chassis_driver_error = "- 驱动机诊断: 正常";
      if (error != 0) {
        chassis_driver_error = "- 驱动机诊断: 异常";
        chassis_driver_error+= "(错误码: 0x" + error.toString(16).toUpperCase() + ")";
      }
      document.getElementById("robot_info_chassis_driver_error").innerText = chassis_driver_error;
      document.getElementById("robot_info_chassis_driver_error").style.display = 'block';
    } else {
      document.getElementById("robot_info_chassis_driver_error").style.display = 'none';
    }
    if (chassis_driver.hasOwnProperty("error1")) {
      var error1 = parseInt(chassis_driver["error1"]);
      var chassis_driver_error1 = "- 转向机诊断: 正常";
      if (error1 != 0) {
        chassis_driver_error1 = "- 转向机诊断: 异常";
        chassis_driver_error1+= "(错误码: 0x" + error1.toString(16).toUpperCase() + ")";
      }
      document.getElementById("robot_info_chassis_driver_error1").innerText = chassis_driver_error1;
      document.getElementById("robot_info_chassis_driver_error1").style.display = 'block';
    } else {
      document.getElementById("robot_info_chassis_driver_error1").style.display = 'none';
    }
    if (chassis_driver.hasOwnProperty("type")) {
      document.getElementById("robot_info_chassis_driver_type").innerText = "- 底盘类型: " + chassis_driver["type"];
      document.getElementById("robot_info_chassis_driver_type").style.display = 'block';
    } else {
      document.getElementById("robot_info_chassis_driver_type").style.display = 'none';
    }
    if (chassis_driver.hasOwnProperty("bat_current")) {
      document.getElementById("robot_info_chassis_driver_bat_current").innerText = "- 电池电流: " + chassis_driver["bat_current"];
      document.getElementById("robot_info_chassis_driver_bat_current").style.display = 'block';
    } else {
      document.getElementById("robot_info_chassis_driver_bat_current").style.display = 'none';
    }
    if (chassis_driver.hasOwnProperty("bat_voltage")) {
      document.getElementById("robot_info_chassis_driver_bat_voltage").innerText = "- 电池电压: " + chassis_driver["bat_voltage"];
      document.getElementById("robot_info_chassis_driver_bat_voltage").style.display = 'block';
    } else {
      document.getElementById("robot_info_chassis_driver_bat_voltage").style.display = 'none';
    }
    if (chassis_driver.hasOwnProperty("motor_current")) {
      document.getElementById("robot_info_chassis_driver_motor_current").innerText = "- 电机电流: " + chassis_driver["motor_current"];
      document.getElementById("robot_info_chassis_driver_motor_current").style.display = 'block';
    } else {
      document.getElementById("robot_info_chassis_driver_motor_current").style.display = 'none';
    }
    if (chassis_driver.hasOwnProperty("temp_ic")) {
      document.getElementById("robot_info_chassis_driver_temp_ic").innerText = "- 主芯片温度: " + chassis_driver["temp_ic"];
      document.getElementById("robot_info_chassis_driver_temp_ic").style.display = 'block';
    } else {
      document.getElementById("robot_info_chassis_driver_temp_ic").style.display = 'none';
    }
    if (chassis_driver.hasOwnProperty("temp_motor_left")) {
      document.getElementById("robot_info_chassis_driver_temp_motor_left").innerText = "- 左电机温度: " + chassis_driver["temp_motor_left"];
      document.getElementById("robot_info_chassis_driver_temp_motor_left").style.display = 'block';
    } else {
      document.getElementById("robot_info_chassis_driver_temp_motor_left").style.display = 'none';
    }
    if (chassis_driver.hasOwnProperty("temp_motor_right")) {
      document.getElementById("robot_info_chassis_driver_temp_motor_right").innerText = "- 右电机温度: " + chassis_driver["temp_motor_right"];
      document.getElementById("robot_info_chassis_driver_temp_motor_right").style.display = 'block';
    } else {
      document.getElementById("robot_info_chassis_driver_temp_motor_right").style.display = 'none';
    }
    document.getElementById("robot_info_chassis_driver").style.display = 'block';
  } else {
    document.getElementById("robot_info_chassis_driver").style.display = 'none';
  }
}

function do_robot_info_battery(robot_info) {
  if (robot_info.hasOwnProperty("battery")) {
    var battery = robot_info["battery"];
    if (battery.hasOwnProperty("level")) {
      document.getElementById("robot_info_battery_level").innerText = "- 电池电量: " + battery["level"] + "%";
      document.getElementById("robot_info_battery_level").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_level").style.display = 'none';
    }
    if (battery.hasOwnProperty("status")) {
      var status = parseInt(battery["status"]);
      var battery_status = "- 电池状态: 未知";
      if (status == 0) {
          battery_status = "- 电池状态: 空闲";
      } else if (status == 1) {
          battery_status = "- 电池状态: 充电";
      } else if (status == 2) {
          battery_status = "- 电池状态: 放电";
      } else if (status == 4) {
          battery_status = "- 电池状态: 充满";
      }
      document.getElementById("robot_info_battery_status").innerText = battery_status;
      document.getElementById("robot_info_battery_status").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_status").style.display = 'none';
    }
    if (battery.hasOwnProperty("current")) {
      document.getElementById("robot_info_battery_current").innerText = "- 电池电流: " + battery["current"].toFixed(1);
      document.getElementById("robot_info_battery_current").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_current").style.display = 'none';
    }
    if (battery.hasOwnProperty("voltage")) {
      document.getElementById("robot_info_battery_voltage").innerText = "- 电池电压: " + battery["voltage"].toFixed(1);
      document.getElementById("robot_info_battery_voltage").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_voltage").style.display = 'none';
    }
    if (battery.hasOwnProperty("temp_max")) {
      document.getElementById("robot_info_battery_temp_max").innerText = "- 最高温度: " + battery["temp_max"];
      document.getElementById("robot_info_battery_temp_max").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_temp_max").style.display = 'none';
    }
    if (battery.hasOwnProperty("temp_min")) {
      document.getElementById("robot_info_battery_temp_min").innerText = "- 最低温度: " + battery["temp_min"];
      document.getElementById("robot_info_battery_temp_min").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_temp_min").style.display = 'none';
    }
    if (battery.hasOwnProperty("charge_cnt")) {
      document.getElementById("robot_info_battery_charge_cnt").innerText = "- 充电次数: " + battery["charge_cnt"];
      document.getElementById("robot_info_battery_charge_cnt").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_charge_cnt").style.display = 'none';
    }
    if (battery.hasOwnProperty("discharge_cnt")) {
      document.getElementById("robot_info_battery_discharge_cnt").innerText = "- 放电次数: " + battery["discharge_cnt"];
      document.getElementById("robot_info_battery_discharge_cnt").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_discharge_cnt").style.display = 'none';
    }
    if (battery.hasOwnProperty("health")) {
      document.getElementById("robot_info_battery_health").innerText = "- 健康指数: " + battery["health"] + "%";
      document.getElementById("robot_info_battery_health").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_health").style.display = 'none';
    }
    if (battery.hasOwnProperty("bat_num")) {
      document.getElementById("robot_info_battery_bat_num").innerText = "- 电池串数: " + battery["bat_num"];
      document.getElementById("robot_info_battery_bat_num").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_bat_num").style.display = 'none';
    }
    if (battery.hasOwnProperty("relay_status")) {
      var relay_status = parseInt(battery["relay_status"]);
      document.getElementById("robot_info_battery_relay_status_bit0").innerText = ((((relay_status >> 0) & 1) != 0) ? "- 总负极继电器状态: 断开" : "- 总负极继电器状态: 吸合");
      document.getElementById("robot_info_battery_relay_status_bit1").innerText = ((((relay_status >> 1) & 1) != 0) ? "- 放电继电器状态: 断开" : "- 放电继电器状态: 吸合");
      document.getElementById("robot_info_battery_relay_status_bit2").innerText = ((((relay_status >> 2) & 1) != 0) ? "- 充电继电器状态: 断开" : "- 充电继电器状态: 吸合");
      document.getElementById("robot_info_battery_relay_status_bit3").innerText = ((((relay_status >> 3) & 1) != 0) ? "- 预充继电器状态: 断开" : "- 预充继电器状态: 吸合");
      document.getElementById("robot_info_battery_relay_status_bit4").innerText = ((((relay_status >> 4) & 1) != 0) ? "- DC继电器状态: 断开" : "- DC继电器状态: 吸合");
      document.getElementById("robot_info_battery_relay_status_bit5").innerText = ((((relay_status >> 5) & 1) != 0) ? "- 加热继电器状态: 断开" : "- 加热继电器状态: 吸合");
      document.getElementById("robot_info_battery_relay_status_bit0").style.display = 'block';
      document.getElementById("robot_info_battery_relay_status_bit1").style.display = 'block';
      document.getElementById("robot_info_battery_relay_status_bit2").style.display = 'block';
      document.getElementById("robot_info_battery_relay_status_bit3").style.display = 'block';
      document.getElementById("robot_info_battery_relay_status_bit4").style.display = 'block';
      document.getElementById("robot_info_battery_relay_status_bit5").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_relay_status_bit0").style.display = 'none';
      document.getElementById("robot_info_battery_relay_status_bit1").style.display = 'none';
      document.getElementById("robot_info_battery_relay_status_bit2").style.display = 'none';
      document.getElementById("robot_info_battery_relay_status_bit3").style.display = 'none';
      document.getElementById("robot_info_battery_relay_status_bit4").style.display = 'none';
      document.getElementById("robot_info_battery_relay_status_bit5").style.display = 'none';
    }
    if (battery.hasOwnProperty("charge_status")) {
      var charge_status = parseInt(battery["charge_status"]);
      document.getElementById("robot_info_battery_charge_status_bit0").innerText = ((((charge_status >> 0) & 1) != 0) ? "- 充电单体电压过高故障: 故障" : "- 充电单体电压过高故障: 无");
      document.getElementById("robot_info_battery_charge_status_bit1").innerText = ((((charge_status >> 1) & 1) != 0) ? "- 充电总电压过高故障: 故障" : "- 充电总电压过高故障: 无");
      document.getElementById("robot_info_battery_charge_status_bit2").innerText = ((((charge_status >> 2) & 1) != 0) ? "- 充电温度过高故障: 故障" : "- 充电温度过高故障: 无");
      document.getElementById("robot_info_battery_charge_status_bit3").innerText = ((((charge_status >> 3) & 1) != 0) ? "- 充电电流故障: 故障" : "- 充电电流故障: 无");
      document.getElementById("robot_info_battery_charge_status_bit4").innerText = ((((charge_status >> 4) & 1) != 0) ? "- 充电温度过低故障: 故障" : "- 充电温度过低故障: 无");
      document.getElementById("robot_info_battery_charge_status_bit0").style.display = 'block';
      document.getElementById("robot_info_battery_charge_status_bit1").style.display = 'block';
      document.getElementById("robot_info_battery_charge_status_bit2").style.display = 'block';
      document.getElementById("robot_info_battery_charge_status_bit3").style.display = 'block';
      document.getElementById("robot_info_battery_charge_status_bit4").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_charge_status_bit0").style.display = 'none';
      document.getElementById("robot_info_battery_charge_status_bit1").style.display = 'none';
      document.getElementById("robot_info_battery_charge_status_bit2").style.display = 'none';
      document.getElementById("robot_info_battery_charge_status_bit3").style.display = 'none';
      document.getElementById("robot_info_battery_charge_status_bit4").style.display = 'none';
    }
    if (battery.hasOwnProperty("discharge_status")) {
      var discharge_status = parseInt(battery["discharge_status"]);
      document.getElementById("robot_info_battery_discharge_status_bit0").innerText = ((((discharge_status >> 0) & 1) != 0) ? "- 充电总电压过低故障: 故障" : "- 充电总电压过低故障: 无");
      document.getElementById("robot_info_battery_discharge_status_bit1").innerText = ((((discharge_status >> 1) & 1) != 0) ? "- 充电单体电压过低故障: 故障" : "- 充电单体电压过低故障: 无");
      document.getElementById("robot_info_battery_discharge_status_bit2").innerText = ((((discharge_status >> 2) & 1) != 0) ? "- 放电温度过低故障: 故障" : "- 放电温度过低故障: 无");
      document.getElementById("robot_info_battery_discharge_status_bit3").innerText = ((((discharge_status >> 3) & 1) != 0) ? "- 放电电流故障: 故障" : "- 放电电流故障: 无");
      document.getElementById("robot_info_battery_discharge_status_bit4").innerText = ((((discharge_status >> 4) & 1) != 0) ? "- 放电温度过高故障: 故障" : "- 放电温度过高故障: 无");
      document.getElementById("robot_info_battery_discharge_status_bit0").style.display = 'block';
      document.getElementById("robot_info_battery_discharge_status_bit1").style.display = 'block';
      document.getElementById("robot_info_battery_discharge_status_bit2").style.display = 'block';
      document.getElementById("robot_info_battery_discharge_status_bit3").style.display = 'block';
      document.getElementById("robot_info_battery_discharge_status_bit4").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_discharge_status_bit0").style.display = 'none';
      document.getElementById("robot_info_battery_discharge_status_bit1").style.display = 'none';
      document.getElementById("robot_info_battery_discharge_status_bit2").style.display = 'none';
      document.getElementById("robot_info_battery_discharge_status_bit3").style.display = 'none';
      document.getElementById("robot_info_battery_discharge_status_bit4").style.display = 'none';
    }
    if (battery.hasOwnProperty("vstatus")) {
      var vstatus = parseInt(battery["vstatus"]);
      var vstatus01 = ((vstatus >> 0) & 3);
      var vstatustext01 = "- 单体过压故障: 无";
      if (vstatus01 == 1) {
        vstatustext01 = "- 单体过压故障: 严重";
      } else if (vstatus01 == 2) {
        vstatustext01 = "- 单体过压故障: 轻微";
      }
      var vstatus23 = ((vstatus >> 2) & 3);
      var vstatustext23 = "- 单体欠压故障: 无";
      if (vstatus23 == 1) {
        vstatustext23 = "- 单体欠压故障: 严重";
      } else if (vstatus23 == 2) {
        vstatustext23 = "- 单体欠压故障: 轻微";
      }
      var vstatus45 = ((vstatus >> 4) & 3);
      var vstatustext45 = "- 单体压差故障: 无";
      if (vstatus45 == 1) {
        vstatustext45 = "- 单体压差故障: 严重";
      } else if (vstatus45 == 2) {
        vstatustext45 = "- 单体压差故障: 轻微";
      }
      var vstatus67 = ((vstatus >> 6) & 3);
      var vstatustext67 = "- 电池组过压故障: 无";
      if (vstatus67 == 1) {
        vstatustext67 = "- 电池组过压故障: 严重";
      } else if (vstatus67 == 2) {
        vstatustext67 = "- 电池组过压故障: 轻微";
      }
      var vstatus89 = ((vstatus >> 8) & 3);
      var vstatustext89 = "- 电池组欠压故障: 无";
      if (vstatus89 == 1) {
        vstatustext89 = "- 电池组欠压故障: 严重";
      } else if (vstatus89 == 2) {
        vstatustext89 = "- 电池组欠压故障: 轻微";
      }
      document.getElementById("robot_info_battery_vstatus01").innerText = vstatustext01;
      document.getElementById("robot_info_battery_vstatus23").innerText = vstatustext23;
      document.getElementById("robot_info_battery_vstatus45").innerText = vstatustext45;
      document.getElementById("robot_info_battery_vstatus67").innerText = vstatustext67;
      document.getElementById("robot_info_battery_vstatus89").innerText = vstatustext89;
      document.getElementById("robot_info_battery_vstatus01").style.display = 'block';
      document.getElementById("robot_info_battery_vstatus23").style.display = 'block';
      document.getElementById("robot_info_battery_vstatus45").style.display = 'block';
      document.getElementById("robot_info_battery_vstatus67").style.display = 'block';
      document.getElementById("robot_info_battery_vstatus89").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_vstatus01").style.display = 'none';
      document.getElementById("robot_info_battery_vstatus23").style.display = 'none';
      document.getElementById("robot_info_battery_vstatus45").style.display = 'none';
      document.getElementById("robot_info_battery_vstatus67").style.display = 'none';
      document.getElementById("robot_info_battery_vstatus89").style.display = 'none';
    }
    if (battery.hasOwnProperty("cstatus")) {
      var cstatus = parseInt(battery["cstatus"]);
      var cstatus01 = ((cstatus >> 0) & 3);
      var cstatustext01 = "- 充电过流故障: 无";
      if (cstatus01 == 1) {
        cstatustext01 = "- 充电过流故障: 严重";
      } else if (cstatus01 == 2) {
        cstatustext01 = "- 充电过流故障: 轻微";
      }
      var cstatus23 = ((cstatus >> 2) & 3);
      var cstatustext23 = "- 放电过流故障: 无";
      if (cstatus23 == 1) {
        cstatustext23 = "- 放电过流故障: 严重";
      } else if (cstatus23 == 2) {
        cstatustext23 = "- 放电过流故障: 轻微";
      }
      var cstatus45 = ((cstatus >> 4) & 3);
      var cstatustext45 = "- 短路保护故障: 无";
      if (cstatus45 == 1) {
        cstatustext45 = "- 短路保护故障: 严重";
      } else if (cstatus45 == 2) {
        cstatustext45 = "- 短路保护故障: 轻微";
      }
      document.getElementById("robot_info_battery_cstatus01").innerText = cstatustext01;
      document.getElementById("robot_info_battery_cstatus23").innerText = cstatustext23;
      document.getElementById("robot_info_battery_cstatus45").innerText = cstatustext45;
      document.getElementById("robot_info_battery_cstatus01").style.display = 'block';
      document.getElementById("robot_info_battery_cstatus23").style.display = 'block';
      document.getElementById("robot_info_battery_cstatus45").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_cstatus01").style.display = 'none';
      document.getElementById("robot_info_battery_cstatus23").style.display = 'none';
      document.getElementById("robot_info_battery_cstatus45").style.display = 'none';
    }

    if (battery.hasOwnProperty("tstatus")) {
      var tstatus = parseInt(battery["tstatus"]);
      var tstatus01 = ((tstatus >> 0) & 3);
      var tstatustext01 = "- 单体充电过温故障: 无";
      if (tstatus01 == 1) {
        tstatustext01 = "- 单体充电过温故障: 严重";
      } else if (tstatus01 == 2) {
        tstatustext01 = "- 单体充电过温故障: 轻微";
      }
      var tstatus23 = ((tstatus >> 2) & 3);
      var tstatustext23 = "- 单体放电过温故障: 无";
      if (tstatus23 == 1) {
        tstatustext23 = "- 单体放电过温故障: 严重";
      } else if (tstatus23 == 2) {
        tstatustext23 = "- 单体放电过温故障: 轻微";
      }
      var tstatus45 = ((tstatus >> 4) & 3);
      var tstatustext45 = "- 单体充电低温故障: 无";
      if (tstatus45 == 1) {
        tstatustext45 = "- 单体充电低温故障: 严重";
      } else if (tstatus45 == 2) {
        tstatustext45 = "- 单体充电低温故障: 轻微";
      }
      var tstatus67 = ((tstatus >> 6) & 3);
      var tstatustext67 = "- 单体放电低温故障: 无";
      if (tstatus67 == 1) {
        tstatustext67 = "- 单体放电低温故障: 严重";
      } else if (tstatus67 == 2) {
        tstatustext67 = "- 单体放电低温故障: 轻微";
      }
      var tstatus89 = ((tstatus >> 8) & 3);
      var tstatustext89 = "- 单体充电温差故障: 无";
      if (tstatus89 == 1) {
        tstatustext89 = "- 单体充电温差故障: 严重";
      } else if (tstatus89 == 2) {
        tstatustext89 = "- 单体充电温差故障: 轻微";
      }
      var tstatus1011 = ((tstatus >> 10) & 3);
      var tstatustext1011 = "- 单体放电温差故障: 无";
      if (tstatus1011 == 1) {
        tstatustext1011 = "- 单体放电温差故障: 严重";
      } else if (tstatus1011 == 2) {
        tstatustext1011 = "- 单体放电温差故障: 轻微";
      }
      document.getElementById("robot_info_battery_tstatus01").innerText = tstatustext01;
      document.getElementById("robot_info_battery_tstatus23").innerText = tstatustext23;
      document.getElementById("robot_info_battery_tstatus45").innerText = tstatustext45;
      document.getElementById("robot_info_battery_tstatus67").innerText = tstatustext67;
      document.getElementById("robot_info_battery_tstatus89").innerText = tstatustext89;
      document.getElementById("robot_info_battery_tstatus1011").innerText = tstatustext1011;
      document.getElementById("robot_info_battery_tstatus01").style.display = 'block';
      document.getElementById("robot_info_battery_tstatus23").style.display = 'block';
      document.getElementById("robot_info_battery_tstatus45").style.display = 'block';
      document.getElementById("robot_info_battery_tstatus67").style.display = 'block';
      document.getElementById("robot_info_battery_tstatus89").style.display = 'block';
      document.getElementById("robot_info_battery_tstatus1011").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_tstatus01").style.display = 'none';
      document.getElementById("robot_info_battery_tstatus23").style.display = 'none';
      document.getElementById("robot_info_battery_tstatus45").style.display = 'none';
      document.getElementById("robot_info_battery_tstatus67").style.display = 'none';
      document.getElementById("robot_info_battery_tstatus89").style.display = 'none';
      document.getElementById("robot_info_battery_tstatus1011").style.display = 'none';
    }
    if (battery.hasOwnProperty("alarm")) {
      var alarm = parseInt(battery["alarm"]);
      var alarm01 = ((alarm >> 0) & 3);
      var alarmtext01 = "- 预充失败故障: 无";
      if (alarm01 == 1) {
        alarmtext01 = "- 预充失败故障: 严重";
      } else if (alarm01 == 2) {
        alarmtext01 = "- 预充失败故障: 轻微";
      }
      var alarm23 = ((alarm >> 2) & 3);
      var alarmtext23 = "- 电量SOC过低故障: 无";
      if (alarm23 == 1) {
        alarmtext23 = "- 电量SOC过低故障: 严重";
      } else if (alarm23 == 2) {
        alarmtext23 = "- 电量SOC过低故障: 轻微";
      }
      document.getElementById("robot_info_battery_alarm01").innerText = alarmtext01;
      document.getElementById("robot_info_battery_alarm23").innerText = alarmtext23;
      document.getElementById("robot_info_battery_alarm01").style.display = 'block';
      document.getElementById("robot_info_battery_alarm23").style.display = 'block';
    } else {
      document.getElementById("robot_info_battery_alarm01").style.display = 'none';
      document.getElementById("robot_info_battery_alarm23").style.display = 'none';
    }
    document.getElementById("robot_info_battery").style.display = 'block';
  } else {
    document.getElementById("robot_info_battery").style.display = 'none';
  }
}

function do_robot_info_anti_drop(robot_info) {
  if (robot_info.hasOwnProperty("ads")) {
    var anti_drop_info = robot_info["ads"];
    if (anti_drop_info.hasOwnProperty("status")) {
      var anti_drop_status = parseInt(anti_drop_info["status"]);
      document.getElementById("robot_info_anti_drop_status").innerText = "- 悬崖检测: 无";
      if (anti_drop_status == 0) {
          document.getElementById("robot_info_anti_drop_status").innerText = "- 悬崖检测: 无";
      } else {
          document.getElementById("robot_info_anti_drop_status").innerText = "- 悬崖检测: 有";
      }
      document.getElementById("robot_info_anti_drop_status").style.display = 'block';
    } else {
      document.getElementById("robot_info_anti_drop_status").style.display = 'none';
    }

    if (anti_drop_info.hasOwnProperty("error")) {
      var anti_drop_error = parseInt(anti_drop_info["error"]);
      document.getElementById("robot_info_anti_drop_error").innerText = "- 悬崖检测: 通讯异常";
      if (anti_drop_error == -1) {
          document.getElementById("robot_info_anti_drop_error").innerText = "- 悬崖检测: 通讯异常";
      } else if (anti_drop_error == 0) {
          document.getElementById("robot_info_anti_drop_error").innerText = "- 悬崖检测: 通讯正常";
      }
      document.getElementById("robot_info_anti_drop_error").style.display = 'block';
    } else {
      document.getElementById("robot_info_anti_drop_error").style.display = 'none';
    }

    document.getElementById("robot_info_anti_drop").style.display = 'block';
  } else {
    document.getElementById("robot_info_anti_drop").style.display = 'none';
  }
  
}

function do_robot_info_double_com(robot_info) {
  if (robot_info.hasOwnProperty("doublecom")) {
    var dbcom_info = robot_info["doublecom"];
    if (dbcom_info.hasOwnProperty("level")) {
      var sig_level = parseInt(dbcom_info["level"]);
      if (sig_level == 0)
      {
          document.getElementById("robot_info_dbcom_level").innerText = "- 多倍通信号强度: 无";
      }
      else if (sig_level == 1)
      {
          document.getElementById("robot_info_dbcom_level").innerText = "- 多倍通信号强度: 弱";
      }
      else if (sig_level == 2)
      {
          document.getElementById("robot_info_dbcom_level").innerText = "- 多倍通信号强度: 中";
      }
      else if (sig_level == 3)
      {
          document.getElementById("robot_info_dbcom_level").innerText = "- 多倍通信号强度: 强";
      }

      document.getElementById("robot_info_dbcom_level").style.display = 'block';
    } else {
      document.getElementById("robot_info_dbcom_level").style.display = 'none';
    }

    if (dbcom_info.hasOwnProperty("sig_val")) {
      document.getElementById("robot_info_dbcom_signal_value").innerText = "- 多倍通信号强度: " + dbcom_info["sig_val"] + " dBm";
      document.getElementById("robot_info_dbcom_signal_value").style.display = 'block';
    } else {
      document.getElementById("robot_info_dbcom_signal_value").style.display = 'none';
    }

    if (dbcom_info.hasOwnProperty("connect_status")) {
      var connect_status = parseInt(dbcom_info["connect_status"]);
      if (connect_status == 1) {
          doublecom_connect_status = "- 诊断结果: 多倍通连接正常";
      } else if (connect_status == 0) {
          doublecom_connect_status = "- 诊断结果: 多倍通连接异常";
      }
      document.getElementById("robot_info_dbcom_connect_status").innerText = doublecom_connect_status;
      document.getElementById("robot_info_dbcom_connect_status").style.display = 'block';
    } else {
      document.getElementById("robot_info_dbcom_connect_status").style.display = 'none';
    }

    if (dbcom_info.hasOwnProperty("login_status")) {
      var login_status = parseInt(dbcom_info["login_status"]);
      if (login_status == 1) {
          doublecom_login_status = "- 诊断结果: 多倍通登录正常";
      } else if (login_status == 0) {
          doublecom_login_status = "- 诊断结果: 多倍通登录异常";
      }
      document.getElementById("robot_info_dbcom_login_status").innerText = doublecom_login_status;
      document.getElementById("robot_info_dbcom_login_status").style.display = 'block';
    } else {
      document.getElementById("robot_info_dbcom_login_status").style.display = 'none';
    }

    document.getElementById("robot_info_4g").style.display = 'block';
  } else {
    document.getElementById("robot_info_4g").style.display = 'none';
  }
}

function do_robot_info_4g(robot_info) {
  /*if (robot_info.hasOwnProperty("4g")) {
    var l4g = robot_info["4g"];
    if (l4g.hasOwnProperty("level")) {
      document.getElementById("robot_info_4g_level").innerText = "- 信号强度: " + l4g["level"] + " dBm";
      document.getElementById("robot_info_4g_level").style.display = 'block';
    } else {
      document.getElementById("robot_info_4g_level").style.display = 'none';
    }
    if (l4g.hasOwnProperty("error")) {
      var error = parseInt(l4g["error"]);
      var l4g_error = "- 诊断结果: 未知异常";
      if (error == -1) {
          l4g_error = "- 诊断结果: 通信异常";
      } else if (error == 0) {
          l4g_error = "- 诊断结果: 正常";
      }
      document.getElementById("robot_info_4g_error").innerText = l4g_error;
      document.getElementById("robot_info_4g_error").style.display = 'block';
    } else {
      document.getElementById("robot_info_4g_error").style.display = 'none';
    }
    document.getElementById("robot_info_4g").style.display = 'block';
  } else {
    document.getElementById("robot_info_4g").style.display = 'none';
  }*/
  if (robot_info.hasOwnProperty("sim")) {
    var sim = robot_info["sim"];
    if (sim.hasOwnProperty("iccid")) {
      document.getElementById("robot_info_sim_iccid").innerText = "- iccid: " + sim["iccid"];
      document.getElementById("robot_info_sim_iccid").style.display = 'block';
    } else {
      document.getElementById("robot_info_sim_iccid").style.display = 'none';
    }
    if (sim.hasOwnProperty("imsi")) {
      document.getElementById("robot_info_sim_imsi").innerText = "- imsi: " + sim["imsi"];;
      document.getElementById("robot_info_sim_imsi").style.display = 'block';
    } else {
      document.getElementById("robot_info_sim_imsi").style.display = 'none';
    }
    if (sim.hasOwnProperty("cellid")) {
      document.getElementById("robot_info_sim_cellid").innerText = "- cellid: " + sim["cellid"];;
      document.getElementById("robot_info_sim_cellid").style.display = 'block';
    } else {
      document.getElementById("robot_info_sim_cellid").style.display = 'none';
    }
  }
  if (robot_info.hasOwnProperty("router")) {
    var router = robot_info["router"];
    if (router.hasOwnProperty("imei")) {
      document.getElementById("robot_info_router_imei").innerText = "- imei: " + router["imei"];
      document.getElementById("robot_info_router_imei").style.display = 'block';
    } else {
      document.getElementById("robot_info_router_imei").style.display = 'none';
    }
    if (router.hasOwnProperty("mac")) {
      document.getElementById("robot_info_router_mac").innerText = "- mac地址: " + router["mac"];
      document.getElementById("robot_info_router_mac").style.display = 'block';
    } else {
      document.getElementById("robot_info_router_mac").style.display = 'none';
    }
  }
}

function do_robot_info_brake(robot_info) {
  if (robot_info.hasOwnProperty("brake")) {
    var brake = robot_info["brake"];
    if (brake.hasOwnProperty("button")) {
      var button = parseInt(brake["button"]);
      if (button != 0) {
        document.getElementById("robot_info_brake_button").innerText = "- 状态->急停按钮: 按下";
      } else {
        document.getElementById("robot_info_brake_button").innerText = "- 状态->急停按钮: 松开";
      }
      document.getElementById("robot_info_brake_button").style.display = 'block';
    } else {
      document.getElementById("robot_info_brake_button").style.display = 'none';
    }
    if (brake.hasOwnProperty("charge")) {
      var charge = parseInt(brake["charge"]);
      if (charge != 0) {
        document.getElementById("robot_info_brake_charge").innerText = "- 状态->充电行程开关: 按下";
      } else {
        document.getElementById("robot_info_brake_charge").innerText = "- 状态->充电行程开关: 松开";
      }
      document.getElementById("robot_info_brake_charge").style.display = 'block';
    } else {
      document.getElementById("robot_info_brake_charge").style.display = 'none';
    }
    if (brake.hasOwnProperty("charge_bumper")) {
      var charge_bumper = parseInt(brake["charge_bumper"]);
      if (charge_bumper != 0) {
        document.getElementById("robot_info_brake_charge_bumper").innerText = "- 状态->充电防撞条: 按下";
      } else {
        document.getElementById("robot_info_brake_charge_bumper").innerText = "- 状态->充电防撞条: 松开";
      }
      document.getElementById("robot_info_brake_charge_bumper").style.display = 'block';
    } else {
      document.getElementById("robot_info_brake_charge_bumper").style.display = 'none';
    }
    if (brake.hasOwnProperty("front_bumper")) {
      var front_bumper = parseInt(brake["front_bumper"]);
      if (front_bumper != 0) {
        document.getElementById("robot_info_brake_front_bumper").innerText = "- 状态->前防撞条: 按下";
      } else {
        document.getElementById("robot_info_brake_front_bumper").innerText = "- 状态->前防撞条: 松开";
      }
      document.getElementById("robot_info_brake_front_bumper").style.display = 'block';
    } else {
      document.getElementById("robot_info_brake_front_bumper").style.display = 'none';
    }
    if (brake.hasOwnProperty("tear_bumper")) {
      var tear_bumper = parseInt(brake["tear_bumper"]);
      if (tear_bumper != 0) {
        document.getElementById("robot_info_brake_tear_bumper").innerText = "- 状态->后防撞条: 按下";
      } else {
        document.getElementById("robot_info_brake_tear_bumper").innerText = "- 状态->后防撞条: 松开";
      }
      document.getElementById("robot_info_brake_tear_bumper").style.display = 'block';
    } else {
      document.getElementById("robot_info_brake_tear_bumper").style.display = 'none';
    }
    if (brake.hasOwnProperty("front_bumper_type")) {
      var front_bumper_type = brake["front_bumper_type"];
      if (front_bumper_type == "normally_closed") {
        document.getElementById("robot_info_brake_front_bumper_type").innerText = "- 类型->前防撞条: 常闭";
      } else {
        document.getElementById("robot_info_brake_front_bumper_type").innerText = "- 类型->前防撞条: 常开";
      }
      document.getElementById("robot_info_brake_front_bumper_type").style.display = 'block';
    } else {
      document.getElementById("robot_info_brake_front_bumper_type").style.display = 'none';
    }
    if (brake.hasOwnProperty("tear_bumper_type")) {
      var tear_bumper_type = brake["tear_bumper_type"];
      if (tear_bumper_type == "normally_closed") {
        document.getElementById("robot_info_brake_tear_bumper_type").innerText = "- 类型->后防撞条: 常闭";
      } else {
        document.getElementById("robot_info_brake_tear_bumper_type").innerText = "- 类型->后防撞条: 常开";
      }
      document.getElementById("robot_info_brake_tear_bumper_type").style.display = 'block';
    } else {
      document.getElementById("robot_info_brake_tear_bumper_type").style.display = 'none';
    }
    if (brake.hasOwnProperty("source")) {
      var source = brake["source"];
      if (source.hasOwnProperty("button")) {
        var source_button = parseInt(source["button"]);
        if (source_button != 0) {
          document.getElementById("robot_info_brake_source_button").innerText = "- 触发刹车->急停按钮: 是";
        } else {
          document.getElementById("robot_info_brake_source_button").innerText = "- 触发刹车->急停按钮: 否";
        }
        document.getElementById("robot_info_brake_source_button").style.display = 'block';
      } else {
        document.getElementById("robot_info_brake_source_button").style.display = 'none';
      }
      if (source.hasOwnProperty("can")) {
        var source_can = parseInt(source["can"]);
        if (source_can != 0) {
          document.getElementById("robot_info_brake_source_can").innerText = "- 触发刹车->远程控制: 是";
        } else {
          document.getElementById("robot_info_brake_source_can").innerText = "- 触发刹车->远程控制: 否";
        }
        document.getElementById("robot_info_brake_source_can").style.display = 'block';
      } else {
        document.getElementById("robot_info_brake_source_can").style.display = 'none';
      }
      if (source.hasOwnProperty("charge")) {
        var source_charge = parseInt(source["charge"]);
        if (source_charge != 0) {
          document.getElementById("robot_info_brake_source_charge").innerText = "- 触发刹车->充电: 是";
        } else {
          document.getElementById("robot_info_brake_source_charge").innerText = "- 触发刹车->充电: 否";
        }
        document.getElementById("robot_info_brake_source_charge").style.display = 'block';
      } else {
        document.getElementById("robot_info_brake_source_charge").style.display = 'none';
      }
      if (source.hasOwnProperty("charge_bumper")) {
        var source_charge_bumper = parseInt(source["charge_bumper"]);
        if (source_charge_bumper != 0) {
          document.getElementById("robot_info_brake_source_charge_bumper").innerText = "- 触发刹车->充电防撞条: 是";
        } else {
          document.getElementById("robot_info_brake_source_charge_bumper").innerText = "- 触发刹车->充电防撞条: 否";
        }
        document.getElementById("robot_info_brake_source_charge_bumper").style.display = 'block';
      } else {
        document.getElementById("robot_info_brake_source_charge_bumper").style.display = 'none';
      }
      if (source.hasOwnProperty("front_bumper")) {
        var source_front_bumper = parseInt(source["front_bumper"]);
        if (source_front_bumper != 0) {
          document.getElementById("robot_info_brake_source_front_bumper").innerText = "- 触发刹车->前撞条: 是";
        } else {
          document.getElementById("robot_info_brake_source_front_bumper").innerText = "- 触发刹车->前撞条: 否";
        }
        document.getElementById("robot_info_brake_source_front_bumper").style.display = 'block';
      } else {
        document.getElementById("robot_info_brake_source_front_bumper").style.display = 'none';
      }
      if (source.hasOwnProperty("iap")) {
        var source_iap = parseInt(source["iap"]);
        if (source_iap != 0) {
          document.getElementById("robot_info_brake_source_iap").innerText = "- 触发刹车->升级: 是";
        } else {
          document.getElementById("robot_info_brake_source_iap").innerText = "- 触发刹车->升级: 否";
        }
        document.getElementById("robot_info_brake_source_iap").style.display = 'block';
      } else {
        document.getElementById("robot_info_brake_source_iap").style.display = 'none';
      }
      if (source.hasOwnProperty("reboot")) {
        var source_reboot = parseInt(source["reboot"]);
        if (source_reboot != 0) {
          document.getElementById("robot_info_brake_source_reboot").innerText = "- 触发刹车->开机: 是";
        } else {
          document.getElementById("robot_info_brake_source_reboot").innerText = "- 触发刹车->开机: 否";
        }
        document.getElementById("robot_info_brake_source_reboot").style.display = 'block';
      } else {
        document.getElementById("robot_info_brake_source_reboot").style.display = 'none';
      }
      if (source.hasOwnProperty("tear_bumper")) {
        var source_tear_bumper = parseInt(source["tear_bumper"]);
        if (source_tear_bumper != 0) {
          document.getElementById("robot_info_brake_source_tear_bumper").innerText = "- 触发刹车->后防撞条: 是";
        } else {
          document.getElementById("robot_info_brake_source_tear_bumper").innerText = "- 触发刹车->后防撞条: 否";
        }
        document.getElementById("robot_info_brake_source_tear_bumper").style.display = 'block';
      } else {
        document.getElementById("robot_info_brake_source_tear_bumper").style.display = 'none';
      }
    }
	
	 if (brake.hasOwnProperty("control")) {
      var control = brake["control"];
      if (control.hasOwnProperty("CEmergency_brake")) {
        var control_CEmergency_brake = parseInt(control["CEmergency_brake"]);
        if (control_CEmergency_brake != 0) {
          document.getElementById("robot_info_brake_control_CEmergency_brake").innerText = "- 控制输出->CEmergency_brake: 是";
        } else {
          document.getElementById("robot_info_brake_control_CEmergency_brake").innerText = "- 控制输出->CEmergency_brake: 否";
        }
        document.getElementById("robot_info_brake_control_CEmergency_brake").style.display = 'block';
      } else {
        document.getElementById("robot_info_brake_control_CEmergency_brake").style.display = 'none';
      }
      if (control.hasOwnProperty("CMotor_brake")) {
        var control_CMotor_brake = parseInt(control["CMotor_brake"]);
        if (control_CMotor_brake != 0) {
          document.getElementById("robot_info_brake_control_CMotor_brake").innerText = "- 控制输出->CMotor_brake: 是";
        } else {
          document.getElementById("robot_info_brake_control_CMotor_brake").innerText = "- 控制输出->CMotor_brake: 否";
        }
        document.getElementById("robot_info_brake_control_CMotor_brake").style.display = 'block';
      } else {
        document.getElementById("robot_info_brake_control_CMotor_brake").style.display = 'none';
      }
      if (control.hasOwnProperty("W_software_brake")) {
        var control_W_software_brake = parseInt(control["W_software_brake"]);
        if (control_W_software_brake != 0) {
          document.getElementById("robot_info_brake_control_W_software_brake").innerText = "- 控制输出->W_software_brake: 是";
        } else {
          document.getElementById("robot_info_brake_control_W_software_brake").innerText = "- 控制输出->W_software_brake: 否";
        }
        document.getElementById("robot_info_brake_control_W_software_brake").style.display = 'block';
      } else {
        document.getElementById("robot_info_brake_control_W_software_brake").style.display = 'none';
      }
    }
    document.getElementById("robot_info_brake").style.display = 'block';
  } else {
    document.getElementById("robot_info_brake").style.display = 'none';
  }
}

function do_robot_info_camera(robot_info) {
  if (robot_info.hasOwnProperty("camera")) {
    var camera = robot_info["camera"];
    var camera_status = "- 旋转抓拍: 未知";
    if (camera.hasOwnProperty("status")) {
      var status = parseInt(camera["status"]);
      if (status == 0) {
        camera_status = "- 旋转抓拍: 网络异常";
      } else if(status == 1) {
        camera_status = "- 旋转抓拍: 空闲";
      } else if(status == 2) {
        camera_status = "- 旋转抓拍: 抓拍";
      }
    }
    document.getElementById("robot_info_camera_status").innerText = camera_status;
    document.getElementById("robot_info_camera_status").style.display = 'block';
    document.getElementById("robot_info_camera").style.display = 'block';
  } else {
    document.getElementById("robot_info_camera").style.display = 'none';
  }
}

function do_robot_info_disperse(robot_info) {
  if (robot_info.hasOwnProperty("disperse")) {
    var disperse = robot_info["disperse"];
    var disperse_status = "- 声波驱散: 未知";
    if (disperse.hasOwnProperty("status")) {
      var status = parseInt(disperse["status"]);
      if (status != 0) {
        disperse_status = "- 声波驱散: 打开";
      } else {
        disperse_status = "- 声波驱散: 关闭";
      }
    }
    document.getElementById("robot_info_disperse_status").innerText = disperse_status;
    document.getElementById("robot_info_disperse_status").style.display = 'block';
    document.getElementById("robot_info_disperse").style.display = 'block';
  } else {
    document.getElementById("robot_info_disperse").style.display = 'none';
  }
}

function do_robot_info_fan(robot_info) {
  if (robot_info.hasOwnProperty("fan")) {
    var fan = robot_info["fan"];
    if (fan.hasOwnProperty("bottom")) {
      var bottom = fan["bottom"];
      var fan_bottom_error = "- 诊断结果: "
      if (bottom.hasOwnProperty("error")) {
        var error = parseInt(bottom["error"]);
        fan_bottom_error += ((error >> 0) & 1) != 0 ? "风扇1-异常" : "风扇1-正常";
        fan_bottom_error += ((error >> 1) & 1) != 0 ? "|风扇2-异常" : "|风扇2-正常";
        fan_bottom_error += ((error >> 2) & 1) != 0 ? "|风扇3-异常" : "|风扇3-正常";
        fan_bottom_error += ((error >> 3) & 1) != 0 ? "|风扇4-异常" : "|风扇4-正常";
      } else {
        fan_bottom_error += "未知";
      }
      var fan_bottom_speed_in = "- 进风风速: 0";
      if (bottom.hasOwnProperty("speed_in")) {
        fan_bottom_speed_in =  "- 进风风速: " + bottom["speed_in"];
      }
      var fan_bottom_speed_out = "- 出风风速: 0";
      if (bottom.hasOwnProperty("speed_out")) {
        fan_bottom_speed_out =  "- 出风风速: " + bottom["speed_out"];
      }
      document.getElementById("robot_info_fan_bottom_error").innerText = fan_bottom_error;
      document.getElementById("robot_info_fan_bottom_speed_in").innerText = fan_bottom_speed_in;
      document.getElementById("robot_info_fan_bottom_speed_out").innerText = fan_bottom_speed_out;
      document.getElementById("robot_info_fan_bottom_error").style.display = 'block';
      document.getElementById("robot_info_fan_bottom_speed_in").style.display = 'block';
      document.getElementById("robot_info_fan_bottom_speed_out").style.display = 'block';
      document.getElementById("robot_info_fan_bottom").style.display = 'block';
    } else {
      document.getElementById("robot_info_fan_bottom").style.display = 'none';
    }

    if (fan.hasOwnProperty("middle")) {
      var middle = fan["middle"];
      var fan_middle_error = "- 诊断结果: "
      if (middle.hasOwnProperty("error")) {
        var error = parseInt(middle["error"]);
        fan_middle_error += ((error >> 0) & 1) != 0 ? "风扇1-异常" : "风扇1-正常";
        fan_middle_error += ((error >> 1) & 1) != 0 ? "|风扇2-异常" : "|风扇2-正常";
      } else {
        fan_middle_error += "未知";
      }
      
      var fan_middle_speed_in = "- 进风风速: 0";
      if (middle.hasOwnProperty("speed_in")) {
        fan_middle_speed_in =  "- 进风风速: " + middle["speed_in"];
      }
      var fan_middle_speed_out = "- 出风风速: 0";
      if (middle.hasOwnProperty("speed_out")) {
        fan_middle_speed_out = "- 出风风速: " + middle["speed_out"];
      }
      document.getElementById("robot_info_fan_middle_error").innerText = fan_middle_error;
      document.getElementById("robot_info_fan_middle_speed_in").innerText = fan_middle_speed_in;
      document.getElementById("robot_info_fan_middle_speed_out").innerText = fan_middle_speed_out;
      document.getElementById("robot_info_fan_middle_error").style.display = 'block';
      document.getElementById("robot_info_fan_middle_speed_in").style.display = 'block';
      document.getElementById("robot_info_fan_middle_speed_out").style.display = 'block';
      document.getElementById("robot_info_fan_middle").style.display = 'block';
    } else {
      document.getElementById("robot_info_fan_middle").style.display = 'none';
    }
  }
}

function do_robot_info_gps(robot_info) {
  if (robot_info.hasOwnProperty("gps")) {
    var gps_error = "- 诊断结果: 通信异常";
    var gps_lati = "- 维度: 0";
    var gps_long = "- 经度: 0";
    var gps_alti = "- 海拔: 0";
    var gps = robot_info["gps"];
    if (gps.hasOwnProperty("error")) {
      var error = parseInt(gps["error"]);
      if (error === -1) {
        gps_error = "- 诊断结果: 通信异常";
      } else if (error === 0) {
        gps_error = "- 诊断结果: 正常";
      } else {
        gps_error = "- 诊断结果: 精度太差";
      }
    }
    if (gps.hasOwnProperty("lati")) {
      gps_lati = "- 维度: " + gps["lati"];
    }
    if (gps.hasOwnProperty("long")) {
      gps_long = "- 经度: " + gps["long"];
    }
    if (gps.hasOwnProperty("alti")) {
      gps_alti = "- 海拔: " + gps["alti"];
    }
    document.getElementById("robot_info_gps_error").innerText = gps_error;
    document.getElementById("robot_info_gps_lati").innerText = gps_lati;
    document.getElementById("robot_info_gps_long").innerText = gps_long;
    document.getElementById("robot_info_gps_alti").innerText = gps_alti;
    document.getElementById("robot_info_gps_error").style.display = 'block';
    document.getElementById("robot_info_gps_lati").style.display = 'block';
    document.getElementById("robot_info_gps_long").style.display = 'block';
    document.getElementById("robot_info_gps_alti").style.display = 'block';
    document.getElementById("robot_info_gps").style.display = 'block';
  } else {
    document.getElementById("robot_info_gps").style.display = 'none';
  }
}

function do_robot_info_voip(robot_info) {
  if (robot_info.hasOwnProperty("voip")) {
    var voip = robot_info["voip"];
    var voip_status = "- 状态: 关闭";
    if (voip.hasOwnProperty("status")) {
      var status = parseInt(voip["status"]);
      voip_status = status != 0 ? "- 状态: 打开" : "- 状态: 关闭";
    }
    document.getElementById("robot_info_voip_status").innerText = voip_status;
    document.getElementById("robot_info_voip_status").style.display = 'block';
    document.getElementById("robot_info_voip").style.display = 'block';
  } else {
    document.getElementById("robot_info_voip").style.display = 'none';
  }
}

function do_robot_info_sensor_hall(robot_info) {
  if (robot_info.hasOwnProperty("sensor_hall")) {
    var sensor_hall = robot_info["sensor_hall"];
    if (sensor_hall.hasOwnProperty("hall1")) {
      document.getElementById("robot_info_sensor_hall1").innerText = "- hall1: " + sensor_hall["hall1"];
      document.getElementById("robot_info_sensor_hall1").style.display = 'block';
    } else {
      document.getElementById("robot_info_sensor_hall1").style.display = 'none';
    }
    if (sensor_hall.hasOwnProperty("hall2")) {
      document.getElementById("robot_info_sensor_hall2").innerText = "- hall2: " + sensor_hall["hall2"];
      document.getElementById("robot_info_sensor_hall2").style.display = 'block';
    } else {
      document.getElementById("robot_info_sensor_hall2").style.display = 'none';
    }
    if (sensor_hall.hasOwnProperty("hall3")) {
      document.getElementById("robot_info_sensor_hall3").innerText = "- hall3: " + sensor_hall["hall3"];
      document.getElementById("robot_info_sensor_hall3").style.display = 'block';
    } else {
      document.getElementById("robot_info_sensor_hall3").style.display = 'none';
    }
    document.getElementById("robot_info_sensor_hall").style.display = 'block';
  } else {
    document.getElementById("robot_info_sensor_hall").style.display = 'none';
  }
}

function do_robot_info_sensor_liquid(robot_info) {
  if (robot_info.hasOwnProperty("sensor_liquid")) {
    var sensor_liquid = robot_info["sensor_liquid"];
    if (sensor_liquid.hasOwnProperty("status")) {
      var status = parseInt(sensor_liquid["status"]);
      var sensor_liquid_status = (status != 0 ? "- 状态: 已浸水" : "- 状态: 未浸水");
      document.getElementById("robot_info_sensor_liquid_status").innerText = sensor_liquid_status;
      document.getElementById("robot_info_sensor_liquid_status").style.display = 'block';
      document.getElementById("robot_info_sensor_liquid").style.display = 'block';
    }
  } else {
    document.getElementById("robot_info_sensor_liquid").style.display = 'none';
  }
}

function do_robot_info_light(robot_info) {
  if (robot_info.hasOwnProperty("light")) {
    var light = robot_info["light"];
    var rb_status = "- 爆闪灯: 关闭"
    var w_status = "- 照明灯: 关闭";
    if (light.hasOwnProperty("rb_status")) {
      var status = parseInt(light["rb_status"]);
      rb_status = (status != 0 ? "- 爆闪灯: 打开" : "- 爆闪灯: 关闭");
    }
    if (light.hasOwnProperty("w_status")) {
      var status = parseInt(light["w_status"]);
      w_status = (status != 0 ? "- 照明灯: 打开" : "- 照明灯: 关闭");
    }
    document.getElementById("robot_info_light_rb_status").innerText = rb_status;
    document.getElementById("robot_info_light_w_status").innerText = w_status;
    document.getElementById("robot_info_light_rb_status").style.display = 'block';
    document.getElementById("robot_info_light_w_status").style.display = 'block';
    document.getElementById("robot_info_light").style.display = 'block';
  } else {
    document.getElementById("robot_info_light").style.display = 'none';
  }
}

function do_robot_info_main_lidar(robot_info) {
  if (robot_info.hasOwnProperty("main_lidar")) {
    var main_lidar = robot_info["main_lidar"];
    var main_lidar_error = "- 状态: 通信异常"
    if (main_lidar.hasOwnProperty("error")) {
      var error = parseInt(main_lidar["error"]);
      if (error == -1) {
        main_lidar_error = "- 状态: 通信异常";
      } else if (error == 0) {
        main_lidar_error = "- 状态: 正常";
      } else {
        main_lidar_error = "- 状态: 设备异常";
      }
    }
    document.getElementById("robot_info_main_lidar_status").innerText = main_lidar_error;
    document.getElementById("robot_info_main_lidar_status").style.display = 'block';
    document.getElementById("robot_info_main_lidar").style.display = 'block';
  } else {
    document.getElementById("robot_info_main_lidar").style.display = 'none';
  }
}

function do_robot_info_gyro(robot_info) {
  if (robot_info.hasOwnProperty("gyro")) {
    var gyro = robot_info["gyro"];
    var gyro_error = "- 状态: 通信异常";
    if (gyro.hasOwnProperty("error")) {
      var error = parseInt(gyro["error"]);
      if (error == -1) {
        gyro_error = "- 状态: 通信异常";
      } else if (error == 0) {
        gyro_error = "- 状态: 正常";
      } else {
        gyro_error = "- 状态: 设备异常";
      }
    }
    document.getElementById("robot_info_gyro_status").innerText = gyro_error;
    document.getElementById("robot_info_gyro_status").style.display = 'block';
    document.getElementById("robot_info_gyro").style.display = 'block';
  } else {
    document.getElementById("robot_info_gyro").style.display = 'none';
  }
}

function do_robot_info_odom(robot_info) {
  if (robot_info.hasOwnProperty("odom")) {
    var odom = robot_info["odom"];
    if (odom.hasOwnProperty("error")) {
      var error = parseInt(odom["error"]);
      var odom_error = "- 状态: 通信异常";
      if (error == -1) {
        odom_error = "- 状态: 通信异常";
      } else if (error == 0) {
        odom_error = "- 状态: 正常";
      } else {
        odom_error = "- 状态: 设备异常";
      }
      document.getElementById("robot_info_odom_status").innerText = odom_error;
      document.getElementById("robot_info_odom_status").style.display = 'block';
    }
    
    if (odom.hasOwnProperty("odo")) {
      var null_item = null;
      if (odom["odo"] != null_item){
        var odom_odo = "- 总里程: " + odom["odo"].toFixed(1) + " 米";
        document.getElementById("robot_info_odom_odo").innerText = odom_odo;
        document.getElementById("robot_info_odom_odo").style.display = 'block';
      }
    }
    if (odom.hasOwnProperty("speed_linear")) {
      var odom_speed_linear = "- 线性速度: " + odom["speed_linear"].toFixed(1) + " 米/秒";
      document.getElementById("robot_info_odom_speed_linear").innerText = odom_speed_linear;
      document.getElementById("robot_info_odom_speed_linear").style.display = 'block';
    }
    if (odom.hasOwnProperty("speed_theta")) {
      var odom_speed_theta = "- 转弯速度: " + odom["speed_theta"].toFixed(1) + " 弧度/秒";
      document.getElementById("robot_info_odom_speed_theta").innerText = odom_speed_theta;
      document.getElementById("robot_info_odom_speed_theta").style.display = 'block';
    }
    document.getElementById("robot_info_odom").style.display = 'block';
  } else {
    document.getElementById("robot_info_odom").style.display = 'none';
  }
}

function do_robot_info_patrol(robot_info) {
  if (robot_info.hasOwnProperty("patrol")) {
    var patrol = robot_info["patrol"];
    if (patrol.hasOwnProperty("status")) {
        var status = parseInt(patrol["status"]);
        var patrol_status = (status != 0 ? "- 巡逻状态: 巡逻中" : "- 巡逻状态: 空闲");
        document.getElementById("robot_info_patrol_status").innerText = patrol_status;
        document.getElementById("robot_info_patrol_status").style.display = 'block';
        document.getElementById("robot_info_base").style.display = 'block';
    }
  }
}

function do_robot_info_slave_lidar(robot_info) {
  if (robot_info.hasOwnProperty("slave_lidar")) {
    var slave_lidar = robot_info["slave_lidar"];
    var slave_lidar_error = "- 状态: 通信异常"
    if (slave_lidar.hasOwnProperty("error")) {
      var error = parseInt(slave_lidar["error"]);
      if (error == -1) {
        slave_lidar_error = "- 状态: 通信异常"
      } else if (error == 0) {
        slave_lidar_error = "- 状态: 正常";
      } else {
        slave_lidar_error = "- 状态: 设备异常";
      }
    }
    document.getElementById("robot_info_slave_lidar_status").innerText = slave_lidar_error;
    document.getElementById("robot_info_slave_lidar_status").style.display = 'block';
    document.getElementById("robot_info_slave_lidar").style.display = 'block';
  } else {
    document.getElementById("robot_info_slave_lidar").style.display = 'none';
  }
}

function do_robot_info_speed(robot_info) {
  if (robot_info.hasOwnProperty("speed")) {
    var speed = robot_info["speed"];
    if (speed.hasOwnProperty("value")) {
      var value = parseFloat(speed["value"]);
      var speed_value = "- 巡逻速度: 低";
      if (value < 0.3) {
        speed_value = "- 巡逻速度: 低";
      } else if (value < 0.5) {
        speed_value = "- 巡逻速度: 中";
      } else {
        speed_value = "- 巡逻速度: 高";
      }
      document.getElementById("robot_info_base_speed").innerText = speed_value;
      document.getElementById("robot_info_base_speed").style.display = 'block';
      document.getElementById("robot_info_base").style.display = 'block';
    }
  }
}

function do_robot_info_upgrade(robot_info) {
  if (robot_info.hasOwnProperty("upgrade")) {
    var upgrade = robot_info["upgrade"];
    if (upgrade.hasOwnProperty("status")) {
      var status = parseInt(upgrade["status"]);
      var upgrade_status = "- 升级状态: 升级完成";
      if (status == 0) {
        upgrade_status = "- 升级状态: 升级完成";
      } else if (status == 1) {
        upgrade_status = "- 升级状态: 正在升级中...";
      } else {
        upgrade_status = "- 升级状态: 升级失败";
      }
      document.getElementById("robot_info_base_upgrade_status").innerText = upgrade_status;
      document.getElementById("robot_info_base_upgrade_status").style.display = 'block';
      document.getElementById("robot_info_base").style.display = 'block';
    }
  }
}

function do_robot_info_volume(robot_info) {
  if (robot_info.hasOwnProperty("volume")) {
    var volume = robot_info["volume"];
    if (volume.hasOwnProperty("muted")) {
      var muted = parseInt(volume["muted"]);
      var volume_muted = (muted != 0 ? "- 喇叭静音: 开" : "- 喇叭静音: 关");
      document.getElementById("robot_info_base_volume_muted").innerText = volume_muted;
      document.getElementById("robot_info_base_volume_muted").style.display = 'block';
      document.getElementById("robot_info_base").style.display = 'block';
    }
    if (volume.hasOwnProperty("value")) {
      var volume_value = "- 喇叭音量: " + parseInt(volume["value"]);
      document.getElementById("robot_info_base_volume_value").innerText = volume_value;
      document.getElementById("robot_info_base_volume_value").style.display = 'block';
      document.getElementById("robot_info_base").style.display = 'block';
    }
  }
}

function do_robot_info_version(robot_info) {
  if (robot_info.hasOwnProperty("version")) {
    var version = robot_info["version"];
    if (version.hasOwnProperty("imx")) {
      document.getElementById("robot_info_version_imx").innerText = "- 主控板软件: " + version["imx"];
      document.getElementById("robot_info_version_imx").style.display = 'block';
    } else {
      document.getElementById("robot_info_version_imx").style.display = 'none';
    }
    if (version.hasOwnProperty("battery_monitor")) {
      document.getElementById("robot_info_version_battery_monitor").innerText = "- 监控板固件: " + version["battery_monitor"];
      document.getElementById("robot_info_version_battery_monitor").style.display = 'block';
    } else {
      document.getElementById("robot_info_version_battery_monitor").style.display = 'none';
    }
    if (version.hasOwnProperty("power")) {
      document.getElementById("robot_info_version_power").innerText = "- 电源板固件: " + version["power"];
      document.getElementById("robot_info_version_power").style.display = 'block';
    } else {
      document.getElementById("robot_info_version_power").style.display = 'none';
    }
    if (version.hasOwnProperty("gs")) {
      document.getElementById("robot_info_version_gs").innerText = "- 导航固件: xs_package-1.4.5";
      document.getElementById("robot_info_version_gs").style.display = 'block';
    } else {
      document.getElementById("robot_info_version_gs").style.display = 'none';
    }
    if(version.hasOwnProperty("ultroso_ks106")) {
      var ks106_ver = parseInt(version["ultroso_ks106"]);
      document.getElementById("robot_info_version_ultroso_ks106").innerText = "- 超声KS106版本: 0x" + ks106_ver.toString(16).toUpperCase();
      document.getElementById("robot_info_version_ultroso_ks106").style.display = 'block';
    }
    if(version.hasOwnProperty("ultroso_ks136")) {
      var ks136_ver = parseInt(version["ultroso_ks136"]);
      document.getElementById("robot_info_version_ultroso_ks136").innerText = "- 超声KS136版本: 0x" + ks136_ver.toString(16).toUpperCase();
      document.getElementById("robot_info_version_ultroso_ks136").style.display = 'block';
    }
    if(version.hasOwnProperty("power_iap")) {
      var power_iap_ver = parseInt(version["power_iap"]);
      document.getElementById("robot_info_version_power_iap").innerText = "- 电源BootLoader版本: 0x" + power_iap_ver.toString(16).toUpperCase();
      document.getElementById("robot_info_version_power_iap").style.display = 'block';
    }
    if (version.hasOwnProperty("chassis_sw_ver")) {
      document.getElementById("robot_info_version_chassis_software_ver").innerText = "- 电机控制板软件版本: " + version["chassis_sw_ver"];
      document.getElementById("robot_info_version_chassis_software_ver").style.display = 'block';
    } else {
      document.getElementById("robot_info_version_chassis_software_ver").style.display = 'none';
    }
    if (version.hasOwnProperty("chassis_core_hw_ver")) {
      var chassis_core_hw_version = parseInt(version["chassis_core_hw_ver"]);
      document.getElementById("robot_info_version_chassis_core_hw_ver").innerText = "- 电机控制板核心板硬件版本: " + chassis_core_hw_version.toString();
      document.getElementById("robot_info_version_chassis_core_hw_ver").style.display = 'block';
    } else {
      document.getElementById("robot_info_version_chassis_core_hw_ver").style.display = 'none';
    }
    if (version.hasOwnProperty("chassis_base_hw_ver")) {
      var chassis_base_hw_version = parseInt(version["chassis_base_hw_ver"])
      document.getElementById("robot_info_version_chassis_base_hw_ver").innerText = "- 电机控制板底板硬件版本: " + chassis_base_hw_version.toString();
      document.getElementById("robot_info_version_chassis_base_hw_ver").style.display = 'block';
    } else {
      document.getElementById("robot_info_version_chassis_base_hw_ver").style.display = 'none';
    }
    if(version.hasOwnProperty("monitor_sw_ver")) {
      document.getElementById("robot_info_version_monitor_sw_ver").innerText = "- 系统监控板软件版本: " + version["monitor_sw_ver"];
      document.getElementById("robot_info_version_monitor_sw_ver").style.display = 'block';
    }
    if(version.hasOwnProperty("monitor_core_hw_ver")) {
      var monitor_core_hw_version = parseInt(version["monitor_core_hw_ver"]);
      document.getElementById("robot_info_version_monitor_core_hw_ver").innerText = "- 系统监控板核心板硬件版本: " + monitor_core_hw_version.toString();
      document.getElementById("robot_info_version_monitor_core_hw_ver").style.display = 'block';
    }
    if(version.hasOwnProperty("monitor_base_hw_ver")) {
      var monitor_base_hw_version = parseInt(version["monitor_base_hw_ver"]);
      document.getElementById("robot_info_version_monitor_base_hw_ver").innerText = "- 系统监控板底板硬件版本: " + monitor_base_hw_version.toString();
      document.getElementById("robot_info_version_monitor_base_hw_ver").style.display = 'block';
    }
    if(version.hasOwnProperty("bms_hw_ver")) {
      document.getElementById("robot_info_version_bms_hw_ver").innerText = "- bms硬件版本: " + version["bms_hw_ver"];
      document.getElementById("robot_info_version_bms_hw_ver").style.display = 'block';
    }
    if(version.hasOwnProperty("bms_sw_ver")) {
      document.getElementById("robot_info_version_bms_sw_ver").innerText = "- bms软件版本: " + version["bms_sw_ver"];
      document.getElementById("robot_info_version_bms_sw_ver").style.display = 'block';
    }
    document.getElementById("robot_info_base").style.display = 'block';
  }
}

function do_robot_info_ppplay(robot_info) {
  if (robot_info.hasOwnProperty("ppplay")) {
    var ppplay = robot_info["ppplay"];
    if (ppplay.hasOwnProperty("status")) {
      var status = parseInt(ppplay["status"]);
      var ppplay_status = "- 状态: 未知";
      if (status == 0) {
        ppplay_status = "- 状态: 空闲";
      } else if (status === 1) {
        ppplay_status = "- 状态: 播放";
      } else if (status === 2) {
        ppplay_status = "- 状态: 暂停";
      } else if (status === 3) {
        ppplay_status = "- 状态: 等待";
      }
      document.getElementById("robot_info_ppplay_status").innerText = ppplay_status;
      document.getElementById("robot_info_ppplay_status").style.display = 'block';
    }
    if (ppplay.hasOwnProperty("interval")) {
      var ppplay_interval = "- 播报间隔: " + ppplay["interval"] + " 毫秒";
      document.getElementById("robot_info_ppplay_interval").innerText = ppplay_interval;
      document.getElementById("robot_info_ppplay_interval").style.display = 'block';
    }
    if (ppplay.hasOwnProperty("name")) {
      var ppplay_name = "- 名称: " + ppplay["name"];
      document.getElementById("robot_info_ppplay_name").innerText = ppplay_name;
      document.getElementById("robot_info_ppplay_name").style.display = 'block';
    }
    if (ppplay.hasOwnProperty("duration")) {
      var ppplay_duration = "- 总时长: " + ppplay["duration"] + " 毫秒";
      document.getElementById("robot_info_ppplay_duration").innerText = ppplay_duration;
      document.getElementById("robot_info_ppplay_duration").style.display = 'block';
    }
    if (ppplay.hasOwnProperty("pts")) {
      var ppplay_pts = "- 播放进度: " + ppplay["pts"] + " 毫秒";
      document.getElementById("robot_info_ppplay_pts").innerText = ppplay_pts;
      document.getElementById("robot_info_ppplay_pts").style.display = 'block';
    }
    document.getElementById("robot_info_ppplay").style.display = 'block';
  } else {
    document.getElementById("robot_info_ppplay").style.display = 'none';
  }
}

function do_robot_info_ultrasound(robot_info) {
  if (robot_info.hasOwnProperty("ultrasound")) {
    var ultrasound = robot_info["ultrasound"];
    if (ultrasound.hasOwnProperty("error")) {
      var error = parseInt(ultrasound["error"]);
      var ultrasound_error = "- 状态: 通信异常";
      if (error == -1) {
        ultrasound_error = "- 状态: 通信异常";
      } else if (error == 0) {
        ultrasound_error = "- 状态: 正常";
      } else {
        ultrasound_error = "- 状态: 设备异常";
      }
      document.getElementById("robot_info_ultrasound_status").innerText = ultrasound_error;
      document.getElementById("robot_info_ultrasound_status").style.display = 'block';
    }
    if (ultrasound.hasOwnProperty("data")) {
      var ultrasound_data = "- 数据: " + ultrasound["data"];
      document.getElementById("robot_info_ultrasound_data").innerText = ultrasound_data;
      document.getElementById("robot_info_ultrasound_data").style.display = 'block';
    }
    document.getElementById("robot_info_ultrasound").style.display = 'block';
  } else {
    document.getElementById("robot_info_ultrasound").style.display = 'none';
  }
}

function do_robot_info_gaussian_status(robot_info) {
  if (robot_info.hasOwnProperty("gaussian_status")) {
    var gaussian_status = robot_info["gaussian_status"];
    var gaussian_status_data = JSON.stringify(gaussian_status, null, 4);
    document.getElementById("robot_info_gaussian_status_data").innerText = gaussian_status_data;
    document.getElementById("robot_info_gaussian_status_data").style.display = 'block';
    document.getElementById("robot_info_gaussian_status").style.display = 'block';
  } else {
    document.getElementById("robot_info_gaussian_status").style.display = 'none';
  }
}

function do_diagnostics(content) {
  if (content.hasOwnProperty("robot_info")) {
    do_robot_info_base(content["robot_info"]);
    do_robot_info_temp_humi(content["robot_info"]);
    do_robot_info_chassis_driver(content["robot_info"]);
    do_robot_info_battery(content["robot_info"]);
    do_robot_info_4g(content["robot_info"]);
    do_robot_info_double_com(content["robot_info"]);
    do_robot_info_anti_drop(content["robot_info"]);
    do_robot_info_brake(content["robot_info"]);
    do_robot_info_camera(content["robot_info"]);
    do_robot_info_disperse(content["robot_info"]);
    do_robot_info_fan(content["robot_info"]);
    do_robot_info_gps(content["robot_info"]);
    do_robot_info_voip(content["robot_info"]);
    do_robot_info_sensor_hall(content["robot_info"]);
    do_robot_info_sensor_liquid(content["robot_info"]);
    do_robot_info_light(content["robot_info"]);
    do_robot_info_main_lidar(content["robot_info"]);
    do_robot_info_gyro(content["robot_info"]);
    do_robot_info_odom(content["robot_info"]);
    do_robot_info_patrol(content["robot_info"]);
    do_robot_info_slave_lidar(content["robot_info"]);
    do_robot_info_speed(content["robot_info"]);
    do_robot_info_upgrade(content["robot_info"]);
    do_robot_info_volume(content["robot_info"]);
    do_robot_info_version(content["robot_info"]);
    do_robot_info_ppplay(content["robot_info"]);
    do_robot_info_ultrasound(content["robot_info"]);
    do_robot_info_gaussian_status(content["robot_info"]);
  }
}
