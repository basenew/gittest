cmake_minimum_required(VERSION 2.8.3)
project(atris_msgs)

find_package(catkin REQUIRED roscpp std_msgs sensor_msgs geometry_msgs nav_msgs message_generation)

file(GLOB_RECURSE MsgFiles "msg/*.msg")
file(GLOB_RECURSE SrvFiles "srv/*.srv")
file(GLOB_RECURSE IncludeFiles "include/*.h")

add_message_files(
  FILES
  AisoundTTS.msg
  CanData.msg
  CanPkg.msg
  ChargeCmd.msg
  PowerChargeCmd.msg
  McuLogCtrl.msg
  ChargeInfo.msg
  DockSDKCmd.msg
  DTMessage.msg
  LampCmd.msg
  LocationCmd.msg
  MiddleFanControl.msg
  NavAbilityMessage.msg
  NavPoint.msg
  NavPoint2.msg
  NavReset.msg
  ProtectMode.msg
  RebootBraked.msg
  RobotBraked.msg
  RobotImu.msg
  RobotInfo.msg
  RobotPose.msg
  RobotShaked.msg
  RobotRunMode.msg
  NetworkState.msg
  SignalMessage.msg
  SprayerCmd.msg
  TimeCalibrated.msg
  UpgradeStatus.msg
  VelCmd.msg
  AntiDrop.msg
  PtzCtrlTest.msg
)

add_service_files(
  FILES
  ExclusiveVoiceChat.srv
  GetAgoraLoginStatus.srv
  GetChassisInfo.srv
  GetDiagnosticsInfo.srv
  GetDisperseStatus.srv
  GetGpsPos.srv
  GetGsLaserRawData.srv
  GetLampStatus.srv
  GetStandbyMode.srv
  GetNavPointList.srv
  GetNavPoint2List.srv
  GetNavPose.srv
  GetOTAPreCondition.srv
  GetPatrolStatus.srv
  GetPPPlayingStatus.srv
  GetPtzStatus.srv
  GetSprayerStatus.srv
  GetSwVersion.srv
  GetVoiceChatStatus.srv
  JoinChannel.srv
  NavToPoint.srv
  NavToPoint2.srv
  PPPlayerControl.srv
  PtzControl.srv
  PeripheralControl.srv
  StopVoiceChat.srv
  GetVisionResult.srv
  GetPtzTemperature.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

add_custom_target(atris_msgs SOURCES ${MsgFiles})
add_custom_target(atris_srvs SOURCES ${SrvFiles})
add_dependencies(atris_msgs ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})
add_dependencies(atris_srvs ${PROJECT_NAME}_gencpp ${catkin_EXPORTED_TARGETS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs message_runtime
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES ${IncludeFiles} DESTINATION include)
    
