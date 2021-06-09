

## 一.方法

*注意：标有异步调用的方法，其返回值仅表示该调用请求是否功成功，当无错误发生时，该接口返回的数据由对应的信号给出。*

------

### 连接机器人

```c++
bool Connect(const std::string &ip, uint16_t port);
```

#### 方法描述:

连接到机器人服务,再调用其他接口之前,必须调用此方法,并且返回值为True.

#### 方法参数:

| 参数名称 | 描述               |
| -------- | ------------------ |
| ip       | 机器人服务的IP地址 |
| port     | 机器人服务的端口   |

#### 返回值:

成功:true,成功连接到机器人服务.
失败:false,通过LastError方法获取失败原因.

------



### 处理通信事件

```c++
1 int ProcessEvent(int max_event_count = 0);
```

#### 方法描述:

信号驱动函数(信号由事件驱动),此函数应该放在程序中的主循环中.所有的信号触发的
前提条件为此方法被调用.

#### 方法参数:

| 参数名称        | 描述                             |
| --------------- | -------------------------------- |
| max_event_count | 事件处理计数,默认0为处理所有事件 |

#### 返回值:

被处理的事件个数.

------


### LastError:

```c++
ErrorCode LastError()const;
```

#### 方法描述:

获取最后一次错误.当所有接口返回-1或者Flase的时候,调用次方法获得具体出错的原
因.

#### 返回值:

错误码

------


### RegisterMe:

```c++
1 bool RegisterMe(int64_t appid=0, const std::string &sercet=std::string());
```

#### 方法描述(异步调用):

注册. 在conncet方法返回True之后, 调用次方法来注册当前SDK,以保证其他接口工作
正常.

#### 方法参数:

| 参数名称 | 描述       |
| -------- | ---------- |
| appid    | 由行深分配 |
| sercet   | 由行深分配 |

#### 返回值:

成功:true,仅指示异步调用成功,具体结果参照信号返回.
失败:false,通过LastError方法获取失败原因.

#### 相关信号:

sig_register_return

------


### UnregisterMe:

```c++
bool UnregisterMe();
```

#### 方法描述(异步调用):

退出注册,使用SDK的程序退出之前,须调用此方法，表示退出使用SDK..

#### 返回值:

成功:true,仅指示异步调用成功,具体结果参照信号返回.
失败:false,通过LastError方法获取失败原因.

#### 相关信号:

sig_unregister_return

------


### QueryParameter:

```c++
int QueryParameter();
```

#### 方法描述(异步调用):

获取机器人参数设置,如速率等级和对应的速率值.

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_query_parameter_return

------


### QueryRoute:

```c++
int QueryRoute();
```
#### 方法描述(异步调用):

查询路线,包括路线的开始时间,到达时间,路点集合,起始站点,结束站点等等信息.

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_query_route_return

------


### MappingControl:

```c++
int MappingControl(const MapperControl& param);
```

#### 方法描述(异步调用):

建图控制,包括开始建图、停止建图、保存建图结果、暂停建图、恢复建图等等操作。

#### 方法参数:

参见结构数据MapperControl

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_mapping_control_return

------


### QueryMap:

```c++
int QueryMap();
```

#### 方法描述(异步调用):

获取当前使用的地图信息，包括地图范围描述及地图图片。

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_query_map_return

------


### SubscribeInfomation:

```c++
int SubscribeInfomation(const Subscribe& param);
```

#### 方法描述(异步调用):

数据订阅,获取车辆相关信息，如车辆状态，电池状态，任务状态，巡逻任务状态等
等。

#### 方法参数:

参见结构数据Subscribe

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_subscribe_infomation_return
sig_vehicle_state
sig_Fuel_state
sig_switch_state
sig_task_state
sig_partrol_task_state
sig_global_pose
sig_arrive_station
sig_turn
sig_vehicle_start
sig_vehicle_stop
sig_obstacle
sig_fault

------


### StartTask:

```c++
int StartTask(const StationInfo& param);
```

#### 方法描述(异步调用):

下发停靠点任务给机器人，需要填入停靠点ID，停靠点位置，车辆朝向，停靠点类型
等信息。

#### 方法参数:

参见结构数据StationInfo

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_start_task_return

------


### StarPathTask:

```c++
int StarPathTask(const PathInfo& param);
```

#### 方法描述(异步调用):

下发路线任务，需要填入路线ID、路线名、开始点、结束点等信息

#### 方法参数:

参见结构数据PathInfo

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_start_path_task_return

------


### ControlTask:

```c++
int ControlTask(const TaskControl &param);
```

#### 方法描述(异步调用):

任务控制，包括开始任务、暂停任务、恢复任务、取消任务等操作。注意：任务下发
后并没有马上执行任务，需要调用此接口让机器人执行任务，

#### 方法参数:

参见结构数据TaskControl

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_control_task_return

------


### Navigate:

```c++
int Navigate(const NavigationPoint &param);
```

#### 方法描述(异步调用):

导航到任意点

#### 方法参数:

参见结构数据NavigationPoint

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_navigate_return

------


### QueryTaskObject:

```c++
int QueryTaskObject(const GetTaskObject &param);
```

#### 方法描述(异步调用):

获取任务对象列表， 例如路线任务列表、停靠点列表、巡逻任务列表、地图列表、站
点列表。

#### 方法参数:

参见结构数据GetTaskObject

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_qto_path_info_return
sig_qto_station_info_return
sig_qto_patro_info_return
sig_qto_group_info_return
sig_qto_map_info_return
sig_qto_site_info_return

------


### SetParameter:

```c++
int SetParameter(const StationInfo &param);
```

#### 方法描述(异步调用):

设置停靠点，需填入停靠点ID、名称、经纬度、停车时的朝向、停靠点类型。

#### 方法参数:

参见结构数据StationInfo

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_set_station_return

------


### SetParameter:

```c++
int SetParameter(const ParameterSetting&param);
```

#### 方法描述(异步调用):

设置参数，例如速度等级，速率值等。

#### 方法参数:

参见结构数据ParameterSetting

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_set_parameter_return

------


### ModifyPathName:

```c++
int ModifyPathName(const PathInfo &param);
```

#### 方法描述(异步调用):

修改路线任务名称，填入路线任务ID与新名称。

#### 方法参数:

参见结构数据PathInfo

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_modify_path_name_return

------


### ControlLocalisation:

```c++
int ControlLocalisation(const LocalisationControl &param);
```

#### 方法描述(异步调用):

定位控制相关操作，定位方式有站点定位、GPS定位、手动定位等。控制定位操作有：
暂停定位、恢复定位、停止定位。

#### 方法参数:

参见结构数据LocalisationControl

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_control_localisation_return

------


### QueryLocalisationPoint:

```c++
int QueryLocalisationPoint();
```

#### 方法描述(异步调用):

拉取定位点列表

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_query_localisation_point_return

------


### QueryLocalisationData:

```c++
int QueryLocalisationData(const LocalisationControl &param);
```

#### 方法描述(异步调用):

使用不同定位方式并拉取定位数据

#### 方法参数:

参见结构数据LocalisationControl

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_query_localisation_data_return

------


### SetLocalisationResult:

```c++
int SetLocalisationResult(const LocalisationResult &param);
```

#### 方法描述(异步调用):

设置定位结果

#### 方法参数:

参见结构数据LocalisationResult

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_set_localisation_result_return

------


### QueryGlobalMap:

```c++
int QueryGlobalMap();
```

#### 方法描述(异步调用):

获取全局底图

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_query_global_map_return

------


### ControlRouteRecording:

```c++
int ControlRouteRecording(const RouteRecordControl &param);
```

#### 方法描述(异步调用):

路径采集控制，包括开始，停止、保存、暂停、恢复等操作。

#### 方法参数:

参见结构数据RouteRecordControl

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_control_route_recording_return

------


### ControlSynchronization:

```c++
int ControlSynchronization (const SyncControl &param);
```

#### 方法描述(异步调用):

数据同步控制

#### 方法参数:

参见结构数据SyncControl

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_control_synchronization_return

------


### ApplyMap:

```c++
int ApplyMap(const MapInfo &param);
```

#### 方法描述(异步调用):

使用地图param所描述的地图，

#### 方法参数:

参见结构数据MapInfo

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_apply_map_return

------


### UpdateMap:

```c++
int UpdateMap(const MapInfo &param);
```

#### 方法描述(异步调用):

修改地图描述信息。

#### 方法参数:

参见结构数据MapInfo

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_update_map__return

------


### DeleteMap:

```c++
int DeleteMap(const MapInfo &param, bool all = false);
```

#### 方法描述(异步调用):

删除地图描述信息。

#### 方法参数:

参见结构数据MapInfo

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_delete_map_return

------


### QueryMappingState:

```c++
int QueryMappingState();
```

#### 方法描述(异步调用):

获取当前建图进度状态，如进行中，已暂停等等。

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_query_mapping_state_return

------


### QueryLocalisationState:

```c++
int QueryLocalisationState();
```

#### 方法描述(异步调用):

获取当前定位进度状态，如进行中，已暂停等等。

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_query_localisation_state_return

------


### QueryLocalisationResult:

```c++
int QueryLocalisationResult();
```

#### 方法描述(异步调用):

获取定位结果

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_query_localisation_result_return

------


### QueryRouteRecordingState:

```c++
int QueryRouteRecordingState();
```

#### 方法描述(异步调用):

获取路径采集进度状态，如进行中，已暂停等等。

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_query_route_recording_state_return

------


### TransferMap:

```c++
int TransferMap(const MapRequest& param);
```

#### 方法描述(异步调用):

地图上传下载

#### 方法参数:

参见结构数据MapRequest

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_transfer_map_return

------


### ApplySite:

```c++
int ApplySite(const SiteInfo& param);
```

#### 方法描述(异步调用):

使用站点

#### 方法参数:

参见结构数据SiteInfo

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_apply_site_return

------


### ControlLocalisating:

```c++
int ControlLocalisating(const LocalisatingControl &param);
```

#### 方法描述(异步调用):

控制当前的定位操作，包括暂停、停止、恢复。

#### 方法参数:

参见结构数据LocalisatingControl

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_control_localisating_return

------


### QueryNavigationSettting:

```c++
int QueryNavigationSettting();
```

#### 方法描述(异步调用):

查询导航相关数据设置，包括避模式、安全距离、速度等级设置等等。

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_query_navigation_setting_return

------


### ModifyNavigationSettting:

```c++
int ModifyNavigationSettting(const NavigationSetting& param);
```

#### 方法描述(异步调用):

修改导航相关数据，包括避模式、安全距离、速度等级设置等等。


#### 方法参数:

参见结构数据NavigationSetting

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_modify_navigation_settting_return

------


### ModifyNavigationSettting:

```c++
int ModifyNavigationSettting(const NavigationSetting& param);
```

#### 方法描述(异步调用):

查询导航设置信息

#### 方法参数:

参见结构数据NavigationSetting

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_modify_navigation_settting_return

------


### RenameMap

```c++
int RenameMap(const MapRename& param);
```

#### 方法描述(异步调用):

重命名地图

#### 方法参数:

参见结构数据MapRename

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_rename_map_return

------


### UpdateStationList

```c++
int UpdateStationList(const StationListUpdate &param);
```

#### 方法描述(异步调用):

更新停靠点列表（整个替换掉当前停靠点列表）

#### 方法参数:

参见结构数据StationListUpdate

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_update_station_list_return

------


### QueryVectorMap

```c++
int QueryVectorMap(const VecMapRequest &param);
```

#### 方法描述(异步调用):

获取当前地图的矢量图信息

#### 方法参数:

参见结构数据VecMapRequest

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_query_vecmap_return

------


### EnableLog:

```c++
static void EnableLog(LogType type, const std::string &path);
```

#### 方法描述(异步调用):

打开日志功能.默认情况下关闭日志功能

#### 方法参数:

1 enum LogType
2 {
3 LT_CONSOLE_LOG = 1,//日志输出到控制台
4 LT_FILE_LOG = 2, //日志输出到文件
5 LT_MAX,
6 };
type:日志类型, 复合选择,可以通过位或方式传递,如LT_CONSOLE_LOG |
LT_FILE_LOG
path:如果LT_FILE_LOG被指定 , 在path指定的位置上生成名为xstsdk.log的日志文
件.注意:path的末尾不需要增加反斜杠(合规的路径:./data ,违规的路径:/data/)

------


### disableLog:

```c++
static void disableLog();
```

#### 方法描述(异步调用):

关闭日志功能，

------

### FormatMap

```c++
int FormateMap(const std::string &format_name, std::string &src_path, std::string &target_path);
```

#### 方法描述:

格式化地图.

#### 方法参数:

format_name:目标格式的名称

src_path:原始地图包名称

arget_path: 格式化后的地图包名称

#### 返回值:

成功:>=0,调用序列号,仅指示异步调用成功,具体结果参照信号返回.
失败:-1,通过LastError方法获取失败原因.

#### 相关信号:

sig_update_station_list_return


## 二. 信号
------

### 信号原型:

1 signal0<> sig_disconnect

#### 信号描述:

机器人服务与SDK失去连接的时候,该信号被触发.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_unregister_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_subscribe_infomation_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_mapping_control_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_start_task_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_control_task_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_start_path_task_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_navigate_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_set_station_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_set_parameter_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_modify_path_name_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_control_localisation_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_set_localisation_result_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_control_route_recording_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_control_synchronization_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_apply_map_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_update_map__return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_delete_map_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_start_point_task_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_start_point_list_task_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1<const SimpleResponse*> sig_transfer_map_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------

### 信号原型:

1 signal1 <const VehicleState*> sig_vehicle_state;

#### 信号参数:

参考数结构数据VehicleState.

------

### 信号原型:

1 signal1<const Fuel*> sig_Fuel_state;

#### 信号参数:

参考数结构数据Fuel.

------

### 信号原型:

1 signal1<const SwitchControl*> sig_switch_state;

#### 信号参数:

参考数结构数据SwitchControl.

------

### 信号原型:

1 signal1<const TaskState*> sig_task_state;

#### 信号参数:

参考数结构数据TaskState.

------

### 信号原型:

1 signal1<const PatrolTaskState*> sig_partrol_task_state;

#### 信号参数:

参考数结构数据PatrolTaskState.

------

### 信号原型:

1 signal1<const GlobalPose*> sig_global_pose;

#### 信号参数:

参考数结构数据GlobalPose.

------

### 信号原型:

1 signal1<const ArriveStation*> sig_arrive_station;

#### 信号参数:

参考数结构数据ArriveStation.

------

### 信号原型:

1 signal1<const Turn*> sig_turn;

#### 信号参数:

参考数结构数据Turn.

------

### 信号原型:

1 signal1<const VehicleStart*> sig_vehicle_start;

#### 信号参数:

参考数结构数据VehicleStart.

------

### 信号原型:

1 signal1<const VehicleStop*> sig_vehicle_stop;

#### 信号参数:

参考数结构数据VehicleStop.

------

### 信号原型:

1 signal1<const Obstacle*> sig_obstacle;

#### 信号参数:

参考数结构数据Obstacle.

------

### 信号原型:

1 signal1<const Fault*> sig_fault;

#### 信号参数:

参考数结构数据Fault.

------

### 信号原型:

1 signal1<const TaskRoute*> sig_query_route_return;

#### 信号参数:

参考数结构数据TaskRoute.

------

### 信号原型:

1 signal1<const PathInfoResponse*> sig_qto_path_info_return;

#### 信号参数:

参考数结构数据PathInfoResponse.

------

### 信号原型:

1 signal1<const StationInfoResponse*> sig_qto_station_info_return;

#### 信号参数:

参考数结构数据StationInfoResponse.

------

### 信号原型:

1 signal1<const PatrolInfoResponse*> sig_qto_patro_info_return;

#### 信号参数:

参考数结构数据PatrolInfoResponse.

------


### 信号原型:

1 signal1<const GroupInfoResponse*> sig_qto_group_info_return;

#### 信号参数:

参考数结构数据GroupInfoResponse.

------


### 信号原型:

1 signal1<const MapInfoResponse*> sig_qto_map_info_return;

#### 信号参数:

参考数结构数据MapInfoResponse.

------


### 信号原型:

1 signal1<const SiteInfoResponse*> sig_qto_site_info_return;

#### 信号参数:

参考数结构数据SiteInfoResponse.

------


### 信号原型:

1 signal1<const ParameterSetting*> sig_query_parameter_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------


### 信号原型:

1 signal1<const StationInfoResponse*> sig_query_localisation_point_return;

#### 信号参数:

参考数结构数据StationInfoResponse.

------


### 信号原型:

1 signal1<const LocalisationResponse*> sig_query_localisation_data_return;

#### 信号参数:

参考数结构数据LocalisationResponse.

------


### 信号原型:

1 signal1<const BinaryData*> sig_query_global_map_return;

#### 信号参数:

参考数结构数据BinaryData.

------


### 信号原型:

1 signal1<const RouteRecordResponse*> sig_route_record_result;

#### 信号参数:

参考数结构数据RouteRecordResponse.

------


### 信号原型:

1 signal1<const SyncProgress*> sig_sync_progress;

#### 信号参数:

参考数结构数据SyncProgress.

------


### 信号原型:

1 signal1<const SyncState*> sig_sync_state;

#### 信号参数:

参考数结构数据SyncState.

------


### 信号原型:

1 signal1<const MapperResponse*> sig_query_map_return;

#### 信号参数:

参考数结构数据MapperResponse.

------


### 信号原型:

1 signal1<const MapperTackPoint*> sig_track_point;

#### 信号参数:

参考数结构数据MapperTackPoint.

------


### 信号原型:

1 signal1<const MapperResponse*> sig_tile_map;

#### 信号参数:

参考数结构数据MapperResponse.

------


### 信号原型:

1 signal1<const MapperCurrent*> sig_query_mapping_state_return;

#### 信号参数:

参考数结构数据MapperCurrent.

------


### 信号原型:

1 signal1<const LocalisationCurrent*> sig_query_localisation_state_return;

#### 信号参数:

参考数结构数据LocalisationCurrent.

------


### 信号原型:

1 signal1<const LocalisationResult*> sig_query_localisation_result_return;

#### 信号参数:

参考数结构数据LocalisationResult.

------


### 信号原型:

1 signal1<const RouteRecordCurrent*>
sig_query_route_recording_state_return;

#### 信号参数:

参考数结构数据RouteRecordCurrent.

------


### 信号原型:

1 signal3<int, const std::string&, const std::string&> sig_register_return;

#### 信号参数:

参考数结构数据MapperCurrent.

------


### 信号原型:

1 signal1<const SimpleResponse*> sig_apply_site_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------


### 信号原型:

1 signal1<const SimpleResponse*> sig_navigate_ex_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------


### 信号原型:

1 signal1<const LocalisationResponse*> sig_control_localisation_ex_return;

#### 信号参数:

参考数结构数据LocalisationResponse.

------


### 信号原型:

1 signal1<const SimpleResponse*> sig_control_localisating_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------


### 信号原型:

1 signal1<const NavigationSetting*> sig_query_navigation_setting_return;

#### 信号参数:

参考数结构数据NavigationSetting.

------


### 信号原型:

1 signal1<const SimpleResponse*> sig_modify_navigation_settting_return;

#### 信号参数:

参考数结构数据SimpleResponse.

------



### 三 . 结构

    enum ErrorCode
    {
        EC_SUCC = 0,
        EC_ERR,
        EC_SESSION_DISCONNECT,
        EC_SESSION_SEND_FAIL,
        EC_NO_IMP,
        EC_BUFF_OVERFLOW,
        EC_DUPLICATED_CID,
        EC_MAX
    };
    
    using namespace std;
    
    struct SimpleResponse
    {
        int32_t call_id;
        //应答结果
        int32_t result;
        //应答消息
        std::string message;
    };

//    struct StationInfo
//    {
//        string station_id;
//        string station_name;
//        //经度
//        double longitude;
//        //维度
//        double latitude;
//        //方向
//        double azimuth;

//        enum StationType
//        {
//            STATION =0,
//            CHARGE = 1,
//            PATROL = 2,
//            GRID = 10000,
//        };
//        StationType type;
//    };

    struct StationInfo
    {
        string station_id;
        string station_name;
        //经度
        double longitude;
        //维度
        double latitude;
        //方向
        double azimuth;
        enum StationType
        {
            STATION = 0,
            CHARGE = 1,
            PATROL = 2, // 不再使用
            GRID = 0x1000,// 不再使用
            GRID_CHARE = 0x1001, // 不再使用
            GRID_NO_ANGLE = 0x1010, //不再使用
            EXACT_STATION = 3,
        };
    
        StationType type;
        //楼栋数
        string building; //二级名
        //单元数
        string uint; //三级名
        //层数
        string floor; //四级名
        int32_t region_id; //子地图编码id
    
        enum StationPositionType
        {
            WGS84 = 0,
            GAUSS = 1,
        };
        std::vector<StationPositionType> position_types;
        double gauss_x;
        double gauss_y;
        //关联id:目前用于关联路网
        int32_t association_id;
    };


    struct TaskConfig
    {
        enum SpeedLevel
        {
            NORMAL = 0,
            LOW = 1,
            HIGH = 2,
        };
        SpeedLevel speed_level;
    
        enum ObstacleType
        {
            OBSTACLEAVOID = 0,
            OBSTACLESTOP = 1,
        };
        ObstacleType obstacle_type;
    };
    
    struct GridInfo
    {
        int32_t grid_height;
        int32_t grid_width;
        int32_t origin_x;
        int32_t origin_y;
        int32_t resolution;
    };
    
    struct MapInfo
    {
        string id;
        string name;
        string create_time;
        GridInfo grid_info;
    
        enum MapType
        {
            LOCAL_MAP = 0,
            REMOTE_MAP = 1,
        };
    
        MapType type ;
        //楼栋数
        string building; //二级名
        //单元数
        string uint; //三级名
        //层数
        string floor; //四级名
        int32_t region_id;//子地图编码id
    };
    
    struct MapperControl
    {
        enum ControlType
        {
            START_GMAPPER = 0,
            STOP_MAPPER = 1,
            SAVE_MAPPER = 2,
            DONOT_SAVE_MAPPER = 3,
            GET_TILE_MAP = 4, // not support
            START_INCREASE_MAPPER = 5,
            GENERATE_MAP = 6,
            ADD_CLOSURE_POINT = 7,
            MATCH_CLOSURE_POINT = 8,
            PAUSE_MAPPERR = 9,
            RESUME_MAPPER = 10,
        };
        ControlType  control_type;
        string map_name; // used when START_GMAPPER
        MapInfo info; //used when MULTI_LAYER_MAPPER
    };
    
    struct Subscribe
    {
        enum SubscribeType
        {
            subscribe = 0,//订阅
            unSubscribe=1//取消订阅
        };
        //订阅的主题
        int32_t topic;
        SubscribeType subscribe_type;
        //数据发送周期 单位ms
        int32_t cycle;
    };
    
    struct VehicleState
    {
        enum Shift
        {
            R = 0,
            P = 1,
            N = 2,
            D = 3,
        };
    
        enum DriveMode
        {
            Standby = 0,
            Manual = 1,
            Auto = 2,
        };
    
        //当前时间
        int64_t date_time;
        //车辆定位经度
        double longitude;
        //车辆定位纬度
        double latitude;
        //车辆所在海拔高度
        double altitude;
        //前轮摆角度
        double steer_angle;
        //方向角（正东逆时针）
        double azimuth;
        //档位 RPND
        Shift shift;
        //油门开度（0-100）%
        double throttle;
        //刹车力度（0-100）%
        double brake;
        //行驶速度 m/s
        double speed;
        //驾驶模式 待机 手动 自动
        DriveMode drive_mode;
    };
    
    struct Fuel
    {
        //剩余燃料
        double residual_fuel;
        //电池电压
        double voltage;
        //电流
        double ammeter;
        //电池温度
        double temperature;
        //续航里程
        double endurance;
        bool is_charging;
        //累计里程
        double odometer;
    };
    
    struct  SwitchControl
    {
        enum Switch
        {
            ON = 0,
            OFF = 1,
        };
    
        enum SwitchType
        {
            WARNINGLAMP=0,
            MAINLAMP=1,
            SUBLAMP=2,
            FOGLAMP=3,
            WHISTLE=4,
            LEFTTURNLIGHT=5,
            RIGHTTURNLIGHT=6,
        };
        Switch	switch_;
        int64_t delayed;
        SwitchType  switch_type;
    };
    
    struct TaskState
    {
        int64_t start_time;
        int32_t point_count;
        int32_t current_point_index;
        //起始站点
        StationInfo start_station;
        //结束站点
        StationInfo end_station;
    };
    
    struct PatrolTaskState
    {
        string name; // patroltask name ongoing
        string id; // patroltask id ongoing
        int32_t task_num;
        int32_t current_task_index; // start from 0
        int32_t loop_num = 5;
        int32_t current_loop_index; // start form 0
        TaskConfig config;
        enum TaskStatus
        {
            STANDBY = 0,//有任务，待接收执行命令
            RUNNING = 1, //正在执行
            PAUSING = 2,//暂停中
            STOPPED = 3, //已停止
        };
        TaskStatus status;
    };
    
    struct EulerAngle
    {
        double roll;
        double pitch;
        double yaw;
    };
    
    struct GlobalPose
    {
        //时间戳
        int64_t date_time;
    
        enum GPSState{
            POWERON=0,// 上电
            INIT=1,//  初始化
            SUCCESS=2,// 成功
        };
        GPSState gps_state;
        int32_t gps_satellite_num;
        int32_t gps_week;
        int64_t gps_millisecond;
        enum PositionStatus
        {
            NONE=0,//未定位
            SINGLE=1,// 单点定位
            PSRDIFF=2,// 差分
            RTKFLOAT=3,// 浮点定位
            RTKFIX=4,// 差分定位
            UNKNOWN=5,// 未知
        };
        PositionStatus position_status;
        EulerAngle gps_angle;
        struct GPSVelocity
        {
            double north_velocity;
            double east_velocity;
            double up_velocity;
        };
        GPSVelocity gps_velocity;
        double lng;
        double lat;
        double altitude;
    
        EulerAngle gps_angle_dev;
        double lng_dev;
        double lat_dev;
        double gauss_x;
        double gauss_y;
    };
    
    struct ArriveStation
    {
        int64_t date_time;
        StationInfo station;
    };



    struct Point3D
    {
        double x;
        double y;
        double z;
    };
    
    struct Turn
    {
        enum Direction
        {
            LEFT=0,
            RIGHT=1,
        };
        //时间
        int64_t date_time;
        //方向
        Direction direction;
        //经度
        double longitude;
        //维度
        double latitude;
        //方向
        double azimuth;
    };
    
    struct VehicleStart
    {
        int64_t date_time;
        //经度
        double longitude;
        //维度
        double latitude;
        //方向
        double azimuth;
    };
    
    struct VehicleStop
    {
        //时间
        int64_t date_time;
        //经度
        double longitude;
        //维度
        double latitude;
        //方向
        double azimuth;
    };
    
    struct Obstacle
    {
        int64_t date_time;
        double longitude;
        double latitude;
        double azimuth;
    
        struct ObstacleInfo
        {
            enum ObstacleType
            {
                PEDESTRIAN=0,
                VEHICLE=1,
            };
            ObstacleType obstacle_type;
            double x;
            double y;
            double z;
        };
        std::vector<ObstacleInfo>  obstacles;
    };


    struct Fault
    {
        enum FaultModul
        {
            SENSOR=0,//传感器
            CHASSIS=1,//底盘
            APPLICATION=2,//应用
        };
    
        enum FaultType{
            GEN=0,//产生
            RELIEVE=1,//解除
        };
    
        enum FaultLevel
        {
            DEBUG=0,//调试
            INFO=1,//消息
            NOTICE = 2,
            WARN=3,//警告
            BUG=4,
            ERROR=5,
            FAULT=6,//故障
            FAILURE=7,//致命
        };
    
        int64_t date_time;
        string fault_code;
        string fault_message;
        FaultModul fault_modul;
        FaultType fault_type;
        FaultLevel fault_leve;
        std::vector<uint8_t> fault_data;
    
    };
    
    struct TaskControl
    {
        enum ControlType
        {
            RUN = 0,
            SUSPEND = 1,
            CANCEL = 2,
            RESUME = 3,
        };
    
       ControlType  control_type;
    };
    
    struct TaskRoute
    {
        int64_t start_time;
        int64_t arrival_time;
        struct Point
        {
            double longitude;
    
            double latitude;
    
            double azimuth;
        };
        //当前位置
        Point current_point;
        //路线点集合
        std::vector<Point> route_points;
        //起始站点
        StationInfo start_station;
        //结束站点
        StationInfo end_station;
    };
    
    struct GPSLocation
    {
        double longitude;
        double latitude;
        double azimuth;
        double altitude;
    };
    
    struct PathInfo
    {
        string path_id;
        string path_name;
        int32_t point_num;
        GPSLocation start_point;
        GPSLocation end_point;
    };
    
    struct PatrolInfo
    {
        string task_name;
        string task_id;
    
        enum TaskType
        {
            STATION_TASK = 0,
            PATH_TASK = 1,
        };
        TaskType task_type;
    
        struct Task
        {
            StationInfo station;
            PathInfo path;
        }task;
    
        struct Date
        {
            // Year of date. Must be from 1 to 9999, or 0 if specifying a date without
            // a year.
            int32_t year;
            // Month of year. Must be from 1 to 12, or 0 if specifying a year without a
            // month and day.
            int32_t month;
            // Day of month. Must be from 1 to 31 and valid for the year and month, or 0
            // if specifying a year by itself or a year and month where the day is not
            // significant.
            int32_t day;
            int32_t hour;
            int32_t min;
        };
        Date date;
        enum LoopType
        {
            CYCLE = 0,
            ROUND = 1, // go there and back
        };
        LoopType loop_type;
        int32_t loop_time;
        TaskConfig config;
    };
    
    struct GroupInfo
    {
        string task_name;
        string task_id;
        struct SubTask
        {
           enum TaskType
            {
                STATION_TASK = 0,
                PATH_TASK = 1,
            };
            TaskType task_type;
            struct Task
            {
                StationInfo station;
                PathInfo path;
            } task;
            TaskConfig config;
            int32_t wait_time; // wait time in min
        };
        std::vector<SubTask> sub_task;
    
        struct Date
        {
            // Year of date. Must be from 1 to 9999, or 0 if specifying a date without
            // a year.
            int32_t year;
            // Month of year. Must be from 1 to 12, or 0 if specifying a year without a
            // month and day.
            int32_t month;
            // Day of month. Must be from 1 to 31 and valid for the year and month, or 0
            // if specifying a year by itself or a year and month where the day is not
            // significant.
            int32_t day;
            int32_t hour;
            int32_t min;
        };
        Date date;
        int32_t loop_time;
        double last_hours;
        Date end_time;
    };
    
    struct SiteInfo
    {
        string id;
        string name;
        int32_t zone;
    };
    
    struct GetTaskObject
    {
        enum TaskObjectType
        {
            STATION_INFO = 0,
            PATH_INFO = 1,
            MAP_POINT_INFO = 2, // not used
            PATROL_INFO = 3,
            GROUP_INFO = 4,
            MAP_INFO = 5,
            SITE_INFO = 6,
        };
        TaskObjectType  object_type;
    };
    
    struct StationInfoResponse
    {
        std::vector<StationInfo> station;
    };
    
    struct PathInfoResponse
    {
        std::vector<PathInfo> path;
    };
    
    struct MapInfoResponse
    {
        std::vector<MapInfo> map;
    };


    struct PatrolInfoResponse
    {
        string name;
        string id;
        std::vector<PatrolInfo> patrol;
    };
    
    struct GroupInfoResponse
    {
        string name;
        string id;
        std::vector<GroupInfo> patrol;
    };
    
    struct SiteInfoResponse
    {
        std::vector<SiteInfo> site;
    };
    
    struct Pose2D
    {
        double x;
        double y;
        double raw;
    };


    struct NavigationPoint
    {
        Pose2D pose;
        GPSLocation GPS_location;
    };
    
    struct ParameterSetting
    {
        enum SpeedLevel
        {
            NORMAL = 0,
            LOW = 1,
            HIGH = 2,
        };
    
        struct SpeedSetting
        {
            SpeedLevel level;
            double speed;
        };
    
        std::vector<SpeedSetting> speed_setting;
    };


    struct LocalisationResponse
    {
        Pose2D pos;
    
        struct Range
        {
            double min_x;
            double min_y;
            double max_x;
            double max_y;
            int32_t grid_size;
        };
        Range range;
        std::vector<uint8_t> point_cloud;
        std::vector<uint8_t> map_body;
        int32_t quality;
    };
    
    struct LocalisationResult
    {
        bool result; //定位结果 true 好 false 不好
        Pose2D pose;
    };
    
    struct BinaryData
    {
        std::vector<uint8_t> binary;
    };
    
    struct RouteRecordControl
    {
        enum ControlType
        {
            START_RECORD = 0,
            STOP_RECORD = 1,
            SAVE_RECORD = 2,
            PAUSE_RECORD = 3,
            RESUME_RECORD = 4,
        };
        ControlType  control_type;
    };
    
    struct RouteRecordResponse
    {
        int32_t index;
        Pose2D pose;
        GPSLocation GPS_location;
    };
    
    struct SyncControl
    {
        enum ControlType
        {
            UPLOAD = 0,
            DOWNLOAD = 1,
            CANCEL = 2,
        };
    
       ControlType  control_type;
    
       enum DataType
        {
            //矢量图
            VECTOR = 0,
            //瓦片图
            MAP_TILE = 1,
            HDMAP = 2,
            MAP_3D = 3,
            MAP_2D = 4,
            //轨迹文件
            POSE = 5,
            //路网
            PATH_NET = 6,
        };
    
       std::vector<DataType> data_types;
    };
    
    struct SyncProgress
    {
        struct DataProgress
        {
            double total;
            double current;
        };
    
        std::vector<DataProgress> progress;
        double total;
        double current;
    };
    
    struct SyncState
    {
        enum State
        {
            ERROR = 0,
            COMPLETE = 1,
        };
        State  state;
        string error_code;
        string error_msg;
    };
    
    struct MapperResponse
    {
        struct MapRange
        {
            double min_x;
            double min_y;
            double max_x;
            double max_y;
            int32_t grid_size;
        };
    
        MapRange range;
        std::vector<uint8_t> map_body;
    };
    
    struct MapperTackPoint
    {
        int32_t index;
        Pose2D pose;
        GPSLocation GPS_location;
    };
    
    struct MapperProgress
    {
        enum ControlType
        {
            UNKOWN = 0,
            GENERATE_MAP = 6,
        };
    
        ControlType type;
    
        enum Result
        {
            FAILED = 0,
            SUCCESS = 1,
            ONGOING = 2,
        };
        Result result;
        string result_msg;
        int32_t progess; // 0~100
    };
    
    struct GridPoint
    {
        string id;
        string name;
        string map_name;
        Point3D position;
        EulerAngle angle;
    };
    
    struct GridPointList
    {
        std::vector<GridPoint> point;
    };
    
    struct MapperCurrent
    {
        enum MapStatus
        {
            NORMAL = 0,
            START_MAP = 1,
            STOP_MAP = 2,
            GENERATE_MAP = 3,
            PAUSE_MAP = 4,
            RESUME_MAP = 5,
        };
        MapStatus  status;
    };
    
    struct LocalisationCurrent
    {
        enum LocalisationStatus
        {
            NORMAL = 0,
            START_LOCAL = 1,
            STOP_LOCAL = 2,
            PAUSE_LOCAL = 3,
            RESUME_LOCAL = 4,
        };
       LocalisationStatus  status;
    };
    
    struct RouteRecordCurrent
    {
        enum RouteRecordStatus
        {
            NORMAL = 0,
            START = 1,
            STOP = 2,
            PAUSE = 3,
            RESUME = 4,
        };
       RouteRecordStatus  status;
    };


```c++
struct NavigationPointEx
{
    int mode;  //模式: 0非路线模式 1路线模式
    int zone;  //条带
    std::string path_name; //路线名称
    std::string point_name; //目的点名
    std::vector<Pose2D> dest_points;
};

struct LocalisationControl
{
    enum ControlType
    {
        LOCAL_BY_GPS = 0,
        LOCAL_BY_STATION = 1,
        LOCAL_MANUAL = 2,
        LOCAL_CHECK = 3,
        LOCAL_MANUAL_NO_GPS = 4,
        LOCAL_PAUSE = 5,
        LOCAL_RESUME = 6,
        LOCAL_STOP = 7,
    };
    ControlType  control_type;
    StationInfo station;
};

struct LocalisationControlEx
{
    enum ControlType
    {
        LOCAL_BY_GPS = 0,
        LOCAL_BY_STATION = 1,
        LOCAL_MANUAL = 2,
        LOCAL_CHECK = 3,
        LOCAL_MANUAL_NO_GPS = 4,
        LOCAL_PAUSE = 5,
        LOCAL_RESUME = 6,
        LOCAL_STOP = 7,
    };
    ControlType  control_type;  //控制类型
    std::string point_name;  //定位点名称
    Point3D point;  //定位点坐标
};

struct LocalisatingControl
{
    enum ControlType
    {
        LOCAL_PAUSE = 0,
        LOCAL_RESUME = 1,
        LOCAL_STOP = 2,
    };
    ControlType  control_type;  //控制类型
};

struct TaskRealStatus
{
    TaskConfig config; // task config
    enum Status
    {
        STANDBY = 0, //有任务，待接收执行命令
        RUNNING = 1, //正在执行
        PAUSING = 2, //暂停中
        STOPPED = 3, //已停止 此时：code为0，则导航成功; code为1，则导航失败;
    };
    struct RealStatus
    {
        Status status;
        int32_t code; //错误码：0为成功，其他为错误码
        string error_msg ; //错误消息内容

    };
    RealStatus status;
};

struct NavigationState
{
    //当前时间
    int64_t date_time;
    //车辆定位经度
    double longitude;
    //车辆定位纬度
    double latitude;
    //车辆所在海拔高度
    double altitude;
    //方向角（正东逆时针）
    double azimuth;
    //高斯
    double global_x;
    double global_y;
    //当前地图名
    string map_name;
    //当前站点名
    string site_name;
    int32_t zone;
    //实时速度
    double speed;
};

struct NavigationSetting
{
    enum Key
    {
        SAFE_DISTACNE = 0,
        URGENCY_STOP_DISTANCE = 1,
        OBSTACLE_RADIUS = 2,
        HEADBACK_ALLOW = 3, //不需要value
        HEADBACK_NOT_ALLOW = 4, //不需要value
        OBSTACLE_AVOID = 5, //壁障，不需要value
        OBSTACLE_STOP = 6, //停障，不需要value
        CURRENT_SPEED_LOW = 7,//应用低速，不需要value
        CURRENT_SPEED_NORMAL = 8,//应用正常速度，不需要value
        CURRENT_SPEED_HIGH = 9,//应用高速，不需要value
    };

    struct Setting
    {
        //ENUM
        Key key;
        //value 可选
        double value;
    };

    std::vector<Setting> setting;
};

struct GeneralKey
{
    string name;//可表示地图名、站点名等
};

struct MapRequest
{
    enum ControlType
    {
        DOWNLOAD = 0,
        UPLOAD = 1,
    };
    ControlType control_type;
    string address;
    string name;
};

struct MapRequestEx
{
    enum ControlType
    {
        DOWNLOAD = 0,
        UPLOAD = 1,
    };
    ControlType control_type;
    string address;
    MapInfo map_info;

    enum Entity{
        ETY_2D = 0,
        ETY_3D = 1,
        ETY_KEYFRAME = 3,
        ETY_MAPTITLE = 4,
        ETY_PATHNET = 5,
        ETY_POSE = 6,
        ETY_VECTOR = 7,
        ETY_ALL = 8,
    };
    std::vector<Entity> entities;
};

struct MapRename
{
    string old_name;
    string new_name;
};

struct StationListUpdate
{
    string map_name;
    std::vector<StationInfo> stationinfos;
};

struct VecJunction
{
    int32_t id;				//唯一标识符
    std::vector<Point3D> vertexes;
};

struct VecParking
{
    int32_t id;				//唯一标识符
    int32_t on_road;			//预留
    std::vector<Point3D> vertexes;  //停车位只存在4个点， 在[0][1][2][3]4个点中， [0]和[3]为车位的入口
};

struct VecZerbraline
{
    int32_t id;				//唯一标识符
    int32_t on_road;			//预留
    std::vector<Point3D> vertexes;
};

struct VecObstacle
{
    int32_t id;				//唯一标识符
    int32_t on_road;			//预留
    int32_t attr;				//预留
    int32_t order_num;		//预留
    std::vector<Point3D> vertexes;
};

struct VecSlopeArea
{
    int32_t id = 1;				//唯一标识符
    std::vector<Point3D> vertexes;
};

struct VecLanemarking
{
    int32_t id;			//道路id
    int32_t marking;		//线序
    int32_t attribute;	//线属性
    int32_t type;			//预留
    int32_t function;		//预留
    int32_t direct_l;		//预留
    int32_t direct_r;		//预留
    int32_t direct_s;		//预留
    int32_t direc_u;		//预留
    int32_t min_speed;	//预留
    int32_t max_speed;	//预留
    std::vector<Point3D> vertexes;
};

struct VecRoadline
{
    int32_t id;					//唯一标识符
    int32_t lane;					//关联道路id
    int32_t line;					//起始车道线
    int32_t midlane_id;			//预留
    int32_t direction;			//预留
    int32_t max_speed;			//最大显示  千米/时
    int32_t marking;				//转向枚举
    int32_t follow_gps;			//跟线模式
    int32_t gps_direct;			//跟线方向
    int32_t class_;				//道路类别
    int32_t type;				//道路类型
    std::vector<Point3D> vertexes;
};

struct VecMap
{
    std::vector<VecJunction> junctions; 			//路口
    std::vector<VecParking> parkings; 			//停车位
    std::vector<VecZerbraline> zerbraline; 		//斑马线
    std::vector<VecObstacle> obstalce; 			//障碍（虚拟墙）
    std::vector<VecSlopeArea> slopeareas; 		//斜铺区域
    std::vector<VecLanemarking> lanemarkings; 	//道路
    std::vector<VecRoadline> roadlines; 			//路网
};

struct VecMapRequest
{
    MapInfo map_desc;	//如果地图名称为空，则获取当前地图矢量地图，反之获取指定地图下的矢量地图
};
```
