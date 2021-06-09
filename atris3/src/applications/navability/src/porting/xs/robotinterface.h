



#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H


#include "sdkcommom.h"
#include <vector>
#include "sigslot.h"
#include <queue>
#include <mutex>
#include <memory>
#include <map>
#include <unordered_map>
#include <functional>


namespace ugv_sdk
{
    class RemoteSession;
    class TransProtocol;
    class SocketClient;
};


namespace xs_sdk
{
    class RemoteSession;

    using namespace sigslot;

    class XS_TECH RobotInterface
    {
    public:
        enum LogType
        {
            LT_CONSOLE_LOG = 1,
            LT_FILE_LOG = 2,
            LT_MAX,
        };

        static void EnableLog(LogType type, const std::string &path);
        static void disableLog();
        static const char* ErrorString(ErrorCode code);

    public:
        RobotInterface();
        ~RobotInterface();

    public:
        bool Connect(const std::string ip, uint16_t port);
        void Disconnect();

        //this function should be called in the main loop of your program.
        //if max_event_count <= 0, all events will be processed;.
        int ProcessEvent(int max_event_count = 0);

        ErrorCode LastError()const;

        //revalent signals:
        //--sig_register_return
        //return value:
        //--On success, false is returned(>=0).
        //--On error, true is returned, and last error is set appropriately.
        bool RegisterMe(int64_t appid=0, const std::string &sercet=std::string());

        //revalent signals:
        //--sig_unregister_return
        //return value:
        //--On success, false is returned(>=0).
        //--On error, true is returned, and last error is set appropriately.
        bool UnregisterMe();

        //revalent signals:
        //--sig_query_parameter_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryParameter();

        //revalent signals:
        //--sig_query_route_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryRoute();

        //revalent signals:
        //--sig_mapping_control_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int MappingControl(const MapperControl& param);

        //revalent signals:
        //--sig_query_map_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryMap();

        //revalent signals:
        //--sig_subscribe_infomation_return
        //--topic=0,sig_vehicle_state
        //--topic=1,sig_Fuel_state
        //--topic=2,sig_arrive_station
        //--topic=3,sig_turn
        //--topic=4,sig_vehicle_start
        //--topic=5,sig_vehicle_stop
        //--topic=6,sig_obstacle
        //--topic=7,sig_fault
        //--topic=8,sig_switch_state
        //--topic=9,sig_task_state
        //--topic=10,sig_partrol_task_state
        //--topic=11,sig_global_pose
        //--topic=13,sig_navigation_state
        //--topic=27,sig_commom_state
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int SubscribeInfomation(const Subscribe& param);

        //revalent signals:
        //--sig_start_task_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int StartTask(const StationInfo& param);

        //revalent signals:
        //--sig_start_path_task_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int StarPathTask(const PathInfo& param);

        //revalent signals:
        //--sig_control_task_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int ControlTask(const TaskControl &param);

        //revalent signals:
        //--sig_navigate_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int Navigate(const NavigationPoint &param);

        //revalent signals:
        //--sig_qto_path_info_return
        //--sig_qto_station_info_return
        //--sig_qto_group_info_return;
        //--sig_qto_map_info_return;
        //--sig_qto_site_info_return;
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryTaskObject(const GetTaskObject &param);

        //revalent signals:
        //--sig_set_station_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int SetStation(const StationInfo &param);

        //revalent signals:
        //--sig_set_parameter_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int SetParameter(const ParameterSetting &param);

        //revalent signals:
        //--sig_modify_path_name_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int ModifyPathName(const PathInfo &param);

        //revalent signals:
        //--sig_control_localisation_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
//        int ControlLocalisation(const LocalisationControl &param);

        //revalent signals:
        //--sig_query_localisation_point_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryLocalisationPoint();

        //revalent signals:
        //--sig_query_localisation_data_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryLocalisationData(const LocalisationControl &param);

        //revalent signals:
        //--sig_set_localisation_result_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int SetLocalisationResult(const LocalisationResult &param);

        //revalent signals:
        //--
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryGlobalMap();

        //revalent signals:
        //--sig_control_route_recording_return
        //--sig_route_record_result
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int ControlRouteRecording(const RouteRecordControl &param);

        //revalent signals:
        //--sig_control_synchronization_return
        //--sig_sync_progress
        //--sig_sync_state
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int ControlSynchronization (const SyncControl &param);

        //revalent signals:
        //--sig_apply_map_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int ApplyMap(const MapInfo &param);

        //revalent signals:
        //--sig_update_map__return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int UpdateMap(const MapInfo &param);

        //revalent signals:
        //--sig_delete_map_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int DeleteMap(const MapInfo &param, bool all = false);

        //revalent signals:
        //--sig_start_point_task_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        //int StartPointTask(const GridPoint &param);

        //revalent signals:
        //--sig_start_point_list_task_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        //int StartPointListTask(const GridPointList &param);

        //revalent signals:
        //--sig_query_mapping_state_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryMappingState();

        //revalent signals:
        //--sig_query_localisation_state_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryLocalisationState();

        //revalent signals:
        //--sig_query_localisation_result_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryLocalisationResult();

        //revalent signals:
        //--sig_query_route_recording_state_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryRouteRecordingState();

        //revalent signals:
        //--sig_transfer_map_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int TransferMap(const MapRequest& param);

        //revalent signals:
        //--sig_transfer_map_ex_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int TransferMapEx(const MapRequestEx& param);


        //revalent signals:
        //--sig_apply_site_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int ApplySite(const SiteInfo& param);

        //revalent signals:
        //--sig_navigate_ex_return
        //--sig_task_real_status
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int NavigateEx(const NavigationPointEx &param);


        //revalent signals:
        //--sig_control_localisation_ex_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int ControlLocalisationEx(const LocalisationControlEx &param);

        //revalent signals:
        //--sig_control_localisating_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int ControlLocalisating(const LocalisatingControl &param);

        //revalent signals:
        //--sig_query_navigation_setting_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryNavigationSettting();

        //revalent signals:
        //--sig_modify_navigation_settting_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int ModifyNavigationSettting(const NavigationSetting& param);

        //revalent signals:
        //--sig_rename_map_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int RenameMap(const MapRename& param);

        //revalent signals:
        //--sig_update_station_list_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int UpdateStationList(const StationListUpdate &param);

        //revalent signals:
        //--sig_query_vecmap_return
        //return value:
        //--On success, the id of calling is returned(>=0).
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryVectorMap(const VecMapRequest &param);

        //The method is not asynchronous.
        //return value:
        //--On success, 0 is returned.
        //--On error, -1 is returned, and last error is set appropriately.
        int FormatMap(const std::string &foramt_name, std::string &src_path, std::string &target_path);

        //return value:
        //--sig_query_map_return
        //return value;
        //--On success, 0 is returned.
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryMapEx(const MapInfo &param);

        //return value:
        //--sig_tsp_navigation_return
        //return value;
        //--On success, 0 is returned.
        //--On error, -1 is returned, and last error is set appropriately.
        int TspNavigate(const TspNavigationRequest &param);

        //return value:
        //--sig_query_station_ex_return
        //return value;
        //--On success, 0 is returned.
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryStationEx(const MapInfo &param);

        //return value:
        //--sig_query_version_return
        //return value;
        //--On success, 0 is returned.
        //--On error, -1 is returned, and last error is set appropriately.
        int QueryVersion();

        //return value:
        //--sig_start_station_localisation
        //return value;
        //--On success, 0 is returned.
        //--On error, -1 is returned, and last error is set appropriately.
        int StartStationLocalisation(const StationLocalisation &param);

        //return value:
        //--sig_upate_dynamic_obstacle_return
        //return value;
        //--On success, 0 is returned.
        //--On error, -1 is returned, and last error is set appropriately.
        int UpdateDynamicObstacle(const DynamicObstacleUpdate &param);

        //return value:
        //--sig_add_station_return
        //return value;
        //--On success, 0 is returned.
        //--On error, -1 is returned, and last error is set appropriately.
        int AddStation(const StationInfo &param);

        //return value:
        //--sig_modify_station_return
        //return value;
        //--On success, 0 is returned.
        //--On error, -1 is returned, and last error is set appropriately.
        int ModifyStation(const StationInfo &param);

        //return value:
        //--sig_remove_station_return
        //return value;
        //--On success, 0 is returned.
        //--On error, -1 is returned, and last error is set appropriately.
        int RemoveStation(const StationInfo &param);


    public:
        signal0<> sig_disconnect;

        signal1<const SimpleResponse*> sig_unregister_return;
        signal1<const SimpleResponse*> sig_subscribe_infomation_return;
        signal1<const SimpleResponse*> sig_mapping_control_return;
        signal1<const SimpleResponse*> sig_start_task_return;
        signal1<const SimpleResponse*> sig_control_task_return;
        signal1<const SimpleResponse*> sig_start_path_task_return;
        signal1<const SimpleResponse*> sig_navigate_return;
        signal1<const SimpleResponse*> sig_set_station_return;
        signal1<const SimpleResponse*> sig_set_parameter_return;
        signal1<const SimpleResponse*> sig_modify_path_name_return;
        signal1<const SimpleResponse*> sig_control_localisation_return;
        signal1<const SimpleResponse*> sig_set_localisation_result_return;
        signal1<const SimpleResponse*> sig_control_route_recording_return;
        signal1<const SimpleResponse*> sig_control_synchronization_return;
        signal1<const SimpleResponse*> sig_apply_map_return;
        signal1<const SimpleResponse*> sig_update_map__return;
        signal1<const SimpleResponse*> sig_delete_map_return;
        signal1<const SimpleResponse*> sig_start_point_task_return;
        signal1<const SimpleResponse*> sig_start_point_list_task_return;
        signal1<const SimpleResponse*> sig_transfer_map_return;
        signal1<const SimpleResponse*> sig_transfer_map_ex_return;
        signal1<const SimpleResponse*> sig_apply_site_return;
        signal1<const SimpleResponse*> sig_navigate_ex_return;
        signal1<const SimpleResponse*> sig_control_localisating_return;
        signal1<const SimpleResponse*> sig_modify_navigation_settting_return;
        signal1<const SimpleResponse*> sig_rename_map_return;
        signal1<const SimpleResponse*> sig_update_station_list_return;
        signal1<const SimpleResponse*> sig_start_station_localisation;
        signal1<const SimpleResponse*> sig_upate_dynamic_obstacle_return;
        signal1<const SimpleResponse*> sig_modify_station_return;
        signal1<const SimpleResponse*> sig_remove_station_return;

        signal1<const VehicleState*> sig_vehicle_state;
        signal1<const Fuel*> sig_Fuel_state;
        signal1<const SwitchControl*> sig_switch_state;
        signal1<const TaskState*> sig_task_state;
        signal1<const PatrolTaskState*> sig_partrol_task_state;
        signal1<const GlobalPose*> sig_global_pose;
        signal1<const ArriveStation*> sig_arrive_station;
        signal1<const Turn*> sig_turn;
        signal1<const VehicleStart*> sig_vehicle_start;
        signal1<const VehicleStop*> sig_vehicle_stop;
        signal1<const Obstacle*> sig_obstacle;
        signal1<const Fault*> sig_fault;
        signal1<const TaskRoute*> sig_query_route_return;
        signal1<const PathInfoResponse*> sig_qto_path_info_return;
        signal1<const StationInfoResponse*> sig_qto_station_info_return;
        signal1<const PatrolInfoResponse*> sig_qto_patro_info_return;
        signal1<const GroupInfoResponse*> sig_qto_group_info_return;
        signal1<const MapInfoResponse*> sig_qto_map_info_return;
        signal1<const SiteInfoResponse*> sig_qto_site_info_return;
        signal1<const ParameterSetting*> sig_query_parameter_return;
        signal1<const StationInfoResponse*> sig_query_localisation_point_return;
//        signal1<const LocalisationResponse*> sig_query_localisation_data_return;
        signal1<const BinaryData*> sig_query_global_map_return;
        signal1<const RouteRecordResponse*> sig_route_record_result;
        signal1<const SyncProgress*> sig_sync_progress;
        signal1<const SyncState*> sig_sync_state;
        signal1<const MapperResponse*> sig_query_map_return;
        signal1<const MapperTackPoint*> sig_track_point;
        signal1<const MapperResponse*> sig_tile_map;
        signal1<const MapperProgress*> sig_mapping_progress;
        signal1<const MapperCurrent*> sig_query_mapping_state_return;
        signal1<const LocalisationCurrent*> sig_query_localisation_state_return;
        signal1<const LocalisationResult*> sig_query_localisation_result_return;
        signal1<const RouteRecordCurrent*> sig_query_route_recording_state_return;
        signal1<const LocalisationResponse*> sig_control_localisation_ex_return;
        signal1<const TaskRealStatus*> sig_task_real_status;
        signal1<const NavigationState*> sig_navigation_state;
        signal1<const NavigationSetting*> sig_query_navigation_setting_return;
        signal1<const SyncProgress*> sig_map_transfer_progress;
        signal1<const VecMap*> sig_query_vecmap_return;
        signal1<const MapperResponse*> sig_query_map_ex_return;
        signal1<const TspNavigationResponse*> sig_tsp_navigation_return;
        signal1<const StationInfoResponse*> sig_query_station_ex_return;
        signal1<const VersionResponse*> sig_query_version_return;
        signal1<const CommonState*> sig_commom_state;
        signal1<const StationInfo*> sig_add_station_return;


        signal3<int, const std::string&, const std::string&> sig_register_return;

    protected:
        void DealWith(int msg_id, const std::vector<uint8_t> &data);
        void CheckPool();
        void CheckAlive();
        bool SendHeartBeat();


    private:
        ErrorCode last_error_;
        std::shared_ptr<RemoteSession> session_;

        struct SimpleResponseHodler
        {
            time_t time;
            std::function<void(const SimpleResponse*)> call_back_func;
        };
        std::map<int, SimpleResponseHodler> greponse_handler;
        std::unordered_map<int, std::function<void(int msg_id, const std::vector<uint8_t> &data)> > msg_handler_;
        int cid_;
        time_t cur_time_point,last_send_hb_time_;
        enum
        {
            WAIT_RECV_HB,
            WAIT_SEND_HB,
        }hb_state_;
    };
}

#endif



