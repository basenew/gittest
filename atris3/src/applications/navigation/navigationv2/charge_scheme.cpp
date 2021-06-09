#include "charge_scheme.h"

#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_chg]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_chg]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_chg]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_chg]"#format, ##args)

namespace nav{

bool Charge::_construct_task(){
   // Linfo("return point:%s(%lf,%lf,%lf)", _ret_pt.np.name.c_str(), _ret_pt.np.pos.x, _ret_pt.np.pos.y, _ret_pt.np.pos.angle);
    if (_tf.empty()){
        _to_out_ret_pt_tsk.point(_out_ret_pt);
        _to_in_ret_pt_tsk.point(_in_ret_pt);
        _to_chg_pt_tsk.point(_chg_pt);
        _to_out_ret_pt_tsk.front(&_to_in_ret_pt_tsk);
        _to_in_ret_pt_tsk.front(&_to_chg_pt_tsk);
        _tf.push(&_to_out_ret_pt_tsk);
        _tf.push(&_to_in_ret_pt_tsk);
        _tf.push(&_to_chg_pt_tsk);
    }
    else{
        _to_out_ret_pt_tsk.point(_out_ret_pt);
        _to_in_ret_pt_tsk.point(_in_ret_pt);
        _to_chg_pt_tsk.point(_chg_pt);
    }

    return !_tf.empty();
}

void Charge::_clear(){
    Linfo("%s clear", _name.c_str());
    _chg_pt = {};
    _in_ret_pt = {};
    _out_ret_pt = {};
}

bool Charge::check_battery(int level){
    Linfo("%s level:%d", __FUNCTION__, level);
    TASK_LOCK();
    shm::ChargeState shm_state;
    shm::iMemory_read_ChargeState(&shm_state);
    Linfo("%s: charge state: %d ", __FUNCTION__, shm_state.state);
    if (is_active() || (shm_state.state != dockSDKState::IDLE)){
        Linfo("%s charge already running, charge state: %d ", __FUNCTION__, shm_state.state);
        return false;
    }

    if (_batterys && _batterys >= level){
        Linfo("%s notify to pc battery level:%d", __FUNCTION__, level);
        Notifier::publish_recharge_battery(level);
        return Config::get_instance()->nav_auto_charge_self && _batterys;
    }

    return false;
}

bool Charge::is_charge_pt_valid(void)
{
/*
    vector<GsNavPoint> points;
    unsigned int i;
    string def_charge_pt_name = "charge";

    string map_name = MapManager::get_instance().get_using_map();

    if(map_name.empty())
    {
        Lerror("%s map name invalid",__FUNCTION__);
        return false;
    }
    Linfo("%s map name = %s\r\n",__FUNCTION__, map_name.c_str());
    int ret = GsApi::get_instance()->map_get_point_list(map_name, points, 1);
    if (ret == ERR_OK)
    {
        Linfo("charge point size: %d",points.size());
        if(_using_pc_charge_pt_name) // if it is using pc setted charge point name
        {
            Lwarn("using pc setted charge point name, charge point name : %s",charge_point_name().c_str());
            for (i = 0; i < points.size(); i++) 
            {
                Linfo("pc setted charge point : %d, name : %s", i, points[i].name.c_str());
                if (points[i].name == charge_point_name())
                {
                    Linfo("found pc setted charge point name in the map point list, pc setted charge point name : %s", points[i].name.c_str());
                    return true;
                }
            }

            Lerror("does not find pc setted charge point on map : %s!!!!",map_name.c_str());
        }
        else
        {
            Lwarn("using default charge point name");
            // using default charge point name
            for (i = 0; i < points.size(); i++) 
            {
                Linfo("charge point : %d, name : %s", i, points[i].name.c_str());
                if (points[i].name == def_charge_pt_name) 
                {
                    Linfo("found default charge point name in the map point list, dafault charge point name : %s", def_charge_pt_name.c_str());
                    return true;
                }
            }
            
            Lerror("does not have charge point on map : %s!!!!",map_name.c_str());
        }
    }

    return false;
*/
    return true;
}

}
