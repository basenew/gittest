#pragma once
#include <iostream>
#include <json/json.h>

#include "config/config.h"
#include "nav_error_code.h"
//#include "navigation.h"
#include "task_node.h"
#include "task_flow.h"
#include "gs_api.h"
#include "task.h"
#include "navigation.h"
#include "scheme.h"
#include "imemory/atris_imemory_api.h"
#include "platform/udock/UdockData.h"


#undef Linfo
#undef Lwarn
#undef Lerror
#undef Ldebug
#define Linfo(format, args...)  atrislog::mtrace(ros::console::levels::Info,  "[nav_chg]"#format, ##args)
#define Lwarn(format, args...)  atrislog::mtrace(ros::console::levels::Warn,  "[nav_chg]"#format, ##args)
#define Lerror(format, args...) atrislog::mtrace(ros::console::levels::Error, "[nav_chg]"#format, ##args)
#define Ldebug(format, args...) atrislog::mtrace(ros::console::levels::Debug, "[nav_chg]"#format, ##args)
using namespace std;
using namespace Json;

namespace nav{
using string_vct= vector<string>;

class Charge:public Scheme{
public:
    Charge()
    :Scheme("charge", Task::TASK_CHARGE)
    ,_to_in_ret_pt_tsk("in_ret_pt", Task::TASK_CHARGE)
     ,_to_out_ret_pt_tsk("out_ret_pt", Task::TASK_RETURN_CHARGER_DOOR)
    ,_chg_enable(true)
    ,_using_pc_charge_pt_name(false)
    ,_batterys(Config::get_instance()->nav_auto_charge_battery)
    {
        _tf.name("charge");
    };
    virtual ~Charge(){
    };

    virtual int start(){
        Linfo("charge start ********************************************************************************!!");
        TASK_LOCK();
        _batterys = Config::get_instance()->nav_auto_charge_battery;
        return _chg_enable ? _next_status(STA, lock):ERR_CHARGE_DISABLED;
    };

    inline void set_pc_charge_pt_name(bool is_using_pc_charge_pt_name){_using_pc_charge_pt_name = is_using_pc_charge_pt_name;};

    inline void return_point(const GsNavPoint& ret_pt){_in_ret_pt = ret_pt;};
    inline void out_return_point(const GsNavPoint& ret_pt){_out_ret_pt = ret_pt;};
    inline const GsNavPoint& return_point(){return _in_ret_pt;};
    inline const GsNavPoint& out_return_point(){return _out_ret_pt;};

    inline void charge_point(const GsNavPoint& chg_pt){_chg_pt = chg_pt;};
    inline const GsNavPoint& charge_point(){return _chg_pt;};

    inline void charge_point_name(const string& chg_pt_name){_chg_pt.np.name = chg_pt_name;};
    inline const string& charge_point_name(){return _chg_pt.np.name;};
    
    inline bool charge_enable(){return _chg_enable;};     
    inline void charge_enable(bool chg_enable){_chg_enable = chg_enable;};     

    inline virtual int parse(const string &scheme_name, Value& scheme){return ERR_OK;};//todo no need

    inline virtual void on_finished(){
        _batterys = Config::get_instance()->nav_auto_charge_battery;
    };

    bool check_battery(int level);
    bool is_charge_pt_valid(void);
    
    bool is_active(){
        return Task::is_active();
    };
protected:
    virtual bool _construct_task();
    virtual void _clear();

private:
    bool _chg_enable;

    int     _batterys;
    GsNavPoint     _in_ret_pt;
    GsNavPoint     _out_ret_pt;
    GsNavPoint     _chg_pt;
    NavPointTask    _to_in_ret_pt_tsk;
    NavPointTask    _to_out_ret_pt_tsk;
    NavChgPointTask _to_chg_pt_tsk;
    bool _using_pc_charge_pt_name;
};


}

