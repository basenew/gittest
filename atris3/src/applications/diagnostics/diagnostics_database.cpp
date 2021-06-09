#include "diagnostics.h"
#include "imemory/atris_imemory_api.h"
#include "database/sqliteengine.h"
#include "diag_helper.h"

#define DIAG_EVENT_TABLE_COLUMN 5
//#define MAXIMUM_DIAG_EVENT_LIMITS  50

#define DIAG_EVENT_STORE_TABLE_COLUMN 3
//#define MAXIMUM_DIAG_EVENT_STORE_LIMITS  50

#define DIAG_EVENT_TABLE "CREATE TABLE IF NOT EXISTS [diag_event] (" \
    "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
    "[eventContent] TEXT," \
    "[status] TEXT," \
    "[serialNum] TEXT," \
    "[timestamp] TEXT)" 

#define DIAG_EVENT_STORED_TABLE "CREATE TABLE IF NOT EXISTS [diag_store_event] (" \
    "[id] INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT," \
    "[title] TEXT," \
    "[event_store] TEXT)"


// create diag event data base if not exist
// initialize database
bool Diagnostics::initDiagDataBase(void)
{
    char **result;
    int row, column;
    int retry_times = 3;
    int iRet;

    log_info("%s called", __FUNCTION__);

    do
    {
        iRet = SqliteEngine::execSQL(DIAG_EVENT_TABLE, SQLITE_DIAG_EVENT_FILE);
        if(iRet != SQLITE_OK)
        {
            log_info("%s create diag event table failed!!! iRet = %d , retry times = %d", __FUNCTION__, iRet, retry_times);
            reinitDiagDataBase();
        }
        else
        {
            log_info("initialize diag event database table success ...iRet = %d", iRet);
            break;
        }

        retry_times--;
        sleep(1);
    }while(retry_times != 0);

    if(retry_times == 0)
    {
        // we initialize failed here , do not need to do the following steps since we create database failed
        log_error("init data base failed after three times!!!");
        return false;
    }
    else
    {
    	log_info("%s retry times not exceeded , retry times = %d", __FUNCTION__, retry_times);
    }

    // from here the data base file exist
    iRet = SqliteEngine::query("SELECT * FROM diag_event", &result, &row, &column, SQLITE_DIAG_EVENT_FILE);
    //log_info("%s check if data base format ok , SELECT * FROM diag_event row = %d , column = %d\r\n", __FUNCTION__, row, column);
    if(iRet == SQLITE_OK)
    {
        if (row > 0 && column != DIAG_EVENT_TABLE_COLUMN) 
        {
            // query data base 
            // data base format wrong
            log_error("%s diag event data base error!!!", __FUNCTION__);
            log_error("%s database format wrong column shoud be %d , but row = %d , column = %d\r\n", __FUNCTION__, DIAG_EVENT_TABLE_COLUMN, row , column);
            SqliteEngine::execSQL("DROP TABLE diag_event", SQLITE_DIAG_EVENT_FILE);
            SqliteEngine::freeQuery(result);
            // reinit the database
            reinitDiagDataBase();
            return false;
        }
        else
        {
        	// row and column should be 0 if there is no record in the database
            log_info("%s diag event data base file format ok!!!",__FUNCTION__);
            log_info("%s row = %d , column = %d\r\n",__FUNCTION__, row , column);
        }
       
    }
    else
    {
        log_info("%s query event from diag event data base failed",__FUNCTION__);
        SqliteEngine::freeQuery(result);
        return false;
    }

    SqliteEngine::freeQuery(result);

    return true;
}

// init the diag event store data base
bool Diagnostics::initDiagEventStoreDataBase(void)
{
    char **result;
    int row, column;
    int retry_times = 3;
    int iRet;

    log_info("%s called", __FUNCTION__);

    do
    {
        iRet = SqliteEngine::execSQL(DIAG_EVENT_STORED_TABLE, SQLITE_DIAG_EVENT_STORE_FILE);
        if(iRet != SQLITE_OK)
        {
            log_info("%s create diag store event table failed!!! iRet = %d , retry times = %d", __FUNCTION__, iRet, retry_times);
            reinitDiagStoreDataBase();
        }
        else
        {
            log_info("initialize diag store event database table success ...iRet = %d", iRet);
            break;
        }

        retry_times--;
        sleep(1);
    }while(retry_times != 0);

    if(retry_times == 0)
    {
        // we initialize failed here , do not need to do the following steps since we create database failed
        log_error("init diag store event data base failed after three times!!!");
        return false;
    }
    else
    {
        log_info("%s diag event store data base retry times not exceeded , retry times = %d, init diag store event data base ok", __FUNCTION__, retry_times);
    }

    // from here the data base file exist
    iRet = SqliteEngine::query("SELECT * FROM diag_store_event", &result, &row, &column, SQLITE_DIAG_EVENT_STORE_FILE);
    //log_info("%s check if diag event store data base format ok , SELECT * FROM diag_store_event row = %d , column = %d\r\n", __FUNCTION__, row, column);
    if(iRet == SQLITE_OK)
    {
        if (row > 0 && column != DIAG_EVENT_STORE_TABLE_COLUMN) 
        {
            // query data base 
            // data base format wrong
            log_error("diag store event data base error!!!");
            log_error("%s database format wrong column shoud be %d , but row = %d , column = %d\r\n", __FUNCTION__, DIAG_EVENT_STORE_TABLE_COLUMN, row , column);
            SqliteEngine::execSQL("DROP TABLE diag_store_event", SQLITE_DIAG_EVENT_STORE_FILE);
            SqliteEngine::freeQuery(result);
            // reinit the event store database
            reinitDiagStoreDataBase();
            return false;
        }
        else
        {
            // row and column should be 0 if there is no record in the database
            log_info("%s diag store event data base file format ok!!!",__FUNCTION__);
            log_info("&&&&&&&&&&&&&&&&&& %s diag store database , row = %d , column = %d &&&&&&&&&&&&&&&&&&&", __FUNCTION__, row , column);
            setEventStoreNum(row); // current itv event stored in the diag event database
        }

    }
    else
    {
        log_info("%s query stored event from diag store event data base failed",__FUNCTION__);
        SqliteEngine::freeQuery(result);
        return false;
    }

    SqliteEngine::freeQuery(result);

    return true;
}

// reinit diag event data base
// first check if the file exist
// mv the data base to bak up
void Diagnostics::reinitDiagDataBase(void)
{
    log_error("%s ", __FUNCTION__);
    std::string diag_event_db_path = SQLITE_DIAG_EVENT_FILE;
    if(access(diag_event_db_path.c_str(), F_OK)==0)
    {
        // if the data base file exist, rename the data base file to .bak
        log_warn("%s rename the diag event data base file to bak!!!!!!",__FUNCTION__);
        std::string cmd = "mv " + std::string(SQLITE_DIAG_EVENT_FILE) + " " + std::string(SQLITE_DIAG_EVENT_FILE) + ".bak";
        system(cmd.c_str());
        SqliteEngine::execSQL(DIAG_EVENT_TABLE,  SQLITE_DIAG_EVENT_FILE);
    }
    else
    {
        // create the data base file if it doest not exist
        log_error("%s diag event database file does not exist",__FUNCTION__);
        SqliteEngine::execSQL(DIAG_EVENT_TABLE,  SQLITE_DIAG_EVENT_FILE); // create database file
    }
}

void Diagnostics::reinitDiagStoreDataBase(void)
{
    log_error("%s ", __FUNCTION__);
    std::string diag_store_event_db_path = SQLITE_DIAG_EVENT_STORE_FILE;
    if(access(diag_store_event_db_path.c_str(), F_OK)==0)
    {
        // if the data base file exist, rename the data base file to .bak
        log_warn("%s rename the diag store event data base file to bak!!!!!!",__FUNCTION__);
        std::string cmd = "mv " + std::string(SQLITE_DIAG_EVENT_STORE_FILE) + " " + std::string(SQLITE_DIAG_EVENT_STORE_FILE) + ".bak";
        system(cmd.c_str());
        SqliteEngine::execSQL(DIAG_EVENT_STORED_TABLE,  SQLITE_DIAG_EVENT_STORE_FILE);
    }
    else
    {
        // create the data base file if it doest not exist
        log_error("%s diag store event database file does not exist",__FUNCTION__);
        SqliteEngine::execSQL(DIAG_EVENT_STORED_TABLE,  SQLITE_DIAG_EVENT_STORE_FILE); // create database file
    }
}

// store the data event into the database
// incase the event is recovered after the machine is restart we report the event to web
// first we query if the event record is already in the table
// if the event name is not in the table , we add the record
// if the event name is already in the table, we update the event name record
// we dont need to add lock to protect the data base
// since the query and update are processed in serial
int Diagnostics::UpdateDiagEventDataBase(const DiagEventData & diag_event_data)
{
    log_info("%s called",__FUNCTION__);
    int ret = SQLITE_OK;
    char **result1, **result;
    int row = 0, column = 0;
    int row1 = 0, column1 = 0;
    std::string content;
    std::stringstream status_stream;
    std::stringstream timestamp_stream;
    std::string serial_num;

    content = diag_event_data.data_item.event_content; // event content , we will use this to get the level and event type
    serial_num = diag_event_data.data_item.event_serial_num;
    status_stream.clear();
    status_stream << diag_event_data.data_item.event_status;
    timestamp_stream.clear();
    timestamp_stream << diag_event_data.event_timestamp;

    log_info("%s event content : %s", __FUNCTION__, content.c_str());
    log_info("%s serial num : %s", __FUNCTION__, serial_num.c_str());
    log_info("%s status : %s", __FUNCTION__, status_stream.str().c_str());
    log_info("%s timestamp : %s", __FUNCTION__, timestamp_stream.str().c_str());

    ret = SqliteEngine::query("SELECT * FROM diag_event", &result, &row, &column, SQLITE_DIAG_EVENT_FILE);
    log_info("%s select * from diag event , row : %d , column : %d\r\n",__FUNCTION__, row, column);
    if(ret == SQLITE_OK)
    {
        if(row > 0 && column != DIAG_EVENT_TABLE_COLUMN)
        {
            // usually cannot reach here, format wrong
            SqliteEngine::freeQuery(result);
            reinitDiagDataBase();
            log_error("%s column num does not match", __FUNCTION__);
            return -1;
        }
        else
        {
            log_info("%s query record from diag event table ok , row = %d", __FUNCTION__, row);
            ret = SqliteEngine::query("SELECT * FROM diag_event WHERE eventContent='"+content+"'", &result1, &row1, &column1, SQLITE_DIAG_EVENT_FILE);
            if(ret == SQLITE_OK)
            {
                if(row1 <= 0)
                {
                    // not exist , just insert the event into the data base table
                    log_info("%s did not find corresponding entry in the table, just insert it as a fresh entry, event content = %s",__FUNCTION__, content.c_str());

                    SqliteEngine::execSQL("INSERT INTO diag_event(eventContent, status, serialNum, timestamp) "
                    "VALUES('"+content+"', '"+status_stream.str()+"', '"+serial_num+"', '"+timestamp_stream.str()+"')", SQLITE_DIAG_EVENT_FILE);
                }
                else
                {
                    // update the entry according to event content field
                    log_info("%s event content already exist just update the table content\r\n", __FUNCTION__);
                    SqliteEngine::execSQL("UPDATE diag_event SET status='"+status_stream.str()+"' WHERE eventContent='"+content+"'", SQLITE_DIAG_EVENT_FILE);
                    SqliteEngine::execSQL("UPDATE diag_event SET serialNum='"+serial_num+"' WHERE eventContent='"+content+"'", SQLITE_DIAG_EVENT_FILE);
                    SqliteEngine::execSQL("UPDATE diag_event SET timestamp='"+timestamp_stream.str()+"' WHERE eventContent='"+content+"'",SQLITE_DIAG_EVENT_FILE);
                }


            }
            else
            {
                log_error("%s SELECT * FROM diag_event WHERE eventContent = failed ret = %d", __FUNCTION__, ret);
                SqliteEngine::freeQuery(result);
                SqliteEngine::freeQuery(result1);
                return -2;
            }
        }
    }
    else
    {
    	log_error("%s select * from diag event data base failed ret = %d", __FUNCTION__, ret);
        SqliteEngine::freeQuery(result);
    	return -3;
    }

    SqliteEngine::freeQuery(result);
    SqliteEngine::freeQuery(result1);
    
    return 0;
}

// report if the warning event is recovered this cycle
// here we deal the event that (status = 1 and this cycle status = 0)
// notify recovered to web this cycle
void Diagnostics::reportRecoveredEvent(const DataBaseCmdStru & event_data_this, const int & event_status_last , const std::string & serial_number_last)
{
	int event_type; // event type used to call different type of function
    if(event_data_this.event_data.data_item.event_status == 0 && event_status_last == 1)
    {
        // notify recovered event to match the serial num last round
        log_info("&&&&&&&& %s warning recovered, report it to web &&&&&&&&",__FUNCTION__);
        log_info("%s event content : %s serial num : %s",__FUNCTION__, event_data_this.event_data.data_item.event_content.c_str(), serial_number_last.c_str());
        event_type = get_warn_type_by_content(event_data_this.event_data.data_item.event_content);
        log_info("%s event type : %d",__FUNCTION__, event_type);
        switch(event_type)
        {
        	case NAVIGATION_WARNING:
        	    // only localization recovered
        	    log_info("%s navigation warning event func called",__FUNCTION__);
        	break;
        	case DEVICE_WARNING:
        	    log_info("%s device warning event func called",__FUNCTION__);
        	    notifyDeviceWarningEvent(event_data_this.event_data.data_item.event_content, 
                                 event_data_this.event_data.data_item.event_status, 
                                 event_data_this.event_data.event_timestamp, 
                                 serial_number_last);
        	break;
        	case COMMUNICATION_WARNING:
                // usually can not reach here
                // communication warning is detected by web
        	    log_info("%s communication warning event func called",__FUNCTION__);
        	break;
        	case ABNORMAL_WARNING:
        	    log_info("%s abnormal warning event funct called",__FUNCTION__);
                notifyAbnormalWarningEvent(event_data_this.event_data.data_item.event_content,
                                event_data_this.event_data.data_item.event_status, 
                                event_data_this.event_data.event_timestamp, 
                                serial_number_last);
        	break;
        	default:
        	    log_error("unrecognized warning message type : %d", event_type);
        	break;
        }
        
    }
    else
    {
    	log_info("%s do not need to report the recovered event",__FUNCTION__);
    	log_info("%s event content : %s , event status : %d , event status last round : %d",__FUNCTION__, 
    		event_data_this.event_data.data_item.event_content.c_str()
    		, event_data_this.event_data.data_item.event_status
    		, event_status_last);
    }
}

// string is event name that we query , status is the output we query from the database
int Diagnostics::getAccordEventStats(const std::string & event_content, const int & status_this, int & status, std::string & serial_num)
{
    int iRet;
    char **result1 = nullptr;
    int row1 = 0;
    int column1 = 0;
    std::string evt_content_str = "";
    long timestamp;

    log_info("%s event content = %s",__FUNCTION__, event_content.c_str());

    iRet = SqliteEngine::query("SELECT * FROM diag_event WHERE eventContent='"+event_content+"'", &result1, &row1, &column1, SQLITE_DIAG_EVENT_FILE);
    if(iRet == SQLITE_OK)
    {
        if (row1 <= 0)
        {
            // we dont need to insert the event to diag event table now
            // it will be inserted to diag event table later
            log_error("%s did not find corresponding event content in diag event table , event content = %s", __FUNCTION__, event_content.c_str());
            SqliteEngine::freeQuery(result1);
            return -1;
        }
        else
        {
            log_info("row1 = %d\r\n", row1);
            evt_content_str = result1[column1+1];
            status = atoi(result1[column1+2]);
            serial_num = result1[column1+3];
            timestamp = atol(result1[column1+4]);
            log_info("query result , event content : %s , event status : %d , serial number : %s , timestamp : %ld", evt_content_str.c_str(), status, serial_num.c_str(), timestamp);
            //if(status_this == 0)
            //{
                // we update the table directly
                // if it is 0 it is clear
                // then if the error event happened again, the serial number will be updated anyway
            // we dont need to judge if the status this cycle is 0 or not
            std::stringstream status_stream;
            status_stream.clear();
            status_stream << status_this;
            log_info("%s save the event status anyway!!!\r\n",__FUNCTION__);
            SqliteEngine::execSQL("UPDATE diag_event SET status='"+status_stream.str()+"' WHERE eventContent='"+event_content+"'", SQLITE_DIAG_EVENT_FILE);
            //}

        }
    }
    else
    {
        log_info("%s query event from database failed iRet = %d\r\n",__FUNCTION__, iRet);
        SqliteEngine::freeQuery(result1);
        return -2;
    }

    SqliteEngine::freeQuery(result1);

    return 0;
}

// thread to process diagnostics event to insert it to database !!!!!!!!!!!!!!!!!!!!!!
// in order to reduce receiving latency of the robot info
// since data base process is slow
// we do the data process in another thread
void Diagnostics::processDiagEvent(void)
{
    printf("%s diag database thread ", __FUNCTION__);
    DataBaseCmdStru tmp;
    int diag_database_thd_run = 1;
    int iRet;
    int error_status_last_rnd;
    std::string serial_num = "";

    while(diag_database_thd_run)
    {
        tmp = event_msg_list_.get();
        log_warn("%s process event data message",__FUNCTION__);
        if(data_base_init_ok_)
        {
        	// check if there is uncleared event last round 
        	// report it to web if the error or event is cleared or recovered
            if(tmp.cmd_type == EVENT_CHECK_LAST_CYCLE)
            {
                // check event last cycle
                log_info("%s deal recover event if happened related to last round",__FUNCTION__);
                iRet = getAccordEventStats(tmp.event_data.data_item.event_content, tmp.event_data.data_item.event_status, error_status_last_rnd, serial_num);
                if(iRet < 0)
                {
                    // just information print
                    log_warn("event get failed , event content = %s", tmp.event_data.data_item.event_content.c_str());
                    continue;
                }
                else
                {
                    log_info("event get success... event content = %s", tmp.event_data.data_item.event_content.c_str());
                }

                // report the recovered event if needed
                reportRecoveredEvent(tmp, error_status_last_rnd, serial_num);
            }
            else if(EVENT_PROC_THIS_CYCLE == tmp.cmd_type)
            {
                // check event happened this cycle
                // if there is event happened just record it anyway
                log_info("%s deal event that happen this round",__FUNCTION__);
                log_info("%s just update the event table",__FUNCTION__);
                iRet = UpdateDiagEventDataBase(tmp.event_data);
                if(iRet < 0)
                {
                    log_error("%s update event to event data base failed!!! iRet = %d",__FUNCTION__, iRet);
                }
                else
                {
                    log_info("%s update event to event data base success... iRet = %d",__FUNCTION__, iRet);
                }
            }
            else
            {
                log_error("%s unrecognized command type : %d",__FUNCTION__, tmp.cmd_type);
            }

        }
        else
        {
            log_error("%s data base init failed!!! , can not add the event to event database",__FUNCTION__);
        }

    }
}

// add the command to data process list
// dont add the message if the list reaches maximum
void Diagnostics::diagEventCmdAdd(const struct DataBaseCmdStru & event_msg)
{
	
	if(event_msg_list_.size() > EVENT_MSG_LIST_SIZE_MAX)
	{
		log_warn("%s event list size reaches maximum , cannot add more messages to event list to process", __FUNCTION__);
		return;
	}

    log_info("%s add message to list\r\n",__FUNCTION__);
	event_msg_list_.add(event_msg);
}

void Diagnostics::diagStoreEventAdd(const std::string title, const Json::Value event_msg)
{
    if(getEventStoreNum() == (MAXIMUM_STORED_EVENT_NUM - 1))
    {
        log_warn("%s diag store event message list reaches maximum , cannot add more event to database, just drop them", __FUNCTION__);
        return;
    }

    log_info("%s add itv event message to list\r\n",__FUNCTION__);
    PostToWebContent post_web_content(title, event_msg);
    event_store_list_.add(post_web_content);
}

// this function maybe unused forever(incase the diagnostics process is restarted, we need to load the event status from database again)
// we query the initial status from the database to initilize the check status variable in each event detect function
// this function is used in case the event occured is reported to web server multiple times
int Diagnostics::initEventVarStats(void)
{
	int i = 0;
	std::string event_content_str = "";
	int event_status = -1;
	int iRet;

	for(i = 0; i < EVENT_VAR_INDEX_MAX; i++)
	{
        event_content_str = get_event_content_by_var_index(i);
        if(event_content_str != "unknown")
        {
        	// get data base status and set to global variable
            iRet = getEventStatusAccordingToEventContent(event_content_str, event_status);
            if(iRet < 0)
            {
            	log_error("%s event variable index = %d", __FUNCTION__, i);
            	log_error("%s access diag event database error, or cannot find corresponding event content entry , where entry content = %s", __FUNCTION__, event_content_str.c_str());
            	continue;
            }
            else
            {
            	//set and initialize the global class private variable here
            	// set the corresponding variable value according to the database
            	switch(i)
            	{
            		case LOCALIZATION_LOST_INDEX:
                        log_info("set localization lost global mem variable");
                        localization_lost_stats_ = event_status;
            		break;
            		case SIGNAL_BAD_INDEX:
                        log_info("set signal bad global mem variable");
                        signal_bad_stats_ = event_status;
            		break;
            		case PTZ_ERROR_INDEX:
                        log_info("set ptz error global mem variable");
                        ptz_error_stats_ = event_status;
            		break;
            		case LIDAR_ERROR_INDEX:
                        log_info("set lidar error global mem variable");
                        lidar_error_stats_ = event_status;
            		break;
            		case ULTRA_ERROR_INDEX:
                        log_info("set ultrasound error global mem variable");
                        ultra_error_stats_ = event_status;
            		break;
            		case ANTI_DROP_ERROR_INDEX:
                        log_info("set anti drop error global mem variable");
                        ads_error_stats_ = event_status;
            		break;
            		case IMU_ERROR_INDEX:
                        log_info("set imu error global mem variable");
                        imu_error_stats_ = event_status;
            		break;
            		case CHASSIS_ERROR_INDEX:
                        log_info("set chassis error global mem variable");
                        chassis_error_stats_ = event_status;
            		break;
            		case BATTERY_ERROR_INDEX:
                        log_info("set battery error global mem variable");
                        battery_error_stats_ = event_status;
            		break;
            		case SPEAKER_ERROR_INDEX:
                        log_info("set speaker error global mem variable");
                        speaker_error_stats_ = event_status;
            		break;
            		case MIC_ERROR_INDEX:
                        log_info("set mic error global mem variable");
                        mic_error_stats_ = event_status;
            		break;
            		case VOIP_ERROR_INDEX:
                        log_info("set voip error global mem variable");
                        voip_error_stats_ = event_status;
            		break;
            		case FLOOD_WARNING_INDEX:
                        log_info("set flood warning global mem variable");
                        flood_warning_stats_ = event_status;
            		break;
            		case OVERTEMP_WARNING_INDEX:
                        log_info("set overtemp warning global mem variable");
                        overtemp_warning_stats_ = event_status;
            		break;
            		case EMERGE_STOP_WARNING_INDEX:
                        log_info("set emerge stop warning global mem variable");
                        emerge_stop_warning_stats_ = event_status;
            		break;
            		case COLLISION_WARNING_INDEX:
                        log_info("set collision warning global mem variable");
                        collision_warning_stats_ = event_status;
            		break;
            		case ABNORMAL_SHUTDOWN_WARNING_INDEX:
                        log_info("set abnormal shutdown warning global mem variable");
                        abnormal_shutdown_warning_stats_ = event_status;
            		break;
            		default:
            		    log_error("invalid event variable index");
            		break;
            	}
            }
        }
        else
        {
        	log_error("%s cannot find corresponding event content string from map!!!", __FUNCTION__);
        }

	}

	return 0;
}

// get last cycle event warning status from the database when the diag module is initialiezed
int Diagnostics::getEventStatusAccordingToEventContent(const std::string & event_content, int & event_status)
{
	int iRet;
    char **result1 = nullptr;
    int row1 = 0;
    int column1 = 0;
    std::string evt_content_str = "";
    std::string serial_num = "";
    long timestamp;

    //log_info("%s event content = %s",__FUNCTION__, event_content.c_str());

    iRet = SqliteEngine::query("SELECT * FROM diag_event WHERE eventContent='"+event_content+"'", &result1, &row1, &column1, SQLITE_DIAG_EVENT_FILE);
    if(iRet == SQLITE_OK)
    {
        if (row1 <= 0)
        {
            // we dont need to insert the event to diag event table now
            // it will be inserted to diag event table later
            //log_error("%s did not find corresponding event content in diag event table , event content = %s", __FUNCTION__, event_content.c_str());
            SqliteEngine::freeQuery(result1);
            return -1;
        }
        else
        {
            log_info("row1 = %d\r\n", row1);
            evt_content_str = result1[column1+1];
            event_status = atoi(result1[column1+2]);
            serial_num = result1[column1+3];
            timestamp = atol(result1[column1+4]);
            //log_info("query result , event content : %s , event status : %d , serial number : %s , timestamp : %ld", evt_content_str.c_str(), event_status, serial_num.c_str(), timestamp);

        }

        
    }
    else
    {
        log_info("%s query event from database failed iRet = %d\r\n",__FUNCTION__, iRet);
        SqliteEngine::freeQuery(result1);
        return -2;
    }

    SqliteEngine::freeQuery(result1);

	return 0;
}

bool Diagnostics::addEventStoreDataBase(const std::string & eventId , const Json::Value event_msg)
{
    int sqlret = SQLITE_OK;
    int try_times = 3;
    Json::FastWriter writer;
    std::string event_str = "";
    event_str = writer.write(event_msg);
    bool bRet = true;
    //std::stringstream danger_type_stream;
    //danger_type_stream << list_data.item.danger_type;
    //std::string detection_string = std::to_string(list_data.detection_timestamp);
    log_info("%s title = %s, event msg = %s",__FUNCTION__, eventId.c_str(), event_str.c_str());
    log_info("%s start",__FUNCTION__);
    boost::unique_lock<boost::mutex> lock(diag_database_mutex_); // add into database mutex , we may also delete from the database and report it to web
    do
    {
        sqlret = SqliteEngine::execSQL("INSERT INTO diag_store_event(title, event_store) VALUES('"
                            +eventId+"', '"
                            +event_str+"')",
                            SQLITE_DIAG_EVENT_STORE_FILE);
        if(sqlret != SQLITE_OK)
        {
            //reinitDiagStoreDataBase();
            log_error("%s insert itv event message into event store database failed!!!",__FUNCTION__);
        }

        if(try_times == 0)
        {
            break;
        }

        try_times--;

    }while(sqlret != SQLITE_OK);
    
    if(try_times == 0)
    {
        log_info("%s add event to event store data base reaches maximum times, insert into database failed",__FUNCTION__);
        bRet = false;
    }
    else
    {
        eventStoreNumInc();
        log_info("%s try time = %d, event database record num : %d",__FUNCTION__, try_times, getEventStoreNum());
    }

    log_info("%s end",__FUNCTION__);
    return bRet;
}

void Diagnostics::storeDiagEventThread(void)
{
    log_info("%s",__FUNCTION__);

    PostToWebContent tmp;
    Json::FastWriter writer;
    bool bRet;

    while(1)
    {
        tmp = event_store_list_.get();
        if(event_data_base_init_ok_)
        {
            std::string eventId;
            Json::Value event_json;
            eventId = tmp.get_event_title();
            event_json = tmp.get_event_content();
            log_info("%s event id : %s , event json %s",__FUNCTION__, eventId.c_str(), writer.write(event_json).c_str());
            bRet = addEventStoreDataBase(eventId, event_json);
            if(!bRet)
            {
                log_error("%s add event into itv store database failed!!!",__FUNCTION__);
            }
            else
            {
                // success
            }

        }
        else
        {
            log_error("%s diag store event database initialized not ok, can not add event to database",__FUNCTION__);
        }

    }
}
// report stored itv event to web
void Diagnostics::reportItvEventThread(void)
{
    char **result = nullptr;
    int row = 0;
    int column = 0;

    boost::unique_lock<boost::mutex> lock(Diagnostics::connect_state_mutex_, boost::defer_lock);
    while(1)
    {
        while(!getConnectState())
        {
            lock.lock();
            log_info("--------%s wait mqtt connected-------",__FUNCTION__);
            connect_state_cond_.wait(lock);
            lock.unlock();
        }

        //log_info("%s wake up from connect signal, check if there is stored event in event stored database",__FUNCTION__);
        // get all record from database

        int cnt = 0;
        int iRet;
        
        // get all record from diag event store database
        {
            boost::unique_lock<boost::mutex> lock(diag_database_mutex_);
            iRet = SqliteEngine::query("SELECT * FROM diag_store_event", &result, &row, &column, SQLITE_DIAG_EVENT_STORE_FILE);
        }

        if(iRet != SQLITE_OK)
        {
            // get nothing from database failed
            log_warn("%s get event msg record failed from diag_store_event database, iRet = %d",__FUNCTION__, iRet);
            SqliteEngine::freeQuery(result);
            waitNotEmptyDataBase();
            continue;
        }
        else
        {
            //log_info("%s get event message from database success, row = %d, column = %d", __FUNCTION__, row, column);
            if(row > 0 && column != DIAG_EVENT_STORE_TABLE_COLUMN)
            {
                log_error("diag store event data base error!!!");
                log_error("%s database format wrong column shoud be %d , but row = %d , column = %d\r\n", __FUNCTION__, DIAG_EVENT_STORE_TABLE_COLUMN, row , column);
                SqliteEngine::execSQL("DROP TABLE diag_store_event", SQLITE_DIAG_EVENT_STORE_FILE);
                SqliteEngine::freeQuery(result);
                // reinit the event store database
                reinitDiagStoreDataBase();
                continue;
            }
            else
            {
                //log_info("%s diag event store database format ok, total record = %d",__FUNCTION__, row);
                if(row == 0)
                {
                    log_info("%s diag event store database empty, wait non empty database",__FUNCTION__);
                    SqliteEngine::freeQuery(result);
                    waitNotEmptyDataBase();
                    continue;
                }
            }
        }
        // recurred post the diag warning event to web
        while(cnt != row)
        {
            if(getConnectState() == 1)
            {
                // if post finished delete the record from the database, decrease the event store num
                std::string event_id = "";
                int id;
                int iRet;
                std::string event_content = "";
                Json::Value root;
                Json::Reader reader;
                id = atoi(result[(cnt+1)*column]);
                event_id = result[(cnt+1)*column+1];
                event_content = result[(cnt+1)*column+2];
                log_info("%s post event to web : id : %d event id : %s , event content : %s", __FUNCTION__, id, event_id.c_str(), event_content.c_str());
                if(reader.parse(event_content, root))
                {
                    log_info("%s json string parse ok",__FUNCTION__);
                    Utils::get_instance()->NotifyRobotStatus("notify_event", root, "");
                }
                //Utils::get_instance()->NotifyRobotStatus("notify_event", event_content, "");
                log_info("%s delete it from database",__FUNCTION__);
                iRet = deleteRecordWithId(id);
                if(iRet < 0)
                {
                    log_error("%s delete record from event store database failed : id = %d",__FUNCTION__, id);
                }
            }
            else
            {
                log_warn("%s remote connect broke, just continue to next round",__FUNCTION__);
                break;
            }
            
            cnt++;
            log_info("%s cnt : %d", __FUNCTION__, cnt);
            usleep(1000*1000);
        }

        log_info("%s post the diag event to web finished, this cycle cnt = %d",__FUNCTION__, cnt);
        SqliteEngine::freeQuery(result);
    }
}

// wait 3 seconds to check if there is record in the data base in case we are on line
void Diagnostics::waitNotEmptyDataBase(void)
{
    boost::unique_lock<boost::mutex> lock(database_wait_mutex_, boost::defer_lock);

    lock.lock();
    database_wait_mutex_cv_.timed_wait(lock, boost::get_system_time() + boost::posix_time::milliseconds(1000*3));
    lock.unlock();
}

int Diagnostics::deleteRecordWithId(int id)
{
    int sqlret;

    std::stringstream id_stream;
    //id_stream.clear();
    //id_stream << id;
    log_info("%s start",__FUNCTION__);
    
    log_info("%s id = %d", __FUNCTION__, id);
    std::string record_id = std::to_string(id);
    log_info("%s record id = %s",__FUNCTION__, record_id.c_str());
    boost::unique_lock<boost::mutex> lock(diag_database_mutex_); // add into database mutex , we may also delete from the database and report it to web

    sqlret = SqliteEngine::execSQL("DELETE FROM diag_store_event WHERE id = " + record_id, SQLITE_DIAG_EVENT_STORE_FILE);
    if(sqlret != SQLITE_OK)
    {
        log_error("%s delete record from event store table failed where id = %d , ret = %d",__FUNCTION__, id, sqlret);
        return -1;
    }
    else
    {
        eventStoreNumDec(); // decrease the event store num
        log_info("%s delete record from event store table success where id = %d",__FUNCTION__, id);
    }

    log_info("%s end",__FUNCTION__);

    return 0;
}


void Diagnostics::eventStoreNumInc(void)
{
    boost::unique_lock<boost::mutex> lock(store_num_mutex_);
    if(event_store_num_ != (MAXIMUM_STORED_EVENT_NUM - 1))
    {
        event_store_num_++;
    }
}

void Diagnostics::eventStoreNumDec(void)
{
    boost::unique_lock<boost::mutex> lock(store_num_mutex_);

    if(event_store_num_ != 0)
    {
        event_store_num_--;
    }
}

int Diagnostics::getEventStoreNum(void)
{
    boost::unique_lock<boost::mutex> lock(store_num_mutex_);
    return event_store_num_;
}

//SQLITE_DIAG_EVENT_STORE_FILE
// this function is called when class is initialized
void Diagnostics::setEventStoreNum(int num)
{
    boost::unique_lock<boost::mutex> lock(store_num_mutex_);
    event_store_num_ = num;
}