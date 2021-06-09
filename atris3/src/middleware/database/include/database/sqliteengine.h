/*
 * sqliteengine.h
 *
 *  Created on: 2018-6-21
 *      Author: fupj
 */

#ifndef SQLITEENGINE_H_
#define SQLITEENGINE_H_
#include <sqlite3.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <libgen.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/sem.h>
#include <sstream>

#define SQLITE_DATA_DIR        "/userdata/impdata"
#define SQLITE_DATA_FILE       "/userdata/impdata/database.db"
#define SQLITE_ABI_FILE        "/userdata/impdata/data_abi.db"

#define  SQLITE_DANGER_IMSI_FILE      "/userdata/impdata/danger_imsi.db"
#define SQLITE_DETECTION_IMSI_FILE    "/userdata/impdata/detection_imsi.db"
#define SQLITE_DIAG_EVENT_FILE "/userdata/impdata/diag_event.db"
#define SQLITE_DIAG_EVENT_STORE_FILE "/userdata/impdata/diag_event_store.db"
#define SQLITE_SW_UPGRADE_FILE "/userdata/impdata/sw_upgrade.db"

namespace SqliteEngine {
int execSQL(std::string sql, std::string fileName= SQLITE_DATA_FILE);
int query(std::string sql, char ***result, int *row, int *column, std::string fileName= SQLITE_DATA_FILE);
void freeQuery(char **result);
} // namespace SqliteEngine

#endif /* SQLITEENGINE_H_ */
