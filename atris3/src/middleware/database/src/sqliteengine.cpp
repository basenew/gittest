/*
 * sqliteengine.cpp
 *
 *  Created on: 2018-6-21
 *      Author: fupj
 */

#include "sqliteengine.h"
#include <sys/stat.h>
#include <sstream>
#include "imemory/semaphore.h"
#include "semaphore.h"
#include "log/log.h"

namespace SqliteEngine {
static int sqlite_semid = -1;

static boost::mutex sqlite_mutex;

#define SQLITE_ENGINE_TAG      "SqliteEngine->"

static bool sqlite_semaphore_init() {
    if (sqlite_semid == -1) {
        boost::unique_lock<boost::mutex> lock(sqlite_mutex);
        if (sqlite_semid == -1) {
            if ((sqlite_semid = semget((key_t)SEM_SEED_sqlite, 1, 0666 | IPC_CREAT)) == -1) {
                log_error(SQLITE_ENGINE_TAG"%s semget failed", __FUNCTION__);
                return false;
            }

            if (!set_semvalue(sqlite_semid, 1)) {
                log_error(SQLITE_ENGINE_TAG"%s semctl failed", __FUNCTION__);
            }
        }
    }

    return true;
}

int execSQL(std::string sql, std::string fileName) {
    int ret = SQLITE_OK, can_retry = 1;
    sqlite3 *db;
    char* errmsg;
retry:
    if (sqlite_semaphore_init()) {
        semaphore_p(sqlite_semid);
        do {
            if(access(SQLITE_DATA_DIR, 0) == -1) {
                mkdir(SQLITE_DATA_DIR, 0777);
            }
            ret = sqlite3_open(fileName.c_str(), &db);
            if (ret != SQLITE_OK) {
                log_error(SQLITE_ENGINE_TAG"%s Open database failed(%d): %s, sql: %s", 
                  __FUNCTION__, ret, fileName.c_str(), sql.c_str());
                break;
            }
            ret = sqlite3_exec(db, sql.c_str(), NULL, NULL, &errmsg);
            if (ret != SQLITE_OK) {
                log_error(SQLITE_ENGINE_TAG"%s sqlite3_exec failed(%d): %s, sql: %s", 
                  __FUNCTION__, ret, errmsg, sql.c_str());
                if (can_retry && ret == SQLITE_NOTADB && sql.find("CREATE") != std::string::npos) {
                    sqlite3_close(db);
                    can_retry = 0;
                    remove(fileName.c_str());
                    semaphore_v(sqlite_semid);
                    goto retry;
                }
            }
            sqlite3_close(db);
        } while(0);
        semaphore_v(sqlite_semid);
    } else {
        log_error(SQLITE_ENGINE_TAG"%s sqlite_semaphore_init failed", __FUNCTION__);
    }
    return ret;
}

int query(std::string sql, char ***result, int *row, int *column, std::string fileName) {
    int ret = SQLITE_OK;
    sqlite3 *db;
    char* errmsg;
    
    *result = NULL;

    if (sqlite_semaphore_init()) {
        semaphore_p(sqlite_semid);
        do {
            if(access(SQLITE_DATA_DIR, 0) == -1) {
                mkdir(SQLITE_DATA_DIR, 0777);
            }
            ret = sqlite3_open(fileName.c_str(), &db);
            if (ret != SQLITE_OK) {
                log_error(SQLITE_ENGINE_TAG"%s Open database failed(%d): %s, sql: %s", 
                  __FUNCTION__, ret, SQLITE_DATA_FILE, sql.c_str());
                break;
            }
            ret = sqlite3_get_table(db, sql.c_str(), result, row, column, &errmsg);
            if (ret != SQLITE_OK) {
                log_error(SQLITE_ENGINE_TAG"%s sqlite3_get_table failed(%d): %s, sql: %s", 
                  __FUNCTION__, ret, errmsg,  sql.c_str());
            }
            sqlite3_close(db);
        } while(0);
        semaphore_v(sqlite_semid);
    } else {
        log_error(SQLITE_ENGINE_TAG"%s sqlite_semaphore_init failed", __FUNCTION__);
    }
    return ret;
}

void freeQuery(char **result) {
    if (result) {
        sqlite3_free_table(result);
    }
}
} // namespace SqliteEngine

