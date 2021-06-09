/*
* File      : kv_db.h
*
* Change Logs:
* Date           Author       Notes
* 2019-07-18     yangzou
*/

#pragma once

#include "database/sqliteengine.h"
#include "log/log.h"

#include <iostream>
#include <map>
#include <vector>

using namespace std;

class DB
{
public:
    DB(const string& create_db_sql)
    {
        SqliteEngine::execSQL(create_db_sql);
    };
    virtual ~DB(){};

    bool select(const string& db_name)
    {
        if (db_name.empty())
            return false;

        mod_ = db_name + "_";
        return true;
    };

    bool exist()
    {
        return false;
    };

protected:
    string mod_;
};

#define CREATE_KV_TABLE_SQL  "CREATE TABLE IF NOT EXISTS [kv] (" \
                             "[k] TEXT UNIQUE," \
                             "[v] TEXT)"

using kv_map=map<string, string>;
class KVDB:public DB
{
public:
    KVDB(const string &invalid_val=string(""), int invalid_int_val = -1)
    :DB(CREATE_KV_TABLE_SQL)
    ,invalid_val_(invalid_val)
    ,invalid_int_val_(invalid_int_val)
    {
    };
    ~KVDB(){};

    bool exist(const string& k);
    bool set(const string& k, const string& v);
    bool set_int(const string& k, int int_v);
    bool del(const string& k);

    const kv_map& get_kvs();

    string get(const string& k);

    int get_int(const string& k);

    bool is_valid(const string &val){return val != invalid_val_;};
    bool is_valid(int int_val){return int_val != invalid_int_val_;};

private:
    kv_map      kv_map_;
    string      invalid_val_;
    int         invalid_int_val_;
}; 

#define CREATE_KSET_TABLE_SQL  "CREATE TABLE IF NOT EXISTS [kset] (" \
                               "[m] TEXT," \
                               "[v] TEXT)"

using sets_vct=vector<string>;
class KSetsDB:public DB
{
public:
    KSetsDB()
    :DB(CREATE_KSET_TABLE_SQL)
    {
    };
    ~KSetsDB(){};

    bool exist(const string& v);
    bool add(const string& v);
    bool del(const string& v);
    bool del_all();

    const sets_vct& get();

private:
    sets_vct      sets_vct_;
}; 


void test_kvdb();

