/*
* File      : nav_db.cpp
*
* Change Logs:
* Date           Author       Notes
* 2019-07-18     yangzou
*/

#include "kv_db.h"
#include <sqlite3.h>


bool KVDB::exist(const string& k)
{
    string sql("select v from kv where k='");
    sql.append(mod_);
    sql.append(k);
    sql.append("'");
    char **result;
    int row, column;
    SqliteEngine::query(sql, &result, &row, &column);
    SqliteEngine::freeQuery(result);
    cout << "sql:" << sql << endl;

    return row > 0;
}

bool KVDB::set(const string& k, const string& v)
{
    string sql("replace into kv values('");
    sql.append(mod_);
    sql.append(k);
    sql.append("','");
    sql.append(v);
    sql.append("')");
    cout << "sql:" << sql << endl;

    return SQLITE_OK == SqliteEngine::execSQL(sql);
}

bool KVDB::set_int(const string& k, int int_v){
    string v = to_string(int_v);
    return set(k, v);
}

bool KVDB::del(const string& k)
{
    string sql("delete from kv where k='");
    sql.append(mod_);
    sql.append(k);
    sql.append("'");
    cout << "sql:" << sql << endl;

    return SQLITE_OK == SqliteEngine::execSQL(sql);
}

string KVDB::get(const string& k)
{
    string sql("select v from kv where k='");
    sql.append(mod_);
    sql.append(k);
    sql.append("'");
    cout << "sql:" << sql << endl;

    char **result;
    int row, column;
    SqliteEngine::query(sql, &result, &row, &column);
    if (row > 0)
    {
        std::string val(result[column]);
        SqliteEngine::freeQuery(result);
        return val; 
    }

    return invalid_val_; 
}

int KVDB::get_int(const string& k){
    string val = get(k);
    if (is_valid(val)){
        return atoi(val.c_str());
    }
    return invalid_int_val_;
}

const kv_map& KVDB::get_kvs()
{
    kv_map_.clear();
    string sql("select k, v from kv where k like '%");
    sql.append(mod_);
    sql.append("%'");
    cout << "sql:" << sql << endl;

    char **result;
    int row, column, col;
    SqliteEngine::query(sql, &result, &row, &column);
    for (int r = 1, col = column; r <= row; r++, col += column)
        kv_map_[result[col] + mod_.length()] = result[col + 1];

    SqliteEngine::freeQuery(result);

    return kv_map_;
}

bool KSetsDB::exist(const string& v)
{
    string sql("select v from kset where m='");
    sql.append(mod_);
    sql.append("' and v='");
    sql.append(v);
    sql.append("'");
    cout << "sql:" << sql << endl;

    char **result;
    int row, column;
    SqliteEngine::query(sql, &result, &row, &column);
    SqliteEngine::freeQuery(result);

    return row > 0;
}

bool KSetsDB::add(const string& v)
{
    string sql("insert into kset values('");
    sql.append(mod_);
    sql.append("','");
    sql.append(v);
    sql.append("')");
    cout << "sql:" << sql << endl;

    return SQLITE_OK == SqliteEngine::execSQL(sql);
}

bool KSetsDB::del(const string& v)
{
    string sql("delete from kset where m='");
    sql.append(mod_);
    sql.append("' and v='");
    sql.append(v);
    sql.append("'");
    cout << "sql:" << sql << endl;

    return SQLITE_OK == SqliteEngine::execSQL(sql);
}

bool KSetsDB::del_all()
{
    string sql("delete from kset where m='");
    sql.append(mod_);
    sql.append("'");
    cout << "sql:" << sql << endl;

    return SQLITE_OK == SqliteEngine::execSQL(sql);
}

const sets_vct& KSetsDB::get()
{
    sets_vct_.clear();   

    string sql("select v from kset where m='");
    sql.append(mod_);
    sql.append("'");
    cout << "sql:" << sql << endl;

    char **result;
    int row, column;
    SqliteEngine::query(sql, &result, &row, &column);
    for (int r = 1, col = column; r <= row; r++, col += column)
    {
        sets_vct_.push_back(result[col]);
    }
    SqliteEngine::freeQuery(result);

    return sets_vct_; 
}

void test_kvdb()
{
    KVDB kv;
    kv.select("test_kv");
    kv.set("k1", "v1");
    cout << "k1 exist:" << kv.exist("k1") << endl;
    cout << "k1:" << kv.get("k1") << endl;
    cout << "k1:" << kv.get("k1") << endl;
    kv.set("k1", "v11");
    cout << "k1:" << kv.get("k1") << endl;
    kv.set("k2", "v2");
    const kv_map& kvs = kv.get_kvs();
    for (auto kv:kvs)
        cout << kv.first << ":" << kv.second << endl;

    kv.del("k1");
    cout << "k1 exist:" << kv.exist("k1") << endl;
    cout << "k1:" << kv.get("k1") << endl;
    kv.set("k3", "v3");
    const kv_map& kvs2 = kv.get_kvs();
    for (auto kv:kvs2)
        cout << kv.first << ":" << kv.second << endl;
    cout << "-----------------------------" << endl;
    KSetsDB ksets;
    ksets.select("test_ksets");
    ksets.add("v1");
    ksets.add("v2");
    ksets.add("v3");

    const sets_vct& sets = ksets.get();
    for (auto v:sets)
        cout << v << endl;

    cout << "v1 exist:" << ksets.exist("v1") << endl;
    cout << "v3 exist:" << ksets.exist("v3") << endl;

    ksets.del("v1");
    cout << "v1 exist:" << ksets.exist("v1") << endl;

    const sets_vct& sets2 = ksets.get();
    for (auto v:sets2)
        cout << v << endl;

    ksets.del_all();

    cout << "v3 exist:" << ksets.exist("v3") << endl;
    const sets_vct& sets3 = ksets.get();
    cout << "sets3 size:" << sets3.size() << endl;
}

















