/*
 * transferfile.cpp
 *
 *  Created on: 2018-7-17
 *      Author: fupj
 */

#include "transferfile.h"
#include <libgen.h>
#include <sys/stat.h>
#include <stdio.h>
#include <fcntl.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <ros/time.h>
#include "utils/utils.h"
#include "config/config.h"
#include "log/log.h"

extern "C"
{
#include "qiniu/qiniu/io.h"
#include "qiniu/qiniu/resumable_io.h"
#include "qiniu/qiniu/rs.h"
#include "qiniu/qiniu/cdn.h"
#include "qiniu/qiniu/base.h"
}

#define QINIU_ACCESS_KEY "OJb5DHhgOxDo42se8R2JvwLyaykWLUBYowqMA3Nu"
#define QINIU_SECRET_KEY "tRfcP40zLvGUwfVANQEClOnyn2ATb2spLki9K7cH"

#define TRANSFER_FILE_TAG "TransferFile->"
#define DOWNLOAD_TIMEOUT  (2*60) // seconds
#define DOWNLOAD_TASK_MAX 3
#define UPLOAD_TASK_MAX 1

bool TransferFile::qiniu_initialized_ = false;
uint8_t TransferFile::dl_task_count_ = 0;
uint8_t TransferFile::ul_task_count_ = 0;
boost::mutex TransferFile::dl_task_mutex_;
boost::mutex TransferFile::ul_task_mutex_;
boost::mutex TransferFile::task_map_mutex_;
boost::condition_variable TransferFile::dl_task_cond_;
boost::condition_variable TransferFile::ul_task_cond_;
std::vector<std::string> TransferFile::task_free_queue_;
std::list<boost::shared_ptr<FileObject> > TransferFile::dl_task_queue_;
std::list<boost::shared_ptr<FileObject> > TransferFile::ul_task_queue_;
std::map<std::string, boost::shared_ptr<boost::thread> > TransferFile::task_map_;
boost::thread* TransferFile::dl_task_thread_ = new boost::thread(boost::bind(&TransferFile::downloadTaskThread));
boost::thread* TransferFile::ul_task_thread_ = new boost::thread(boost::bind(&TransferFile::uploadTaskThread));

typedef struct {
    boost::shared_ptr<FileObject> obj;
    CURL *handle;
    curl_off_t resumeByte;
    curl_off_t dlnow;
    uint32_t dltime;
    double downloadFileLength;
} DownloadUserData_t;

typedef struct {
    boost::shared_ptr<FileObject> obj;
    int64_t progress;
    int64_t uploadFileLength;
} UploadUserData_t;

static size_t nousecb(char *buffer, size_t x, size_t y, void *userdata) {
    (void)buffer;
    (void)userdata;
    return x * y;
}

static int rio_notify(void* recvr, int blkIdx, int blkSize, Qiniu_Rio_BlkputRet* ret) {
    (void)blkIdx;
    UploadUserData_t *data = static_cast<UploadUserData_t *>(recvr);
    uint64_t progress = 0;
    uint32_t blk = blkSize;
    if (blk == ret->offset) {
        data->progress += blk;
        progress = data->progress;
    } else {
        progress = data->progress + ret->offset;
    }
    data->obj->progress(((double)progress/data->uploadFileLength)*100);
    return 0;
}

void TransferFile::download(boost::shared_ptr<FileObject> obj) {
    boost::unique_lock<boost::mutex> lock(TransferFile::dl_task_mutex_);
    TransferFile::dl_task_queue_.push_back(obj);
    TransferFile::dl_task_cond_.notify_one();
}

void TransferFile::upload(boost::shared_ptr<FileObject> obj) {
    boost::unique_lock<boost::mutex> lock(TransferFile::ul_task_mutex_);
    if(Config::get_instance()->hfs_type == "qiniu"){
        if (!TransferFile::qiniu_initialized_) {
            Qiniu_Global_Init(-1);
            TransferFile::qiniu_initialized_ = true;
        }
    }
    TransferFile::ul_task_queue_.push_back(obj);
    TransferFile::ul_task_cond_.notify_one();
}

void TransferFile::DownloadFileNotifyHander(TransferStates state)
{
    if (state == TRANSFER_FILE_COMPLETED) {
        // std::cout << "download file success" << std::endl;
    } else if (state == TRANSFER_FILE_ERROR) {
   
    }
}

void TransferFile::DownloadFile(const std::string &file_name, const std::string &url, const NotifyHandler &hander)
{
    boost::shared_ptr<FileObject> download_file = boost::shared_ptr<FileObject> (new FileObject());
    download_file->local_path = file_name;
    download_file->remote_path = url;

    //download_file->SetNotifyCallback(std::bind(&hander, std::placeholders::_1));
    download_file->SetNotifyCallback(hander);
    download(download_file);
}

void TransferFile::UploadFileNotifyHander(TransferStates state)
{
    if (state == TRANSFER_FILE_COMPLETED) {
         // std::cout << "uploadfile file success" << std::endl;
    } else if (state == TRANSFER_FILE_ERROR) {
        
    }
}

void TransferFile::UploadFile(const std::string &local_path, const NotifyHandler &hander)
{
    boost::shared_ptr<FileObject> upload_file = boost::shared_ptr<FileObject> (new FileObject());
    upload_file->local_path = local_path;

    std::string file_name;
    std::size_t idx = local_path.rfind("/");
    if (idx != std::string::npos) {
        file_name = local_path.substr(idx+1);
    }else{
        file_name = local_path;
    }
    if (Config::get_instance()->hfs_type == "qiniu") {
        upload_file->remote_path = QINIU_BUCKET_NAME + file_name;
    } else {
        if (*(Config::get_instance()->hfs_url.end()-1) != '/') {
            upload_file->remote_path = Config::get_instance()->hfs_url + "/" + file_name;
        } else {
            upload_file->remote_path = Config::get_instance()->hfs_url + file_name;
        }
    }

    upload_file->deleteDays = 7;
    //upload_file->SetNotifyCallback(std::bind(&TransferFile::UploadFileNotifyHander, std::placeholders::_1));
    upload_file->SetNotifyCallback(hander);
    upload(upload_file);
}

off_t TransferFile::getLocalFileLength(std::string path) {
    int ret;
    struct stat fileStat;
    memset(&fileStat, 0, sizeof(fileStat));
    ret = stat(path.c_str(), &fileStat);
    if (ret == 0) {
        return fileStat.st_size;
    }
    return 0;
}

double TransferFile::getDownloadFileLength(const std::string& url) {
    CURL *easy_handle = NULL;
    int ret = CURLE_OK;
    double size = -1;
    do {
        easy_handle = curl_easy_init();
        if (!easy_handle) {
            log_error(TRANSFER_FILE_TAG"%s curl_easy_init error", __FUNCTION__);
            break;
        }

        // Only get the header data
        ret = curl_easy_setopt(easy_handle, CURLOPT_URL, url.c_str());
        ret |= curl_easy_setopt(easy_handle, CURLOPT_HEADER, 1L);
        ret |= curl_easy_setopt(easy_handle, CURLOPT_NOBODY, 1L);
        ret |= curl_easy_setopt(easy_handle, CURLOPT_WRITEFUNCTION, nousecb);
        ret |= curl_easy_setopt(easy_handle, CURLOPT_CONNECTTIMEOUT, 30L);
        ret |= curl_easy_setopt(easy_handle, CURLOPT_NOSIGNAL, 1L);

        if (ret != CURLE_OK) {
            log_error(TRANSFER_FILE_TAG"%s curl_easy_setopt error", __FUNCTION__);
            break;
        }

        ret = curl_easy_perform(easy_handle);
        if (ret != CURLE_OK) {
            log_error(TRANSFER_FILE_TAG"%s curl_easy_perform error(%d):%s",
                    __FUNCTION__, ret, curl_easy_strerror(static_cast<CURLcode>(ret)));
            break;
        }

        // size = -1 if no Content-Length return or Content-Length=0
        ret = curl_easy_getinfo(easy_handle, CURLINFO_CONTENT_LENGTH_DOWNLOAD, &size);
        if (ret != CURLE_OK) {
            log_error(TRANSFER_FILE_TAG"%s curl_easy_getinfo error", __FUNCTION__);
            break;
        }
    } while (0);

    if (easy_handle != NULL) {
        curl_easy_cleanup(easy_handle);
    }

    return size;
}

size_t TransferFile::writeFunction(void *buffer, size_t size, size_t nmemb, void *userdata) {
    FILE *fp = static_cast<FILE *>(userdata);
    size_t length = fwrite(buffer, size, nmemb, fp);
    if (length != nmemb) {
        return length;
    }
    return size * nmemb;
}

int TransferFile::progressFunction(void* userdata, curl_off_t dltotal,
        curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow) {
    DownloadUserData_t *data = static_cast<DownloadUserData_t *>(userdata);
    double speed = 0, progress = 0;
    curl_easy_getinfo(data->handle, CURLINFO_SPEED_DOWNLOAD, &speed);
    if (dltotal != 0 && speed != 0) {
        progress = (dlnow + data->resumeByte) / data->downloadFileLength * 100;
        data->obj->progress(progress);
    }

    if (data->obj->cancel) {
        log_error(TRANSFER_FILE_TAG"%s curl_easy_perform cancel", __FUNCTION__);
        return -1;
    }
    
    uint32_t sec = ros::Time::now().sec;
    if (data->dlnow == dlnow) {
        if (data->dltime > sec) {
            data->dltime = sec;
        }

        // timeout DOWNLOAD_TIMEOUT seconds
        if ((sec - data->dltime) >= DOWNLOAD_TIMEOUT) {
            log_error(TRANSFER_FILE_TAG"%s curl_easy_perform timeout(%f seconds)",
                    __FUNCTION__, ((sec - data->dltime)*1.0));
            return -1;
        }
    } else {
        data->dlnow = dlnow;
        data->dltime = sec;
    }
    return 0;
}

void TransferFile::downloadThread(boost::shared_ptr<FileObject> obj, const std::string task_id) {
    CURL *easy_handle = NULL;
    FILE *fp = NULL;
    int ret = -1;
    std::string partPath, saveTo, remotePath, saveDir;

    do {
        if (obj == nullptr) {
            log_error(TRANSFER_FILE_TAG"%s curl_easy_init error", __FUNCTION__);
            break;
        }
        
        if (obj->cancel) {
            log_warn(TRANSFER_FILE_TAG"%s cancel", __FUNCTION__);
            break;
        }

        remotePath = obj->remote_path;
        partPath = obj->local_path + ".part";
        saveTo = obj->local_path;

        if (!obj->breakpoint) {
            remove(partPath.c_str());
        }
        
        std::size_t idx = saveTo.rfind("/");
        if (idx != std::string::npos) {
            saveDir = saveTo.substr(0, idx);
            if(access(saveDir.c_str(), F_OK) !=0 ) {
                Utils::createDir(saveDir);
            }
        }

        double downloadFileLength = TransferFile::getDownloadFileLength(remotePath);
        if (downloadFileLength < 0) {
            log_warn(TRANSFER_FILE_TAG"%s getDownloadFileLength %f", __FUNCTION__, downloadFileLength);
        }

        easy_handle = curl_easy_init();
        if (!easy_handle) {
            log_error(TRANSFER_FILE_TAG"%s curl_easy_init error", __FUNCTION__);
            obj->notify(TRANSFER_FILE_ERROR, "curl_easy_init error");
            break;
        }

        obj->easy_handle = easy_handle;

        fp = fopen(partPath.c_str(), "ab+");
        if (fp == NULL) {
            log_error(TRANSFER_FILE_TAG"%s file open failed: %s", __FUNCTION__, partPath.c_str());
            obj->notify(TRANSFER_FILE_ERROR, "open local file failed");
            break;
        }

        ret = curl_easy_setopt(easy_handle, CURLOPT_URL, remotePath.c_str());
        ret |= curl_easy_setopt(easy_handle, CURLOPT_WRITEFUNCTION, &TransferFile::writeFunction);
        ret |= curl_easy_setopt(easy_handle, CURLOPT_WRITEDATA, fp);
        ret |= curl_easy_setopt(easy_handle, CURLOPT_FAILONERROR, 1L);
        ret |= curl_easy_setopt(easy_handle, CURLOPT_CONNECTTIMEOUT,30L);
        ret |= curl_easy_setopt(easy_handle, CURLOPT_NOSIGNAL, 1L);
        ret |= curl_easy_setopt(easy_handle, CURLOPT_FORBID_REUSE, 1);

        curl_off_t resumeByte = getLocalFileLength(partPath);
        if (resumeByte > 0) {
            ret |= curl_easy_setopt(easy_handle, CURLOPT_RESUME_FROM_LARGE, resumeByte);
        }

        DownloadUserData_t data;
        data.obj= obj;
        data.handle = easy_handle;
        data.resumeByte = resumeByte;
        data.dlnow = -1;
        data.dltime = ros::Time::now().sec;
        data.downloadFileLength = downloadFileLength;

        if (downloadFileLength > 0) {
          ret |= curl_easy_setopt(easy_handle, CURLOPT_NOPROGRESS, 0L);
          ret |= curl_easy_setopt(easy_handle, CURLOPT_XFERINFOFUNCTION, &TransferFile::progressFunction);
          ret |= curl_easy_setopt(easy_handle, CURLOPT_XFERINFODATA, &data);
        }

        if (ret != CURLE_OK) {
            log_error(TRANSFER_FILE_TAG"%s curl_easy_setopt error", __FUNCTION__);
            obj->notify(TRANSFER_FILE_ERROR, "curl_easy_setopt error");
            break;
        }

        obj->notify(TRANSFER_FILE_STARTED);
        ret = curl_easy_perform(easy_handle);
        if (ret != CURLE_OK) {
            log_error(TRANSFER_FILE_TAG"%s curl_easy_perform error(%d):%s", __FUNCTION__, ret, curl_easy_strerror(static_cast<CURLcode>(ret)));
            if (ret == CURLE_HTTP_RETURNED_ERROR) {
                int code = 0;
                curl_easy_getinfo(easy_handle, CURLINFO_RESPONSE_CODE, &code);
                log_error(TRANSFER_FILE_TAG"%s HTTP error code: %d", __FUNCTION__, code);
            }
            obj->notify(TRANSFER_FILE_ERROR, "curl_easy_perform error");
        }
    } while (0);

    if (fp != NULL) {
        fclose(fp);
    }

    if (ret == CURLE_OK) {
        remove(saveTo.c_str());
        rename(partPath.c_str(), saveTo.c_str());
        obj->notify(TRANSFER_FILE_COMPLETED, obj->local_path);
    }

    if (easy_handle != NULL) {
        obj->easy_handle = NULL;
        curl_easy_cleanup(easy_handle);
    }

    boost::unique_lock<boost::mutex> dl_task_lock(TransferFile::dl_task_mutex_);
    boost::unique_lock<boost::mutex> task_map_lock(TransferFile::task_map_mutex_);
    TransferFile::task_free_queue_.push_back(task_id);
    TransferFile::dl_task_count_ -= 1;
    TransferFile::dl_task_cond_.notify_one();
}

void TransferFile::uploadTolocalSeverThread(boost::shared_ptr<FileObject> obj, const std::string task_id) {
    char* key;
    char *d = NULL;
    std::string bucket;
    std::string resp;
    off_t bytes;
    do {
        std::size_t idx = obj->remote_path.rfind("/");
        if (idx != std::string::npos) {
            bucket = obj->remote_path.substr(0, idx+1);
        }
/*
        if (bucket != Config::get_instance()->hfs_url) {
            log_error(TRANSFER_FILE_TAG"%s bucket error: %s", __FUNCTION__, bucket.c_str());
            obj->notify(TRANSFER_FILE_ERROR);
            break; 
        }
 */               
        if ((key = basename((char*)obj->remote_path.c_str())) == NULL) {
            log_error(TRANSFER_FILE_TAG"%s base name is null: %s", __FUNCTION__, obj->remote_path.c_str());
            obj->notify(TRANSFER_FILE_ERROR);
            break;
        }

        if(access(obj->local_path.c_str(), F_OK) !=0 ) {
            log_error(TRANSFER_FILE_TAG"%s local file no found: %s", __FUNCTION__, obj->local_path.c_str());
            obj->notify(TRANSFER_FILE_ERROR);
            break;
        }
        if ((bytes = getLocalFileLength(obj->local_path.c_str())) <= 0) {
            log_error(TRANSFER_FILE_TAG"%s invalid filesize: %ld.", __FUNCTION__, bytes);
            obj->notify(TRANSFER_FILE_ERROR);
            break;
        }
        
        struct stat st;
        if(stat(obj->local_path.c_str(), &st) != 0){
            log_error(TRANSFER_FILE_TAG"%s stat file fail:%s err:%d", __FUNCTION__, obj->local_path.c_str(), errno);
            obj->notify(TRANSFER_FILE_ERROR);
            break;
        }
      
        obj->notify(TRANSFER_FILE_STARTED);
        log_debug(TRANSFER_FILE_TAG"%s   obj remote path:%s , obj local path:%s", __FUNCTION__, obj->remote_path.c_str(), obj->local_path.c_str());
        if(Utils::get_instance()->http_local_post_file(Config::get_instance()->hfs_url, obj->local_path, resp) != 0){
            log_error(TRANSFER_FILE_TAG"%s url :%s post file fail.",  __FUNCTION__, obj->remote_path.c_str());
            obj->notify(TRANSFER_FILE_ERROR);
        }else{
            obj->notify(TRANSFER_FILE_COMPLETED, obj->remote_path);
        }    

    }while(0);

    boost::unique_lock<boost::mutex> ul_task_lock(TransferFile::ul_task_mutex_);
    boost::unique_lock<boost::mutex> task_map_lock(TransferFile::task_map_mutex_);
    TransferFile::task_free_queue_.push_back(task_id);
    TransferFile::ul_task_count_ -= 1;
    TransferFile::ul_task_cond_.notify_one();
}

void TransferFile::uploadThread(boost::shared_ptr<FileObject> obj, const std::string task_id) {
    char* key, *uptoken;
    char policy[120] = {0};
    Qiniu_RS_PutPolicy putPolicy;
    Qiniu_Error err;
    Qiniu_Rio_PutRet putRet;
    Qiniu_Mac mac;
    Qiniu_Client client;
    std::string bucket;
    off_t bytes;

    do {
        std::size_t idx = obj->remote_path.rfind("/");
        if (idx != std::string::npos) {
            bucket = obj->remote_path.substr(0, idx);
        }

        if (bucket != "http://video.ubtrobot.com") {
            log_error(TRANSFER_FILE_TAG"%s bucket error: %s", __FUNCTION__, bucket.c_str());
            obj->notify(TRANSFER_FILE_ERROR);
            break;
        }

        if ((key = basename((char*)obj->remote_path.c_str())) == NULL) {
            log_error(TRANSFER_FILE_TAG"%s base name is null: %s", __FUNCTION__, obj->remote_path.c_str());
            obj->notify(TRANSFER_FILE_ERROR);
            break;
        }

        if(access(obj->local_path.c_str(), F_OK) !=0 ) {
            log_error(TRANSFER_FILE_TAG"%s local file no found: %s", __FUNCTION__, obj->local_path.c_str());
            obj->notify(TRANSFER_FILE_ERROR);
            break;
        }
        if ((bytes = getLocalFileLength(obj->local_path.c_str())) <= 0) {
            log_error(TRANSFER_FILE_TAG"%s invalid filesize: %ld.", __FUNCTION__, bytes);
            obj->notify(TRANSFER_FILE_ERROR);
            break;
        }

        Qiniu_Zero(putPolicy);
        sprintf(policy, "%s:%s", "ubtech", key);
        putPolicy.scope = policy;
        putPolicy.expires = 24*3600;
        putPolicy.deleteAfterDays = obj->deleteDays;
        mac.accessKey = QINIU_ACCESS_KEY;
        mac.secretKey = QINIU_SECRET_KEY;
        if ((uptoken = Qiniu_RS_PutPolicy_Token(&putPolicy, &mac)) == NULL) {
            log_error(TRANSFER_FILE_TAG"%s create uptoken failed.", __FUNCTION__);
            obj->notify(TRANSFER_FILE_ERROR);
            break;
        }

        Qiniu_Client_InitMacAuth(&client, 1024, &mac);
        obj->notify(TRANSFER_FILE_STARTED);
#if 1
        Qiniu_Rio_PutExtra extra;
        Qiniu_Zero(extra);
        UploadUserData_t recvr = { obj, 0, bytes };
        extra.notify = rio_notify;
        extra.notifyRecvr = &recvr;
        err = Qiniu_Rio_PutFile(&client, &putRet, uptoken, key, obj->local_path.c_str(), &extra);
#else
        err = Qiniu_Io_PutFile(&client, &putRet, uptoken, key, obj->local_path.c_str(), NULL);
#endif
        if (err.code != 200) {
            log_error(TRANSFER_FILE_TAG"%s error code: %d, message: %s",  __FUNCTION__, err.code, err.message);
            log_error(TRANSFER_FILE_TAG"%s error response header:\n%s",  __FUNCTION__, Qiniu_Buffer_CStr(&client.respHeader));
            log_error(TRANSFER_FILE_TAG"%s error response body:\n%s",  __FUNCTION__, Qiniu_Buffer_CStr(&client.b));
            obj->notify(TRANSFER_FILE_ERROR);
        } else {
            obj->notify(TRANSFER_FILE_COMPLETED, obj->remote_path);
        }

        Qiniu_Client_Cleanup(&client);

    } while (0);

    boost::unique_lock<boost::mutex> ul_task_lock(TransferFile::ul_task_mutex_);
    boost::unique_lock<boost::mutex> task_map_lock(TransferFile::task_map_mutex_);
    TransferFile::task_free_queue_.push_back(task_id);
    TransferFile::ul_task_count_ -= 1;
    TransferFile::ul_task_cond_.notify_one();
}

void TransferFile::downloadTaskThread() {
    boost::unique_lock<boost::mutex> dl_task_lock(TransferFile::dl_task_mutex_, boost::defer_lock);
    boost::unique_lock<boost::mutex> task_map_lock(TransferFile::task_map_mutex_, boost::defer_lock);
    while (true) {
        dl_task_lock.lock();
        if (TransferFile::dl_task_count_ >= DOWNLOAD_TASK_MAX || TransferFile::dl_task_queue_.size() <= 0) {
            TransferFile::dl_task_cond_.wait(dl_task_lock);
        }

        if (TransferFile::dl_task_count_ < DOWNLOAD_TASK_MAX && TransferFile::dl_task_queue_.size() > 0) {
            boost::uuids::random_generator rgen;
            std::size_t hash_value = boost::uuids::hash_value(rgen());
            std::stringstream ss; ss<<hash_value;
            std::string task_id = ss.str();

            boost::shared_ptr<FileObject> obj = TransferFile::dl_task_queue_.front();
            TransferFile::dl_task_queue_.pop_front();

            task_map_lock.lock();
            if (!TransferFile::task_map_.count(task_id)) {
                TransferFile::dl_task_count_ += 1;
                boost::shared_ptr<boost::thread> task(new boost::thread(boost::bind(&TransferFile::downloadThread, obj, task_id)));
                TransferFile::task_map_[task_id] = task;
            } else {
                log_error(TRANSFER_FILE_TAG"%s Already exits: %s", __FUNCTION__, obj->remote_path.c_str());
                obj->notify(TRANSFER_FILE_ERROR, "remote_url already exits");
            }
            task_map_lock.unlock();
        }

        if (TransferFile::task_free_queue_.size() > 0) {
            task_map_lock.lock();
            for (std::size_t i=0; i < TransferFile::task_free_queue_.size(); i++) {
                std::string task_id = TransferFile::task_free_queue_[i];
                if (TransferFile::task_map_.count(task_id)) {
                    boost::shared_ptr<boost::thread> task = TransferFile::task_map_[task_id];
                    task->interrupt(); task->join();
                    TransferFile::task_map_.erase(task_id);
                }
            }
            task_map_lock.unlock();
        }
        dl_task_lock.unlock();
    }
}

void TransferFile::uploadTaskThread() {
    boost::unique_lock<boost::mutex> ul_task_lock(TransferFile::ul_task_mutex_, boost::defer_lock);
    boost::unique_lock<boost::mutex> task_map_lock(TransferFile::task_map_mutex_, boost::defer_lock);
    while (true) {
        ul_task_lock.lock();
        if (TransferFile::ul_task_count_ >= UPLOAD_TASK_MAX || TransferFile::ul_task_queue_.size() <= 0) {
            TransferFile::ul_task_cond_.wait(ul_task_lock);
        }

        if (TransferFile::ul_task_count_ < UPLOAD_TASK_MAX && TransferFile::ul_task_queue_.size() > 0) {

            boost::uuids::random_generator rgen;
            std::size_t hash_value = boost::uuids::hash_value(rgen());
            std::stringstream ss; ss<<hash_value;
            std::string task_id = ss.str();

            boost::shared_ptr<FileObject> obj = TransferFile::ul_task_queue_.front();
            TransferFile::ul_task_queue_.pop_front();

            task_map_lock.lock();
            if (!TransferFile::task_map_.count(task_id)) {
                TransferFile::ul_task_count_ += 1;
                if(Config::get_instance()->hfs_type == "qiniu"){
                    boost::shared_ptr<boost::thread> task(new boost::thread(boost::bind(&TransferFile::uploadThread, obj, task_id)));
                    TransferFile::task_map_[task_id] = task;
                }else{
                    boost::shared_ptr<boost::thread> task(new boost::thread(boost::bind(&TransferFile::uploadTolocalSeverThread, obj, task_id)));
                    TransferFile::task_map_[task_id] = task;
                }
                
            } else {
                log_error(TRANSFER_FILE_TAG"%s Already exits: %s", __FUNCTION__, obj->remote_path.c_str());
                obj->notify(TRANSFER_FILE_ERROR, "remote_url already exits");
            }
            task_map_lock.unlock();
        }

        if (TransferFile::task_free_queue_.size() > 0) {
            task_map_lock.lock();
            for (std::size_t i=0; i < TransferFile::task_free_queue_.size(); i++) {
                std::string task_id = TransferFile::task_free_queue_[i];
                if (TransferFile::task_map_.count(task_id)) {
                    boost::shared_ptr<boost::thread> task = TransferFile::task_map_[task_id];
                    task->interrupt(); task->join();
                    TransferFile::task_map_.erase(task_id);
                }
            }
            task_map_lock.unlock();
        }
        ul_task_lock.unlock();
    }
}

bool TransferFile::uploaded(std::string& key) {
    Qiniu_Mac mac;
    mac.accessKey = QINIU_ACCESS_KEY;
    mac.secretKey = QINIU_SECRET_KEY;

    Qiniu_Client client;
    Qiniu_Client_InitMacAuth(&client, 1024, &mac);
    Qiniu_RS_StatRet ret;
    Qiniu_Error err = Qiniu_RS_Stat(&client, &ret, "ubtech", key.c_str());
    Qiniu_Client_Cleanup(&client);
    return ((err.code == 612) ? false : true);
}

