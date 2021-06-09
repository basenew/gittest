/*
 * transferfile.h
 *
 *  Created on: 2018-7-17
 *      Author: fupj
 */

#ifndef TRANSFERFILE_H_
#define TRANSFERFILE_H_
#include <boost/thread.hpp>
#include <boost/signals2.hpp>
#include <string>
#include <functional>
#include <curl/curl.h>

#define QINIU_BUCKET_NAME "http://video.ubtrobot.com/"

typedef enum {
    TRANSFER_FILE_STARTED          = 0,
    TRANSFER_FILE_COMPLETED        = 1 << 0,
    TRANSFER_FILE_ERROR            = 1 << 1,
} TransferStates;

typedef std::function <void (TransferStates state, std::string &path)> NotifyHandler;

class TransferFile;
class FileObject {
public:
    std::string local_path;
    std::string remote_path;
    bool cancel;
    bool breakpoint;

    /*For uploading, Expired deletion of files*/
    int32_t deleteDays;

    virtual void progress(double progress /* 0~100 */) { }
    virtual void notify(TransferStates state, std::string msg = "", int code = 0)
    {
        if (notify_hander_) {
            notify_hander_(state, msg);
        }
    }
    virtual ~FileObject() { }
    FileObject()
      : easy_handle(NULL)
      , deleteDays(30)
      , cancel(false)
      , breakpoint(false)
      , notify_hander_(nullptr) { }
    CURL * getEasyHandle() { return easy_handle; }
    void SetNotifyCallback(const NotifyHandler &hander) {notify_hander_ = hander;}

private:
    CURL *easy_handle;
    friend class TransferFile;
    NotifyHandler notify_hander_;
};

class TransferFile {
public:
    virtual ~TransferFile() { }
    static void download(boost::shared_ptr<FileObject> obj);
    static void upload(boost::shared_ptr<FileObject> obj);
    static bool uploaded(std::string& key);
    static void DownloadFile(const std::string &file_name, const std::string &url, const NotifyHandler &hander);
    static void UploadFile(const std::string &file_name, const NotifyHandler &hander);

private:
    TransferFile() { }
    static void downloadThread(boost::shared_ptr<FileObject> obj, const std::string task_id);
    static void uploadThread(boost::shared_ptr<FileObject> obj, const std::string task_id);
    static void uploadTolocalSeverThread(boost::shared_ptr<FileObject> obj, const std::string task_id);
    static size_t writeFunction(void *buffer, size_t size, size_t nmemb, void *userdata);
    static int progressFunction(void *userdata, curl_off_t dltotal, curl_off_t dlnow, curl_off_t ultotal, curl_off_t ulnow);
    static double getDownloadFileLength(const std::string& url);
    static off_t getLocalFileLength(std::string path);
    static void downloadTaskThread();
    static void uploadTaskThread();
    static void DownloadFileNotifyHander(TransferStates state);
    static void UploadFileNotifyHander(TransferStates state);

private:
    static uint8_t dl_task_count_;
    static uint8_t ul_task_count_;
    static boost::mutex dl_task_mutex_;
    static boost::mutex ul_task_mutex_;
    static boost::mutex task_map_mutex_;
    static boost::condition_variable dl_task_cond_;
    static boost::condition_variable ul_task_cond_;
    static std::list<boost::shared_ptr<FileObject> > dl_task_queue_;
    static std::list<boost::shared_ptr<FileObject> > ul_task_queue_;
    static boost::thread* dl_task_thread_;
    static boost::thread* ul_task_thread_;
    static std::map<std::string, boost::shared_ptr<boost::thread> > task_map_;
    static std::vector<std::string> task_free_queue_;
    static bool qiniu_initialized_;
};

#endif /* TRANSFERFILE_H_ */
