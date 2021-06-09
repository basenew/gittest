#include "SFtpClient.h"
#include <string.h>
#include <iostream>

SFtpClient::SFtpClient()
{
    //ctor
}

SFtpClient::~SFtpClient()
{
    //dtor
}


CURL * SFtpClient::curl_init()
{
    curl_global_init(CURL_GLOBAL_DEFAULT);
    CURL *curl = curl_easy_init();
    if(NULL == curl)
    {
        fprintf(stderr, "Init curl failed.\n");
        exit(1);
    }
    return curl;
}


void SFtpClient::curl_set_download_opt(CURL *curl, const char *url, const char *user_key, FILE *file)
{
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_USERPWD, user_key);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, file);
//	curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);
}

void SFtpClient::curl_exit(CURL *curl)
{
    curl_easy_cleanup(curl);
    curl_global_cleanup();
}

CURLcode SFtpClient::curl_perform(CURL *curl)
{
    CURLcode ret = curl_easy_perform(curl);
    if(ret != 0)
    {
        fprintf(stderr, "Perform curl failed.\n");
        curl_exit(curl);
        exit(1);
    }
    return ret;
}


FTP_STATE SFtpClient::ftp_download(const FTP_OPT ftp_option)
{
    FTP_STATE state;
    CURL *curl;
    FILE *fp = fopen(ftp_option.localfile.c_str(), "w");
    if(NULL == fp)
    {
        fprintf(stderr, "Open file failed at %s:%d\n", __FILE__, __LINE__);
        return FTP_UPLOAD_FAILED;
    }

    curl = curl_init();
    curl_set_download_opt(curl, ftp_option.remotefile.c_str(), ftp_option.user_key.c_str(), fp);
    if(CURLE_OK == curl_perform(curl))
    {
        state = FTP_DOWNLOAD_SUCCESS;
    }
    else
    {
        state = FTP_DOWNLOAD_FAILED;
        fprintf(stderr, "Open %s failed at %s:%d\n", ftp_option.remotefile.c_str(), __FILE__, __LINE__);
    }


    curl_exit(curl);
    fclose(fp);
    return state;
}

static size_t write_response(void *ptr, size_t size1, size_t nmemb, void *stream)
{
    if(nullptr == stream)
        return 0;
    SBuffer* temp = (SBuffer*)stream;
    size_t realsize = size1 * nmemb;
    temp->SetData((char*)ptr, realsize);
    return realsize;
}

FTP_STATE SFtpClient::ListFile(SBuffer* buffer, const char* url, const char *user_key)
{
    FTP_STATE state;
    CURL *curl;
    curl = curl_init();
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_USERPWD, user_key);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, buffer);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_response);
    if(CURLE_OK == curl_perform(curl))
        state = FTP_DOWNLOAD_SUCCESS;
    else
        state = FTP_DOWNLOAD_FAILED;

    curl_exit(curl);
    return state;
}
