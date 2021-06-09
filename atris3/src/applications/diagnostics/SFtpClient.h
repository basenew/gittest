#ifndef SFTPCLIENT_H
#define SFTPCLIENT_H
#include <stdio.h>
#include <stdlib.h>
#include <curl/curl.h>
#include "SCommon.h"
#include "SBuffer.h"

class SFtpClient
{
    public:
        SFtpClient();
        ~SFtpClient();
        FTP_STATE ftp_download(const FTP_OPT ftp_option);
        FTP_STATE ListFile(SBuffer* buffer, const char* url, const char *user_key);

        int DownLoad();
    protected:

    private:
        CURL * curl_init();
        void curl_set_download_opt(CURL *curl, const char *url, const char *user_key, FILE *file);
        void curl_exit(CURL *curl);
        CURLcode curl_perform(CURL *curl);


};

#endif // SFTPCLIENT_H
