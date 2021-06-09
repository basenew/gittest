#ifndef SFTPLISTRESP_H
#define SFTPLISTRESP_H
#include "SCommon.h"
#include <vector>
class SFtpListResp
{
    public:
        SFtpListResp();
        ~SFtpListResp();

        int parse(const std::string& strResp, std::vector<FileInfo>& fileInfos);

    protected:

    private:
};

#endif // SFTPLISTRESP_H
