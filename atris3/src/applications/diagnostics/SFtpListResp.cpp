#include "SFtpListResp.h"
#include "StringHelper.h"

SFtpListResp::SFtpListResp()
{
    //ctor
}

SFtpListResp::~SFtpListResp()
{
    //dtor
}

int SFtpListResp::parse(const std::string& strResp, std::vector<FileInfo>& fileInfos)
{
    if(strResp.empty())
        return -1;
    std::vector<std::string> strLineInfos = StringHelper::split(strResp, "\n");
    FileInfo fileInfo;
    for(size_t i=0; i<strLineInfos.size(); ++i)
    {
        if(strLineInfos[i].empty())
            continue;

        int nPose = strLineInfos[i].find_last_of(" ");
        if(nPose == std::string::npos)
            continue;

        fileInfo.name = strLineInfos[i].substr(nPose+1, strLineInfos[i].length() - nPose -1);
        if(fileInfo.name.empty())
            continue;

        if(strLineInfos[i][0] == 'd')
        {
            fileInfo.type = NS_FTP::DIR;
        }
        else
        {
            fileInfo.type = NS_FTP::FILE;
        }
        fileInfos.push_back(fileInfo);
    }
    return 0;
}
