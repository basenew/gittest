#ifndef SCOMMON_H_INCLUDED
#define SCOMMON_H_INCLUDED
#include <string>
/*FTP OPERATION CODE*/
typedef enum FTP_STATE
{
    FTP_UPLOAD_SUCCESS,
    FTP_UPLOAD_FAILED,
    FTP_DOWNLOAD_SUCCESS,
    FTP_DOWNLOAD_FAILED
} FTP_STATE;

/*FTP OPERATIONS OPTIONS*/
typedef struct FTP_OPT
{
    std::string     remotefile;		/*url of ftp*/
    std::string     user_key;		/*username:password*/
    std::string     localfile;		/*filepath*/
} FTP_OPT;

namespace NS_FTP
{
enum En_FileType
{
    FILE  = 0,
    DIR =  1
};
}



struct FileInfo
{
    NS_FTP::En_FileType  type;
    std::string  name;
};

#endif // SCOMMON_H_INCLUDED