#include "StringHelper.h"


StringHelper::StringHelper()
{
}


StringHelper::~StringHelper()
{
}

std::vector<std::string> StringHelper::split(const std::string &s, const std::string &seperator)
{
    std::vector<std::string> result;
    typedef std::string::size_type string_size;
    string_size i = 0;

    while (i != s.size()){
        int flag = 0;
        while (i != s.size() && flag == 0){
            flag = 1;
            for (string_size x = 0; x < seperator.size(); ++x)
            if (s[i] == seperator[x]){
                ++i;
                flag = 0;
                break;
            }
        }

        flag = 0;
        string_size j = i;
        while (j != s.size() && flag == 0){
            for (string_size x = 0; x < seperator.size(); ++x)
            if (s[j] == seperator[x]){
                flag = 1;
                break;
            }
            if (flag == 0)
                ++j;
        }
        if (i != j){
            result.push_back(s.substr(i, j - i));
            i = j;
        }
    }
    return result;
}

std::string StringHelper::trim(const std::string &strValue)
{
    std::string s = strValue;
    if(!s.empty())
    {
        s.erase(0,s.find_first_not_of(" "));
        s.erase(s.find_last_not_of(" ")+1);
    }
    return s;
}
