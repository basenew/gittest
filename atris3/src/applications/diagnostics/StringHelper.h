#pragma once
#include <string>
#include <vector>

class StringHelper
{
public:
    StringHelper();
    ~StringHelper();
    static std::vector<std::string> split(const std::string &s, const std::string &seperator);
   static std::string trim(const std::string& s);
};

