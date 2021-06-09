#ifndef XML_H_
#define XML_H_

#include <string>
#include <map>
#include <vector>
#include <tinyxml.h>
#include <tinystr.h>

typedef std::map<std::string, std::string> String_Map;
typedef std::pair <std::string, std::string> String_Pair;

class Xml{
public:
    static bool createBaseXml(std::string &xmlString, const std::string &strRootNodeName);
    static bool createBaseXml(std::string &xmlString, const std::string &strRootNodeName, 
                              const std::vector<std::string> &firstLevelNodeName);
    static bool addNodeText(std::string &xmlString, const std::string &strParNodeName, 
                            const std::string &strNodeName, const std::string &strText);
    static bool delNode(std::string &xmlString, const std::string &strNodeName);
    static bool modifyNodeText(std::string &xmlString, const std::string &strNodeName, const std::string &strText);
    static bool queryNodeText(const std::string &xmlString, const std::string &strNodeName, std::string &strText);
    static bool addNodeAttribute(std::string &xmlString, const std::string &strParNodeName,
                                 const std::string &strNodeName, std::map<std::string, std::string> &AttMap);
    static bool modifyNodeAttribute(std::string &xmlString, const std::string &strNodeName, 
                                    std::map<std::string, std::string> &AttMap);
    static bool queryNodeAttribute(const std::string &xmlString, const std::string &strNodeName, 
                                    std::map<std::string, std::string> &attMap);
    static bool queryAllChildNodeAttribute(const std::string &xmlString, const std::string &strParNodeName,
                                           const std::string &strChildNodeName, std::vector<String_Map> &attVector);
    static bool queryAllChildNodeText(const std::string &xmlString, const std::string &strParNodeName,
                                      std::vector<std::string> &childNodeText);
    static bool isChildNodeInParNode(const std::string &xmlString, const std::string &strParNodeName,
                                     const std::string &strChildNodeName);
private:
    static bool getNodePointerByName(TiXmlElement* pRootEle, const std::string &strNodeName, TiXmlElement* &node);

};

#endif