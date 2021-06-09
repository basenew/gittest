#include "xml.h"

bool Xml::createBaseXml(std::string &xmlString, const std::string &strRootNodeName)
{
	TiXmlDocument *pDoc = new TiXmlDocument;
	if (NULL == pDoc) {
		return false;
	}
	TiXmlDeclaration *pDeclaration = new TiXmlDeclaration("1.0", "", "");
	if (NULL == pDeclaration) {
		return false;
	}
	pDoc->LinkEndChild(pDeclaration);

	TiXmlElement *pRootEle = new TiXmlElement(strRootNodeName.c_str());
	if (NULL == pRootEle) {
		return false;
	}
	pDoc->LinkEndChild(pRootEle);

	TiXmlPrinter printer;
	pDoc->Accept(&printer);
	xmlString = printer.CStr();
	pDoc->Clear();
	delete pDoc;
	pDoc = nullptr;

	return true;
}

bool Xml::createBaseXml(std::string &xmlString, const std::string &strRootNodeName, 
						const std::vector<std::string> &firstLevelNodeName)
{
	TiXmlDocument *pDoc = new TiXmlDocument;
	if (NULL == pDoc) {
		return false;
	}
	TiXmlDeclaration *pDeclaration = new TiXmlDeclaration("1.0", "", "");
	if (NULL == pDeclaration) {
		return false;
	}
	pDoc->LinkEndChild(pDeclaration);

	TiXmlElement *pRootEle = new TiXmlElement(strRootNodeName.c_str());
	if (NULL == pRootEle) {
		return false;
	}
	pDoc->LinkEndChild(pRootEle);

	for (unsigned int i = 0; i < firstLevelNodeName.size(); i++) {
		TiXmlElement *element = new TiXmlElement(firstLevelNodeName[i].c_str());
		if (NULL == element) {
			continue;
		} 
		pRootEle->LinkEndChild(element);
	}

	TiXmlPrinter printer;
	pDoc->Accept(&printer);
	xmlString = printer.CStr();
	pDoc->Clear();
	delete pDoc;
	pDoc = nullptr;
	
	return true;
}


bool Xml::getNodePointerByName(TiXmlElement* pRootEle, const std::string &strNodeName, TiXmlElement* &node)
{
	if (pRootEle == NULL) {
		return false;
	}
	if (strNodeName == pRootEle->Value()) {
		node = pRootEle;
		return true;
	}
	TiXmlElement* pEle = pRootEle;
	for (pEle = pRootEle->FirstChildElement(); pEle; pEle = pEle->NextSiblingElement()) {
		if (getNodePointerByName(pEle, strNodeName, node)) {
			return true;
		}
	}
	return false;
}

bool Xml::addNodeText(std::string &xmlString, const std::string &strParNodeName, 
					  const std::string &strNodeName, const std::string &strText)
{
	if (xmlString.empty()) {
		return false;
	}

	TiXmlDocument *pDoc = new TiXmlDocument();
	if (NULL == pDoc) {
		return false;
	}
	pDoc->Parse(xmlString.c_str());
	TiXmlElement *pRootEle = pDoc->RootElement();
	if (NULL == pRootEle) {
		return false;
	}
	TiXmlElement *pNode = NULL;
	getNodePointerByName(pRootEle, strParNodeName, pNode);
	if (NULL != pNode) {
		TiXmlElement *pNewNode = new TiXmlElement(strNodeName.c_str());
		if (NULL == pNewNode) {
			return false;
		}
		TiXmlText *pNewValue = new TiXmlText(strText.c_str());
		pNewNode->LinkEndChild(pNewValue);
		pNode->InsertEndChild(*pNewNode);

		TiXmlPrinter printer;
		pDoc->Accept(&printer);
		xmlString = printer.CStr();
		pDoc->Clear();
		delete pDoc;
		pDoc = nullptr;
		return true;
	} else {
		return false;
	}
}

bool Xml::delNode(std::string &xmlString, const std::string &strNodeName)
{
	TiXmlDocument *pDoc = new TiXmlDocument();
	if (NULL == pDoc) {
		return false;
	}
	pDoc->Parse(xmlString.c_str());
	TiXmlElement *pRootEle = pDoc->RootElement();
	if (NULL == pRootEle) {
		return false;
	}
	TiXmlElement *pNode = NULL;
	getNodePointerByName(pRootEle, strNodeName, pNode);
	if (pRootEle == pNode) {
	    if (pDoc->RemoveChild(pRootEle)) {
	        TiXmlPrinter printer;
			pDoc->Accept(&printer);
			xmlString = printer.CStr();
			pDoc->Clear();
			delete pDoc;
			pDoc = nullptr;
	  		return true;
	 	} else  {
	 		return false;
	 	}
	}

	if (NULL != pNode) {
		TiXmlNode *pParNode =  pNode->Parent();
		if (NULL == pParNode)	{
	        return false;
		}

		TiXmlElement* pParentEle = pParNode->ToElement();
		if (NULL != pParentEle) {
		    if (pParentEle->RemoveChild(pNode)) {
		      	TiXmlPrinter printer;
				pDoc->Accept(&printer);
				xmlString = printer.CStr();
		    } else {
				return false;
			}
		}
	} else {
	    return false;
	}

	return false;
}

bool Xml::modifyNodeText(std::string &xmlString, const std::string &strNodeName, const std::string &strText)
{
	if (xmlString.empty()) {
		return false;
	}

	TiXmlDocument *pDoc = new TiXmlDocument();
	if (NULL == pDoc) {
		return false;
	}
	pDoc->Parse(xmlString.c_str());
	TiXmlElement *pRootEle = pDoc->RootElement();
	if (NULL == pRootEle) {
		return false;
	}
	TiXmlElement *pNode = NULL;
	getNodePointerByName(pRootEle, strNodeName, pNode);
	if (NULL != pNode) {
		pNode->Clear();
		TiXmlText *pValue = new TiXmlText(strText.c_str());
		pNode->LinkEndChild(pValue);

		TiXmlPrinter printer;
		pDoc->Accept(&printer);
		xmlString = printer.CStr();
		pDoc->Clear();
		delete pDoc;
		pDoc = nullptr;
		return true;
	} else {
		return false;
	}
}

bool Xml::queryNodeText(const std::string &xmlString, const std::string &strNodeName, std::string &strText)
{
	if (xmlString.empty()) {
		return false;
	}

	TiXmlDocument *pDoc = new TiXmlDocument();
	if (NULL == pDoc) {
		return false;
	}
	pDoc->Parse(xmlString.c_str());
	TiXmlElement *pRootEle = pDoc->RootElement();
	if (NULL == pRootEle) {
		return false;
	}
	TiXmlElement *pNode = NULL;
	getNodePointerByName(pRootEle, strNodeName, pNode);
	if (NULL != pNode) {
		if (NULL != pNode->GetText()) {
			strText = pNode->GetText();
		} else {
			strText = "";
		}
		pDoc->Clear();
		delete pDoc;
		pDoc = nullptr;
		return true;
	} else {
		return false;
	}
}

bool Xml::addNodeAttribute(std::string &xmlString, const std::string &strParNodeName,
						   const std::string &strNodeName,
						   std::map<std::string, std::string> &AttMap)
{
	if (xmlString.empty()) {
		return false;
	}

	TiXmlDocument *pDoc = new TiXmlDocument();
	if (NULL == pDoc) {
		return false;
	}
	pDoc->Parse(xmlString.c_str());
	TiXmlElement *pRootEle = pDoc->RootElement();
	if (NULL == pRootEle) {
		return false;
	}
	TiXmlElement *pNode = NULL;
	getNodePointerByName(pRootEle, strParNodeName, pNode);
	if (NULL != pNode) {
		TiXmlElement *pNewNode = new TiXmlElement(strNodeName.c_str());
		if (NULL == pNewNode) {
			return false;
		}
		std::map<std::string, std::string>::iterator iter;
		for (iter = AttMap.begin(); iter != AttMap.end(); iter++) {
			pNewNode->SetAttribute(iter->first.c_str(), iter->second.c_str());
		}
		pNode->InsertEndChild(*pNewNode);

		TiXmlPrinter printer;
		pDoc->Accept(&printer);
		xmlString = printer.CStr();
		pDoc->Clear();
		delete pDoc;
		pDoc = nullptr;
		return true;
	} else {
		return false;
	}
}

bool Xml::modifyNodeAttribute(std::string &xmlString, const std::string &strNodeName, 
							  std::map<std::string, std::string> &AttMap)
{
	if (xmlString.empty()) {
		return false;
	}

	TiXmlDocument *pDoc = new TiXmlDocument();
	if (NULL == pDoc) {
		return false;
	}
	pDoc->Parse(xmlString.c_str());
	TiXmlElement *pRootEle = pDoc->RootElement();
	if (NULL == pRootEle) {
		return false;
	}

	TiXmlElement *pNode = NULL;
	getNodePointerByName(pRootEle, strNodeName, pNode);
	if (NULL != pNode) {
		TiXmlAttribute* pAttr = NULL;
		std::string strAttName = "";
		std::string strAttValue = "";
		for (pAttr = pNode->FirstAttribute(); pAttr; pAttr = pAttr->Next()) {
			strAttName = pAttr->Name();
			std::map<std::string, std::string>::iterator iter;
			for (iter = AttMap.begin(); iter != AttMap.end(); iter++) {
				if (strAttName == iter->first) {
					pAttr->SetValue(iter->second.c_str());
				}
			}
		}

		TiXmlPrinter printer;
		pDoc->Accept(&printer);
		xmlString = printer.CStr();
		pDoc->Clear();
		delete pDoc;
		pDoc = nullptr;
		return true;
	} else {
		return false;
	}
}

bool Xml::queryNodeAttribute(const std::string &xmlString, const std::string &strNodeName, 
	               			 String_Map &attMap)
{
	if (xmlString.empty()) {
		return false;
	}

	TiXmlDocument *pDoc = new TiXmlDocument();
	if (NULL == pDoc) {
		return false;
	}
	pDoc->Parse(xmlString.c_str());
	TiXmlElement *pRootEle = pDoc->RootElement();
	if (NULL == pRootEle) {
		return false;
	}
	TiXmlElement *pNode = NULL;
	getNodePointerByName(pRootEle, strNodeName, pNode);
	if (NULL != pNode) {
		TiXmlAttribute* pAttr = NULL;
		for (pAttr = pNode->FirstAttribute(); pAttr; pAttr = pAttr->Next()) {
			std::string strAttName = pAttr->Name();
			std::string strAttValue = pAttr->Value();
			attMap.insert(String_Pair(strAttName, strAttValue));
		}
		pDoc->Clear();
		delete pDoc;
		pDoc = nullptr;
		return true;
	} else {
		return false;
	}
	return true;
}

bool Xml::queryAllChildNodeAttribute(const std::string &xmlString, const std::string &strParNodeName,
									 const std::string &strChildNodeName, std::vector<String_Map> &attVector)
{
	if (xmlString.empty()) {
		return false;
	}
	
	TiXmlDocument *pDoc = new TiXmlDocument();
	if (NULL == pDoc) {
		return false;
	}
	pDoc->Parse(xmlString.c_str());
	TiXmlElement *pRootEle = pDoc->RootElement();
	if (NULL == pRootEle) {
		return false;
	}

	TiXmlElement *pParNode = NULL;
	TiXmlElement *pNode = NULL;
	String_Map AttMap;
	TiXmlAttribute* pAttr = NULL;
	getNodePointerByName(pRootEle, strParNodeName, pParNode);
	if (NULL == pParNode) {
		return false;
	}

	TiXmlElement* pParEle = pParNode;
	for (pParEle = pParNode->FirstChildElement(); pParEle; pParEle = pParEle->NextSiblingElement()) {
		if (strcmp(pParEle->Value(), strChildNodeName.c_str()) != 0) {
			continue;
		}
		getNodePointerByName(pParEle, strChildNodeName, pNode);
		if (NULL == pNode) {
			break;
		}

		AttMap.clear();
		for (pAttr = pNode->FirstAttribute(); pAttr; pAttr = pAttr->Next()) {
			std::string strAttName = pAttr->Name();
			std::string strAttValue = pAttr->Value();
			AttMap.insert(String_Pair(strAttName, strAttValue));
		}
		attVector.push_back(AttMap);
	} 

	if (attVector.size() == 0) {
		return false;
	} 

	pDoc->Clear();
	delete pDoc;
	pDoc = nullptr;
	return true;
}

bool Xml::queryAllChildNodeText(const std::string &xmlString, const std::string &strParNodeName,
								std::vector<std::string> &childNodeText)
{
	if (xmlString.empty()) {
		return false;
	}
	
	TiXmlDocument *pDoc = new TiXmlDocument();
	if (NULL == pDoc) {
		return false;
	}
	pDoc->Parse(xmlString.c_str());
	TiXmlElement *pRootEle = pDoc->RootElement();
	if (NULL == pRootEle) {
		return false;
	}

	TiXmlElement *pParNode = NULL;
	TiXmlElement *pNode = NULL;
	String_Map AttMap;
	TiXmlAttribute* pAttr = NULL;
	getNodePointerByName(pRootEle, strParNodeName, pParNode);
	if (NULL == pParNode) {
		return false;
	}

	TiXmlElement* pChildEle = NULL;
	std::string strText = "";
	for (pChildEle = pParNode->FirstChildElement(); pChildEle; pChildEle = pChildEle->NextSiblingElement()) {
		if (NULL != pChildEle->GetText()) {
			strText = pChildEle->GetText();
			childNodeText.push_back(strText);
		} 
	} 

	if (childNodeText.size() == 0) {
		return false;
	} 

	pDoc->Clear();
	delete pDoc;
	pDoc = nullptr;
	return true;
}


bool Xml::isChildNodeInParNode(const std::string &xmlString, const std::string &strParNodeName,
                           const std::string &strChildNodeName)
{
	if (xmlString.empty()) {
		return false;
	}

	TiXmlDocument *pDoc = new TiXmlDocument();
	if (NULL == pDoc) {
		return false;
	}
	pDoc->Parse(xmlString.c_str());
	TiXmlElement *pRootEle = pDoc->RootElement();
	if (NULL == pRootEle) {
		return false;
	}

	TiXmlElement *pParNode = NULL;
	TiXmlElement *pNode = NULL;
	getNodePointerByName(pRootEle, strParNodeName, pParNode);
	if (NULL == pParNode) {
		return false;
	}

	TiXmlElement* pParEle = pParNode;
	for (pParEle = pParNode->FirstChildElement(); pParEle; pParEle = pParEle->NextSiblingElement()) {
		if (strcmp(pParEle->Value(), strChildNodeName.c_str()) == 0) {
			pDoc->Clear();
			delete pDoc;
			pDoc = nullptr;
			return true;
		}
	}

	return false;
}