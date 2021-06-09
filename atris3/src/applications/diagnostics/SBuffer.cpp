#include "SBuffer.h"
#include <string.h>
#include <iostream>

SBuffer::SBuffer(int nSize)
:m_pData(nullptr)
, m_nSize(0)
, m_nBufferSize(nSize)
{
    m_pData = new char[m_nBufferSize];
}

SBuffer::~SBuffer()
{
    if(nullptr != m_pData)
    {
        delete m_pData;
        m_pData = nullptr;
    }
}

char* SBuffer::GetData()
{
    return m_pData;
}

int SBuffer::GetSize()
{
    return m_nSize;
}

int SBuffer::GetBufferSize()
{
    return m_nBufferSize;
}

bool SBuffer::SetData(char* data, int length)
{
    if (data == nullptr)
    {
        std::cout << "SetData data == nullptr\n";
        return false;
    }

    if (m_nSize+length >= m_nBufferSize)
    {
        m_pData = (char *)realloc(m_pData, m_nSize+length+1);
        m_nBufferSize = m_nSize+length+1;
    }

    memcpy(m_pData+m_nSize, data, length);

    m_nSize += length;
    m_pData[m_nSize] = '\0';
    return true;
}