#ifndef _S_Buffer_H_
#define _S_Buffer_H_

class SBuffer
{
public:
    SBuffer(int nSize);
    ~SBuffer();

    bool SetData(char* data, int length);
    char* GetData();
    int GetSize();
    int GetBufferSize();
private:
    char*           m_pData;
    int             m_nSize;
    int             m_nBufferSize;
};

#endif