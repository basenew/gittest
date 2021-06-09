#ifndef ATRIS_DEVICE_VOIP_H_
#define ATRIS_DEVICE_VOIP_H_

#include <vector>
#include <string>

enum HandleType {
    HANDLEVOIPTYPE_GET,
    HANDLEVOIPTYPE_SET
};

class Voip {
public:
    ~Voip();
    static Voip* get_instance()
    {
        static Voip singleton;
        return &singleton;
    }

    int CancelCall();
    int GetVolume(int *volume);
    int SetVolume(int volume);
    int SetSipNum(const std::string &num);
    void SetCallingStatus(bool calling) {calling_now_ = calling;}
    bool IsCallingNow() {return calling_now_;}

private:
    Voip();
    void InitSetting();
    int handleVoipAttributes(HandleType type, const std::vector<std::string> &AttrText, 
                             std::string &httpResp);
    bool calling_now_;

};

#endif