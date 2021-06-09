#ifndef __NEGOTIATES_H__
#define __NEGOTIATES_H__

class Negotiates
{
public:
    ~Negotiates() {}

    private:
        Negotiates();
        void create_udp_server();
        void udp_server_proc();

    public:
        static Negotiates* get_instance(){
            static Negotiates singleton;
            return &singleton;
        }

};

#endif