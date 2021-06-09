#ifndef __EEPROM_H__
#define __EEPROM_H__

#define SN_MAX_LEN  32
typedef struct EepromData_
{
    char sn[SN_MAX_LEN];//设备SN码
    unsigned int odo;//底盘运动总里程,单位厘米
    unsigned long long time; //单位为秒
    char camera_reg_id[SN_MAX_LEN];
    char chassis_type;
}EepromData;

enum AtrisCmd
{
	CMD_REQ_CFG = 100,
	CMD_RESP_CFG = 101,
	CMD_REQ_SAVE_CFG,
	CMD_RESP_SAVE_CFG
};

enum ChassisType
{
    CHASSIS_JC = 'j',
    CHASSIS_GX = 'g',
    CHASSIS_WHEEL  = 'w'
};

typedef struct AtrisPkg_
{
	int magic_code;//666
	int cmd;
	int len;
}AtrisPkg;

class Eeprom
{
    private:
        Eeprom() {}

        int fd;
        int _i2c_read(int fd, unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int bytes);
        int _i2c_write(int fd, unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int bytes);

    public:
        EepromData  atris_info;
        int init();
        int read(unsigned char *buf, int len);
        int write(unsigned char *buf, int len);
        int clear();
        void cfg_srv_proc();
        void gen_camera_id();

        static Eeprom* get_instance(){
            static Eeprom singleton;
            return &singleton;
        }
};

#endif
