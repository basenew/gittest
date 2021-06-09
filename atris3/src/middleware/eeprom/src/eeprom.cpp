#include "eeprom.h"
#include <stdio.h>
#include <stdlib.h>
#include <linux/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "log/log.h"

#include <boost/thread/thread.hpp>

#define MAX_I2C_MSG 2
#define I2C_RETRIES 0x701
#define I2C_TIMEOUT 0x702
#define I2C_RDWR 0x707
#define I2C_M_RD 0x1
#define SN_LEN 17
#define CHIP_ADDR_1 0x54 //sector 1  2K
#define CHIP_ADDR_2 0x55 //sector 2  2k

#define EEPROM_I2C_DEV  "/dev/i2c-1"
#define SN_LEN  17

#define CFG_PORT    6789
#define MAGIC_CODE  4792

struct i2c_msg {
    __u16    addr;
    __u16    flags;
    __u16    len;
    __u8 *buf;
};

struct i2c_rdwr_ioctl_data{
    struct i2c_msg *msgs;
    int nmsgs;
};

int Eeprom::_i2c_write(int fd, unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo)
{
    int ret;
    unsigned char buftmp[32];
    struct i2c_rdwr_ioctl_data i2c_data;

    i2c_data.nmsgs = 1;
    i2c_data.msgs = (struct i2c_msg *)malloc(i2c_data.nmsgs *sizeof(struct i2c_msg));
    if (i2c_data.msgs == NULL){
        log_error("malloc error");
        close(fd);
        return -1;
    }

    //ioctl(fd, I2C_TIMEOUT, 1);
    //ioctl(fd, I2C_RETRIES, 2);

    memset(buftmp, 0, 32);
    buftmp[0] = sub_addr;
    memcpy(buftmp + 1, buff, ByteNo);
    i2c_data.msgs[0].len = ByteNo + 1;
    i2c_data.msgs[0].addr = device_addr;
    i2c_data.msgs[0].flags = 0;     // 0: write 1:read
    i2c_data.msgs[0].buf = buftmp;
    ret = ioctl(fd, I2C_RDWR, (unsigned long)&i2c_data);
    if (ret < 0) {
        log_error("write reg %x %x error\r\n", device_addr, sub_addr);
        close(fd);
        free(i2c_data.msgs);
        return -1;
    }
    close(fd);
    free(i2c_data.msgs);
#ifdef DEBUG
    int i;
    printf("i2c_write 0x%02x:",buftmp[0]);
    for(i=0; i<ByteNo; i++){
        printf(" 0x%02x",buftmp[1+i]);
    }
    printf("\n");
#endif
    usleep(100000);
    return 0;
}

int Eeprom::_i2c_read(int fd, unsigned char device_addr, unsigned char sub_addr, unsigned char *buff, int ByteNo)
{
    int ret;
    unsigned char buftmp[32];
    struct i2c_rdwr_ioctl_data i2c_data;

    i2c_data.nmsgs = 2;
    i2c_data.msgs = (struct i2c_msg *)malloc(i2c_data.nmsgs *sizeof(struct i2c_msg));
    if (i2c_data.msgs == NULL){
        log_error("malloc error");
        close(fd);
        return -1;
    }

    //ioctl(fd, I2C_TIMEOUT, 1);
    //ioctl(fd, I2C_RETRIES, 2);

    memset(buftmp, 0, 32);
    buftmp[0] = sub_addr;
    i2c_data.msgs[0].len = 1;
    i2c_data.msgs[0].addr = device_addr;
    i2c_data.msgs[0].flags = 0;     // 0: write 1:read
    i2c_data.msgs[0].buf = buftmp;

    i2c_data.msgs[1].len = ByteNo;
    i2c_data.msgs[1].addr = device_addr;
    i2c_data.msgs[1].flags = 1;     // 0: write 1:read
    i2c_data.msgs[1].buf = buff;

    ret = ioctl(fd, I2C_RDWR, (unsigned long)&i2c_data);
    if (ret < 0){
        log_error("read data %x %x error\r\n", device_addr, sub_addr);
        close(fd);
        free(i2c_data.msgs);
        return -1;
    }
    close(fd);
    free(i2c_data.msgs);
#ifdef DEBUG
    int i;
    printf("i2c__read 0x%02x:",buftmp[0]);
    for (i = 0; i < ByteNo; i++){
        printf(" 0x%02x",buff[i]);
    }
    printf("\n");
#endif

    return 0;
}

int Eeprom::init()
{
    memset(&atris_info, 0, sizeof(atris_info));

    fd = open(EEPROM_I2C_DEV, O_RDWR);
    if(fd < 0){
        log_error("open eeprom device fail.");
        return -1;
    }

    int ret = read((unsigned char*)&atris_info, sizeof(atris_info));
    if(ret < 0){
        log_error("init atris_info fail.");
        return -1;
    }
    
    atris_info.sn[SN_LEN] = '\0';
    if (strstr(atris_info.sn, "UBT") == NULL) {
        log_error("Eeprom::init(): Invalid SN(%s)", atris_info.sn);
        atris_info.sn[0] =  '\0';
    }

    gen_camera_id();

    if(atris_info.chassis_type == CHASSIS_GX){
        log_info("this atris is based on guoxing chassis!");
    }
    else if(atris_info.chassis_type == CHASSIS_JC){
        log_info("this atris is based on jc chassis!");
    }
    else if(atris_info.chassis_type == CHASSIS_WHEEL){
        log_info("this atris is based on wheel chassis!");
    }
    else{
        log_error("can't find fit chassis,set default jc chassis!");
        atris_info.chassis_type = CHASSIS_JC;
    }

    boost::thread cfg_server_thread(boost::bind(&Eeprom::cfg_srv_proc, this));
    log_warn("atris parameters:\nsn:%s\nchassis type:%d\nodo:%d cm\trun_time:%lld seconds\ncamera_id:%s\n",
            atris_info.sn, atris_info.chassis_type, atris_info.odo, atris_info.time, atris_info.camera_reg_id);

    return 0;
}

/**
 * @brief gen_camera_id 生成camera ID
 */
void Eeprom::gen_camera_id()
{
    if (strstr(atris_info.sn, "UBT") != NULL) {
        memset(atris_info.camera_reg_id, '\0', SN_MAX_LEN);
        memcpy(atris_info.camera_reg_id, atris_info.sn, SN_LEN);
    } else {
        atris_info.camera_reg_id[0] = '\0';
    }
}

int Eeprom::read(unsigned char *buf, int len)
{
    if(fd < 0){
        log_error("invaild fd");
        return -1;
    }

    int ret = _i2c_read(fd, CHIP_ADDR_1, 0, buf, len);
    if(ret < 0){
        log_error("read eeprom data error");
        return ret;
    }

    return 0;
}

int Eeprom::write(unsigned char *buf, int len)
{
    int ret = 0;

    if(fd < 0){
        log_error("invaild fd");
        return -1;
    }

    int i = 0;
    for(i = 0; i < len/8; i ++){
        ret = _i2c_write(fd, CHIP_ADDR_1, i*8, &buf[i*8], 8);
        if(ret < 0){
            log_error("write eeprom fail.");
            return ret;
        }
    }

    ret = _i2c_write(fd, CHIP_ADDR_1, i*8, &buf[i*8], len%8);
    if(ret < 0){
        log_error("write eeprom fail.");
        return ret;
    }

    return 0;
}

/**
 * @brief clear 清除存储在eeprom中的车信息
 *
 * @return
 */
int Eeprom::clear()
{
    memset(&atris_info, 0, sizeof(atris_info));
    return write((unsigned char*)&atris_info, sizeof(atris_info));
}

void Eeprom::cfg_srv_proc()
{
    int sockfd = -1;

    struct sockaddr_in server;
    bzero(&server,sizeof(server));
    server.sin_family=AF_INET;
    server.sin_addr.s_addr=htonl(INADDR_ANY);
    server.sin_port=htons(CFG_PORT);

    if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0){
        log_error("create udp socket fail.");
        return ;
    }

    if((bind(sockfd, (struct sockaddr *)&server, sizeof(server))) < 0){
        log_error("bind error.");
        close(sockfd);
        return ;
    }

    unsigned int l = sizeof(struct sockaddr_in);
    struct sockaddr_in client;
    unsigned char data[1024];

    log_warn("init cfg server ok.");
    int p_len = sizeof(AtrisPkg);
    int cfg_size = sizeof(atris_info);
    while(1){
        memset(data, 0, sizeof(data));

        int ret = ::recvfrom(sockfd, data, sizeof(data), 0, (struct sockaddr *)&client, &l);
        if(ret < 0){
            continue;
        }

        AtrisPkg *pkg = (AtrisPkg*)data;
        if(pkg->cmd == CMD_REQ_CFG && pkg->magic_code == MAGIC_CODE){
            memset(data, 0, sizeof(data));
            AtrisPkg p;
            p.magic_code = MAGIC_CODE;
            p.cmd = CMD_RESP_CFG;
            p.len = sizeof(atris_info);
            memcpy(data, &p, p_len);
            memcpy(data+p_len, &atris_info, cfg_size);
            if(sendto(sockfd, data, cfg_size+p_len, 0, (struct sockaddr*)&client, l)<0){
                log_error("send to client fail.");
            }
            log_info("sn:%s type:%c, camera id:%s", atris_info.sn, atris_info.chassis_type,
                    atris_info.camera_reg_id);
        }
        else if(pkg->cmd == CMD_REQ_SAVE_CFG && pkg->magic_code == MAGIC_CODE){
            //backup config
            EepromData cfg_backup;
            memcpy(&cfg_backup, &atris_info, sizeof(cfg_backup));

            EepromData *cfg = (EepromData*)(data+p_len);
            log_info("recv:sn:%s type:%d", cfg->sn, cfg->chassis_type);
            memcpy(atris_info.sn, cfg->sn, sizeof(cfg->sn));
            atris_info.chassis_type = cfg->chassis_type;
            int ret = write((unsigned char*)&atris_info, sizeof(atris_info));
            if(ret == 0){
                AtrisPkg p;
                gen_camera_id();
                p.magic_code = MAGIC_CODE;
                p.cmd = CMD_RESP_SAVE_CFG;
                p.len = sizeof(atris_info);
                if(sendto(sockfd, &p, p_len, 0, (struct sockaddr*)&client, l)<0){
                    log_error("send to client fail.");
                }
                log_info("success save config data.:%s, type:%c",
                        atris_info.sn, atris_info.chassis_type);
            }
            else{
                //when save fail, restore data before
                log_info("save data fail.");
                memcpy(&atris_info, &cfg_backup, sizeof(cfg_backup));
            }
        }
    }
    
    close(sockfd);
}
