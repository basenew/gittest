#ifndef _REMOTE_CTRL_H_
#define _REMOTE_CTRL_H_


#define REMOTE_REPORT_PERIOD          50  //50ms, 20HZ

#ifdef __cplusplus
extern "C" {
#endif
#define REMOTE_CHARNNEL1      1
#define REMOTE_CHARNNEL2      2
#define REMOTE_CHARNNEL3      3
#define REMOTE_CHARNNEL4      4
#define REMOTE_CHARNNEL5      5
#define REMOTE_CHARNNEL6      6
#define REMOTE_CHARNNEL7      7
#define REMOTE_CHARNNEL8      8
#define REMOTE_CHARNNEL9      9
#define REMOTE_CHARNNEL10     10
#define REMOTE_CHARNNEL11     11
#define REMOTE_CHARNNEL12     12
#define REMOTE_CHARNNEL13     13
#define REMOTE_CHARNNEL14     14
#define REMOTE_CHARNNEL15     15
#define REMOTE_CHARNNEL16     16
#define REMOTE_FLAG           17
int32_t remote_ctrl_init(void);		//遥控器初始化函数
//int32_t sbus_data_post(uint16_t* _channels, uint8_t _flag);
uint8_t remote_get_sbus_data(uint16_t* _channels);	//获取遥控器各通道当前值
uint8_t remote_is_effecting(void);	//检测遥控器是否起效
int getChannel(int ch);		//获取遥控器某通道数据
int getFlag(void);
#ifdef __cplusplus
}
#endif


#endif
