#ifndef __APP_MODULES_BRAKE_H__
#define __APP_MODULES_BRAKE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "board.h"


enum
{
    BRAKE_STATUS_UNLOCK  = 0,       //ɲ������
    BRAKE_STATUS_LOCK,              //ɲ������
};

enum
{
    BRAKE_CAUSE_REBOOT = 0,         //�����Զ�ɲ��
	BRAKE_CAUSE_E_STOP,             //��ͣɲ��
    BRAKE_CAUSE_ANTI,               //��ײ��ɲ��
    BRAKE_CAUSE_CHARGEGUN,          //ֱ��ɲ��
    BRAKE_CAUSE_HOST,               //���ض�ɲ������ɲ��
	  BRAKE_CAUSE_CLIFF,
    //���8��ԭ��
};


typedef struct
{
    int8_t     status;    // ɲ��״̬
    uint8_t    cause;     // ɲ��ԭ��
    rt_mutex_t lock;
} brake_t;
	
	
void brake_get_status(brake_t* _brake);            //��ȡ��ǰɲ����Ϣ
void brake_set(uint8_t cause, uint8_t _status);    //ɲ������
void brake_set_byhost(uint8_t status);             //��������ɲ��״̬
void anti_brake_process(void);                     //��ײ��ɲ��������
void brake_unlock_byestop(void);                   //�����ͣɲ��
uint8_t get_brake_status(void);                    //��ȡ��ǰɲ��״̬
uint8_t get_brake_cause(void);                     //��ȡ��ǰɲ��ԭ��

void chassis_transport_mode_set(uint8_t _staus);   //����ģʽ����			����ģʽ����������ʱ���ܽ����ӵ���������״̬��
uint8_t chassis_transport_mode_get(void);          //��ȡ����ģʽ״̬

#ifdef __cplusplus
}
#endif


#endif



