#ifndef __CAN_PKG_H__
#define __CAN_PKG_H__

#include "log/log.h"

#define CAN_TIMEOUT 2000

#pragma pack(1)
typedef struct
{
    union HEAD{
        struct ID{
            unsigned int channel:16;
            unsigned int index:8;
            unsigned int end:1;
            unsigned int can:1;
            unsigned int mode:3;
            unsigned int size:3;
        }id;
        unsigned int h32;
    }head;
    unsigned char data[8];
}CanPkg;
#pragma pack()

inline void dump_can_pkg(CanPkg* pkg)
{
    log_info("head:%x bus=%02x channel=%02x data:%02x %02x %02x %02x %02x %02x %02x %02x", pkg->head.h32,pkg->head.id.can, pkg->head.id.channel,
            pkg->data[0], pkg->data[1], pkg->data[2], pkg->data[3], pkg->data[4], pkg->data[5], pkg->data[6], pkg->data[7]);
}

enum CanChannel
{
    CH_LAMP = 0x80,
    CH_POWER = 0x81,
    CH_ULTRASOUND_1 = 0x90,
    CH_ULTRASOUND_2 = 0x91,

    CH_BATTERY_0 = 0xA0,
    CH_BATTERY_1 = 0xA1,
    CH_BATTERY_2 = 0xA2,
    CH_PM_CTRL = 0xB0,

    CH_PM_MONITOR = 0x82,

    CH_CHARGER_LIMIT_SWITCH = 0x89,
    CH_CHARGER_BT = 0x8A,
    CH_CHARGER_CHARGE = 0x8B,
    CH_CHARGER_CHARGE_FINISH = 0x8C,
    CH_CHARGER_POWER_ON = 0x8D,

    CH_BATTERY_TEST_0 = 0xF0,
    CH_BATTERY_TEST_1 = 0xF1,
    CH_BATTERY_TEST_2 = 0xF2,
    CH_BATTERY_TEST_3 = 0XF3,

    CH_SHAKE_DIE = 0XF4
};

#define CAN_CMD_BRAKE 0x37

#endif
