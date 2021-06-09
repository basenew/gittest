#ifndef __PLATFORM_CHASsIS_DEF_H__
#define __PLATFORM_CHASsIS_DEF_H__

/*-----------------------------------------------
 * 浼樺厛绾ф帶鍒舵ā鍧?
------------------------------------------------*/
//#define NUMBER_OF_PRIORITY	4

#define DF_CHASSIS_PRIORITY_TTS false

typedef enum control_sequence
{
    REMOTE = 0,
    WEB_CTRL = 1,
    SHTTPD = 2,
    PC_CTRL = 3,
    RECHARGE = 4,
    SELF_TEST = 5,
    GS = 6,
    TIME_OUT = 7,
}Control_Owner;
#define NUMBER_OF_PRIORITY	(TIME_OUT + 1)

#endif
