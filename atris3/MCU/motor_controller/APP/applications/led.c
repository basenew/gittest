
#include "board.h"
#include "qled.h"


/*run led*/
#define LED_RUN_PIN GET_PIN(A, 5)
int led_send_run(void)
{
    qled_add(LED_RUN_PIN, 1);
    qled_set_blink(LED_RUN_PIN, 1000, 1000);
    return 0;
}
INIT_APP_EXPORT(led_send_run);

int led_send_upgrade(void)
{
    qled_add(LED_RUN_PIN, 1);
    qled_set_blink(LED_RUN_PIN, 200, 200);
    return 0;
}


/*sos led*/
/*
#define LED_SOS_PIN GET_PIN(E, 15)
static int sos_send_times = 0;
static const u16 sos_datas[] = 
{
    200, 200, 200, 200, 200, 200,       //short 3 times
    600, 600, 600, 600, 600, 600,       //long 3 times
    200, 200, 200, 200, 200, 200 + 2000 //short 3 times and 2000ms interval
};

static void led_sos_cb(void)
{
    sos_send_times--;
    if (sos_send_times > 0)
    {
        qled_set_special(LED_SOS_PIN, sos_datas, sizeof(sos_datas)/sizeof(u16), led_sos_cb);
    }
    else
    {
        qled_remove(LED_SOS_PIN);
    }
}

int led_send_sos(void)
{
    sos_send_times = 1000;
    qled_add(LED_SOS_PIN, 1);
    qled_set_special(LED_SOS_PIN, sos_datas, sizeof(sos_datas)/sizeof(u16), led_sos_cb);
    return 0;
}
INIT_APP_EXPORT(led_send_sos);
*/



