#ifndef __APP_KEY_H__
#define __APP_KEY_H__


#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    KEY_VAL_INVALID = 0,
    KEY_VAL_SFOTKEY_IN,
    // KEY_VAL_SFOTKEY_OUT,
    KEY_VAL_SFOTKEY_SHORT,    // 必须松开才生效
    KEY_VAL_SFOTKEY_LONG,     // 必须松开才生效
    KEY_VAL_SFOTKEY_KEEPLONG, // 不须松开,持续长按一定时间

    KEY_VAL_MAX,
} key_val_t;

typedef struct
{
    key_val_t val;
    char *name;
} key_val_c_t;

typedef int32_t (*key_subscriber_t)(key_val_t);

void key_subscribe(key_subscriber_t _sub);
int32_t key_init(void);


#ifdef __cplusplus
}
#endif

#endif
