#ifndef ____APP_MODULES_HWWDG_H__
#define ____APP_MODULES_HWWDG_H__

#ifdef __cplusplus
extern "C" {
#endif

void hw_wdg_feed(void);
int hw_wdg_init(void);
void hw_wgd_enable(rt_uint8_t _status);



#ifdef __cplusplus
}
#endif

#endif
