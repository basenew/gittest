#ifndef ____APP_LOG_H__
#define ____APP_LOG_H__

#ifdef __cplusplus
extern "C" {
#endif


int log_notify_msg(const char* buf, int len);
int log_init(void);
uint8_t log_is_init(void);

#ifdef __cplusplus
}
#endif

#endif

