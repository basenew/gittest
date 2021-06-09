#ifndef ____LCOALPACKAGES_SW_H__
#define ____LCOALPACKAGES_SW_H__

#ifdef __cplusplus
extern "C" {
#endif

/*polling period, ms*/
#define SW_POLL_SLICE   10  


typedef enum
{
    SW_SOFT_BUTTUN = 0,

    SW_NUM
} sw_idx_t;

typedef void (*sw_callback_t)(sw_idx_t idx, uint8_t status);


int sw_init(void);
void    sw_unit_disable(sw_idx_t idx);
void    sw_unit_enable(sw_idx_t idx,  uint32_t ms);
void    sw_unit_attach(sw_idx_t idx,  sw_callback_t _cb);
void    sw_unit_detach(sw_idx_t idx);
uint8_t sw_unit_read(sw_idx_t idx);

#ifdef __cplusplus
}
#endif

#endif


