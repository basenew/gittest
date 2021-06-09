#ifndef __APP_CAN_BXCAN_FILTER_H__
#define __APP_CAN_BXCAN_FILTER_H__

void CANFilterConfig_Scale32_IdList(CAN_HandleTypeDef *hcan);
void CANFilterConfig_Scale16_IdList(CAN_HandleTypeDef *hcan);
void CANFilterConfig_Scale32_IdMask_StandardIdOnly(CAN_HandleTypeDef *hcan);
void CANFilterConfig_Scale32_IdMask_ExtendIdOnly(CAN_HandleTypeDef *hcan);

#endif
