/*
 * COPYRIGHT (C) Copyright 2019-2029; UBT TECH; SHENZHEN, CHINA
 *
 * File       : bxcan_filter.c
 * Brief      : bxcan 滤波器配置

 * Change Logs
 * Date           Author        Version       Notes
 * 2019-11-06     wuxiaofeng    v1.0          first version
 * note:
 * 1.若CAN ID数量少，使用列表模式，过滤比较精准
 * 2.若只有标准ID，可以使用16位宽模式
 * 3.若CAN ID数量多，可以使用多个滤波器，不通的模式组合使用。ID 值相近的可以归纳成一组，使用掩码模式进行过滤
 * 4.掩码模式时，有可能部分不期望的ID也会通过过滤器，掩码放得越宽，带进其他ID的几率越大
 * 5.对于相近的ID，可以提前计算好屏蔽码，直接在代码中填入，而不是在代码中临时计算，可以提高软件效率。
 */
#include "drv_common.h"
#include "stm32f4xx.h"
#include "can_cmd.h"

/*
32 位宽列表模式
CAN_FxR1 和 CAN_FxR2 既可以存储标准ID 也可以存储扩展ID
一个滤波器最多可以过滤 2 个ID
*/
void CANFilterConfig_Scale32_IdList(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;

    /*标准 CAN ID*/
    uint32_t StdId = 0x321;
    /*扩展 CAN ID*/
    uint32_t ExtId = 0x1800f001;

    /*使用过滤器 0*/
    sFilterConfig.FilterBank = 0;
    /*列表模式*/
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    /*32 位宽*/
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    /*设置标准 ID*/
    sFilterConfig.FilterIdHigh = StdId << 5;
    /*设置 IDE 位为 0*/
    sFilterConfig.FilterIdLow = 0 | CAN_ID_STD;
    /*设置扩展 ID*/
    sFilterConfig.FilterMaskIdHigh = ((ExtId << 3) >> 16) & 0xffff;
    /*设置 IDE 位为 1*/
    sFilterConfig.FilterMaskIdLow = ((ExtId << 3) & 0xffff) | CAN_ID_EXT;
    /*接收到的报文放入到 FIFO0 中*/
    sFilterConfig.FilterFIFOAssignment = 0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
16 位宽的列表模式
CAN_FxR1 和 CAN_FxR2 分别可以存储 2 个标准ID
一个滤波器最多可以过滤 4 个标准ID
*/
void CANFilterConfig_Scale16_IdList(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;

    uint32_t StdId1 = 0x00;
    uint32_t StdId2 = NODE_ID;
    uint32_t StdId3 = 0x00;
    uint32_t StdId4 = 0x00;

    /*使用过滤器 1*/
    sFilterConfig.FilterBank = 0;
    /*列表模式*/
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
    /*16 位宽*/
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
    //4 个标准 ID 分别放入到两个寄存器的高低字节中
    sFilterConfig.FilterIdHigh = StdId1 << 5;
    sFilterConfig.FilterIdLow = StdId2 << 5;
    sFilterConfig.FilterMaskIdHigh = StdId3 << 5;
    sFilterConfig.FilterMaskIdLow = StdId4 << 5;
    /*接收到的报文放入到 FIFO0 中*/
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
32 位宽的掩码模式
CAN_FxR1 存储验证ID，CAN_FxR2 存储掩码
一个滤波器可以过滤多个标准ID或扩展ID
任意一个期望通过的 CAN ID 都是可以设为验证码，屏蔽码是所有期望通过的 CAN ID
相互同或后的结果
*/

/*只过滤标准ID*/
void CANFilterConfig_Scale32_IdMask_StandardIdOnly(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;
    uint16_t mask, num, tmp, i;

    uint16_t StdIdArray[10] = {0x7e0, 0x7e1, 0x7e2, 0x7e3, 0x7e4, 0x7e5, 0x7e6, 0x7e7, 0x7e8, 0x7e9};

    /*使用过滤器 2*/
    sFilterConfig.FilterBank = 2;
    /*掩码模式 32位宽*/
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    /*验证码可以设置为 StdIdArray[]数组中任意一个*/
    sFilterConfig.FilterIdHigh = (StdIdArray[0] << 5);
    sFilterConfig.FilterIdLow = 0;

    /*计算屏蔽码*/
    mask = 0x7ff;
    num = sizeof(StdIdArray) / sizeof(StdIdArray[0]);
    /*屏蔽码位 StdIdArray[]数组中所有成员的同或结果*/
    for (i = 0; i < num; i++)
    { /*所有数组成员与第 0 个成员进行同或操作*/
        tmp = StdIdArray[i] ^ (~StdIdArray[0]);
        mask &= tmp;
    }
    sFilterConfig.FilterMaskIdHigh = (mask << 5);
    /*只接收数据帧*/
    sFilterConfig.FilterMaskIdLow = 0 | 0x02; // 0|0x02
    /*接收到的报文放入到 FIFO0 中*/
    sFilterConfig.FilterFIFOAssignment = 0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/*只过滤扩展ID*/
void CANFilterConfig_Scale32_IdMask_ExtendIdOnly(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;

    uint32_t ExtIdArray[10] = {0x1839f101, 0x1835f102, 0x1835f113, 0x1835f124, 0x1835f105};
    //uint32_t ExtIdArray[] = {0x1830};
    uint32_t mask, num, tmp, i;

    sFilterConfig.FilterBank = 3;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    sFilterConfig.FilterIdHigh = ((ExtIdArray[0] << 3) >> 16) & 0xffff;
    sFilterConfig.FilterIdLow = ((ExtIdArray[0] << 3) & 0xffff) | CAN_ID_EXT;

    /*计算屏蔽码*/
    mask = 0x1fffffff;
    num = sizeof(ExtIdArray) / sizeof(ExtIdArray[0]);
    /*屏蔽码位是数组中所有成员的同或结果*/
    for (i = 0; i < num; i++)
    {
        tmp = ExtIdArray[i] ^ (~ExtIdArray[0]);
        mask &= tmp;
    }
    mask <<= 3; /*对齐寄存器*/
    sFilterConfig.FilterMaskIdHigh = (mask >> 16) & 0xffff;
    sFilterConfig.FilterMaskIdLow = (mask & 0xffff) | 0x02; /*只接收数据帧*/ //
    sFilterConfig.FilterFIFOAssignment = 0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/*标准 ID 与扩展ID 混合过滤 */
void CANFilterConfig_Scale32_IdMask_StandardId_ExtendId_Mix(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;
    uint32_t mask, num, tmp, i, standard_mask, extend_mask, mix_mask;

    uint32_t StdIdArray[10] = {0x711, 0x712, 0x713, 0x714, 0x715, 0x716, 0x717, 0x718, 0x719, 0x71a};
    uint32_t ExtIdArray[10] = {0x1900fAB1, 0x1900fAB2, 0x1900fAB3, 0x1900fAB4, 0x1900fAB5};

    sFilterConfig.FilterBank = 4;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

    /*使用第一个扩展 CAN ID 作为验证码*/
    sFilterConfig.FilterIdHigh = ((ExtIdArray[0] << 3) >> 16) & 0xffff;
    sFilterConfig.FilterIdLow = ((ExtIdArray[0] << 3) & 0xffff);

    /*计算屏蔽码*/
    /*首先计算出所有标准 CAN ID 的屏蔽码*/
    standard_mask = 0x7ff;
    num = sizeof(StdIdArray) / sizeof(StdIdArray[0]);
    for (i = 0; i < num; i++)
    {
        tmp = StdIdArray[i] ^ (~StdIdArray[0]);
        standard_mask &= tmp;
    }
    /*接着计算出所有扩展 CAN ID 的屏蔽码*/
    extend_mask = 0x1fffffff;
    num = sizeof(ExtIdArray) / sizeof(ExtIdArray[0]);
    for (i = 0; i < num; i++)
    {
        tmp = ExtIdArray[i] ^ (~ExtIdArray[0]);
        extend_mask &= tmp;
    }
    /*再计算标准 CAN ID 与扩展 CAN ID 混合的屏蔽码*/
    mix_mask = (StdIdArray[0] << 18) ^ (~ExtIdArray[0]);
    /*最后计算最终的屏蔽码*/
    mask = (standard_mask << 18) & extend_mask & mix_mask;
    /*对齐寄存器*/
    mask <<= 3;
    sFilterConfig.FilterMaskIdHigh = (mask >> 16) & 0xffff;
    sFilterConfig.FilterMaskIdLow = (mask & 0xffff);
    sFilterConfig.FilterFIFOAssignment = 0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
16 位宽掩码模式
CAN_FxR1 的低16位是为验证码,对应屏蔽码为 CAN_FxR2 低16位
CAN_FxR1 的高16位是为验证码,对应屏蔽码为 CAN_FxR2 高16位
一个滤波器有两对验证码与屏蔽码组合，都只能对标准 CAN ID 进行过滤
*/

void CANFilterConfig_Scale16_IdMask(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;
    uint16_t mask, tmp, i, num;
    /*第一组标准 CAN ID*/
    uint16_t StdIdArray1[10] = {0x7D1, 0x7D2, 0x7D3, 0x7D4, 0x7D5, 0x7D6, 0x7D7, 0x7D8, 0x7D9, 0x7DA};
    /*第二组标准 CAN ID*/
    uint16_t StdIdArray2[10] = {0x751, 0x752, 0x753, 0x754, 0x755, 0x756, 0x757, 0x758, 0x759, 0x75A};

    sFilterConfig.FilterBank = 5;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

    /*配置第一个过滤对*/
    /*设置第一个验证码*/
    sFilterConfig.FilterIdLow = StdIdArray1[0] << 5;
    /*计算第一个屏蔽码*/
    mask = 0x7ff;
    num = sizeof(StdIdArray1) / sizeof(StdIdArray1[0]);
    for (i = 0; i < num; i++)
    {
        tmp = StdIdArray1[i] ^ (~StdIdArray1[0]);
        mask &= tmp;
    }
    sFilterConfig.FilterMaskIdLow = (mask << 5) | 0x10; /*只接收数据帧*/

    /*配置第二个过滤对*/
    sFilterConfig.FilterIdHigh = StdIdArray2[0] << 5;
    mask = 0x7ff;
    num = sizeof(StdIdArray2) / sizeof(StdIdArray2[0]);
    for (i = 0; i < num; i++)
    {
        tmp = StdIdArray2[i] ^ (~StdIdArray2[0]);
        mask &= tmp;
    }
    sFilterConfig.FilterMaskIdHigh = (mask << 5) | 0x10;

    sFilterConfig.FilterFIFOAssignment = 0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}
