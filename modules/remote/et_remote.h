#pragma once

#include <stdint.h>
#include "usart.h"
#include "bsp_usart.h"

/* Rocker Micro Def */

/* Switch Micro Def */
#define RC_SW_UP ((uint16_t)1)   // 开关向上时的值
#define RC_SW_MID ((uint16_t)3)  // 开关中间时的值
#define RC_SW_DOWN ((uint16_t)2) // 开关向下时的值
// 三个判断开关状态的宏
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

/* Knob Micro Def */

typedef struct
{
    int16_t rocker_l_; // 左水平
    int16_t rocker_l1; // 左竖直
    int16_t rocker_r_; // 右水平
    int16_t rocker_r1; // 右竖直

    uint8_t switch_left_2 : 2;  // 左侧二档开关
    uint8_t switch_left_3 : 2;  // 左侧三档开关
    uint8_t switch_right_2 : 2; // 右侧二档开关
    uint8_t switch_right_3 : 2; // 右侧三档开关

    uint16_t knob_left;  // 左侧旋钮
    uint16_t knod_right; // 右侧旋钮
} ETRC_Ctrl_s;

ETRC_Ctrl_s *ETRemoteInit(UART_HandleTypeDef *huart);

uint8_t ETRemoteIsOnline();
