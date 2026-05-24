#include "et_remote.h"
#include "daemon.h"
#include <string.h>

#include "user_lib.h"

#define REMOTE_FRAME_SIZE 25u // 遥控器接收的buffer大小

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((int16_t)353)
#define RC_CH_VALUE_OFFSET ((int16_t)1024)
#define RC_CH_VALUE_MAX ((int16_t)1694)

#define ET_CHANNEL(num) ((num) - 1)
// #define ET_SWITCHSELECT(num) ((num > RC_CH_VALUE_OFFSET) ? RC_SW_DOWN : ((num < RC_CH_VALUE_OFFSET) ? RC_SW_UP : RC_SW_MID))
#define ET_SWITCHSELECT(num) ((num == RC_CH_VALUE_OFFSET) ? RC_SW_MID  \
                              : (num == RC_CH_VALUE_MAX)  ? RC_SW_DOWN \
                              : (num == RC_CH_VALUE_MIN)  ? RC_SW_UP   \
                                                          : 0)

__section(".ccmram") static ETRC_Ctrl_s rc_ctrl;
static USARTInstance *instance = NULL;
static DaemonInstance *daemon = NULL;

static inline void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
    {
        if (*(&rc_ctrl.rocker_l_ + i) > RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET ||
            *(&rc_ctrl.rocker_l_ + i) < RC_CH_VALUE_MIN - RC_CH_VALUE_OFFSET)
            *(&rc_ctrl.rocker_l_ + i) = 0;
    }
}

static inline void DecodeRemote(const int16_t sbus_buf[8])
{
    rc_ctrl.rocker_l1 = sbus_buf[ET_CHANNEL(3)] - RC_CH_VALUE_OFFSET; // 升降舵(反)
    rc_ctrl.rocker_l_ = sbus_buf[ET_CHANNEL(4)] - RC_CH_VALUE_OFFSET; // 方向舵(反)
    rc_ctrl.rocker_r1 = sbus_buf[ET_CHANNEL(2)] - RC_CH_VALUE_OFFSET; // 油门(反)
    rc_ctrl.rocker_r_ = sbus_buf[ET_CHANNEL(1)] - RC_CH_VALUE_OFFSET; // 副翼(反)
    RectifyRCjoystick();

    rc_ctrl.switch_right_2 = ET_SWITCHSELECT(sbus_buf[ET_CHANNEL(5)]); // SD(正)
    rc_ctrl.switch_right_3 = ET_SWITCHSELECT(sbus_buf[ET_CHANNEL(6)]); // SC(辅助1)(正)
    rc_ctrl.switch_left_3 = ET_SWITCHSELECT(sbus_buf[ET_CHANNEL(7)]);  // SB(辅助2)(正)
    rc_ctrl.switch_left_2 = ET_SWITCHSELECT(sbus_buf[ET_CHANNEL(8)]);  // SA(辅助3)(正)
}

static void RCLostCallback(void *id)
{
    // @todo 遥控器丢失的处理
    memset(&rc_ctrl, 0, sizeof(ETRC_Ctrl_s));
    USARTServiceInit(id); // 尝试重新启动接收
}

static void RemoteControlRxCallback(USARTInstance *bind_data)
{
    const uint8_t * const buff = instance->recv_buff;
    if (buff[0] != 0x0F)
        return;
        
    DaemonReload(daemon); // 先喂狗

    // 目前仅保留前八个通道
    int16_t channels[] = {
        [0] = (((int16_t)buff[1]) >> 0 | ((int16_t)buff[2]) << 8) & 0x07FF,
        [1] = (((int16_t)buff[2]) >> 3 | ((int16_t)buff[3]) << 5) & 0x07FF,
        [2] = ((int16_t)buff[3] >> 6 | ((int16_t)buff[4] << 2) | (int16_t)buff[5] << 10) & 0x07FF,
        [3] = ((int16_t)buff[5] >> 1 | ((int16_t)buff[6] << 7)) & 0x07FF,
        [4] = ((int16_t)buff[6] >> 4 | ((int16_t)buff[7] << 4)) & 0x07FF,
        [5] = ((int16_t)buff[7] >> 7 | ((int16_t)buff[8] << 1) | (int16_t)buff[9] << 9) & 0x07FF,
        [6] = ((int16_t)buff[9] >> 2 | ((int16_t)buff[10] << 6)) & 0x07FF,
        [7] = ((int16_t)buff[10] >> 5 | ((int16_t)buff[11] << 3)) & 0x07FF,
        // [8] = ((int16_t)buff[12] << 0 | ((int16_t)buff[13] << 8)) & 0x07FF,
        // [9] = ((int16_t)buff[13] >> 3 | ((int16_t)buff[14] << 5)) & 0x07FF,
        // [10] = ((int16_t)buff[14] >> 6 | ((int16_t)buff[15] << 2) | (int16_t)buff[16] << 10) & 0x07FF,
        // [11] = ((int16_t)buff[16] >> 1 | ((int16_t)buff[17] << 7)) & 0x07FF,
        // [12] = ((int16_t)buff[17] >> 4 | ((int16_t)buff[18] << 4)) & 0x07FF,
        // [13] = ((int16_t)buff[18] >> 7 | ((int16_t)buff[19] << 1) | (int16_t)buff[20] << 9) & 0x07FF,
        // [14] = ((int16_t)buff[20] >> 2 | ((int16_t)buff[21] << 6)) & 0x07FF,
        // [15] = ((int16_t)buff[21] >> 5 | ((int16_t)buff[22] << 3)) & 0x07FF,
    };

    DecodeRemote(channels); // 进行协议解析
}

ETRC_Ctrl_s *ETRemoteInit(UART_HandleTypeDef *huart)
{
    if (instance != NULL)
        while (1) // 目前仅支持一个遥控进行控制
            ;

    memset(&rc_ctrl, 0, sizeof(ETRC_Ctrl_s));

    instance = USARTRegister(&(USART_Init_Config_s){
        .module_callback = RemoteControlRxCallback,
        .recv_buff_size = REMOTE_FRAME_SIZE,
        .usart_handle = huart,
    });

    daemon = DaemonRegister(&(Daemon_Init_Config_s){
        .callback = RCLostCallback,
        .owner_id = instance,
        .reload_count = 10,
    });

    return &rc_ctrl;
}

uint8_t ETRemoteIsOnline()
{
    if (!instance)
        return 0; // Return 0 if instance is NULL

    if (DaemonIsOnline(daemon) &&
        (switch_is_up(rc_ctrl.switch_left_2) || switch_is_down(rc_ctrl.switch_left_2))) // 二档开关，非高即低且不为0
    {
        return 1;
    }

    return 0;
}
