/*
 * @Descripttion:
 * @version:
 * @Author: Chenfu
 * @Date: 2022-12-02 21:32:47
 * @LastEditTime: 2022-12-05 15:29:49
 */
#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"

__section(".ccmram") static SuperCapInstance super_cap_instance; // 可以由app保存此指针
static uint8_t cap_error_code = 0;

static void SuperCapRxCallback(CANInstance *_instance)
{
    memcpy(&super_cap_instance.cap_msg, _instance->rx_buff, sizeof(SuperCap_Msg_s));

    cap_error_code |= super_cap_instance.cap_msg.errorCode;
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    memset(&super_cap_instance, 0, sizeof(SuperCapInstance));
    
    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance.can_ins = CANRegister(&supercap_config->can_config);
    return &super_cap_instance;
}

void SuperCapSend(SuperCapInstance *instance, SuperCap_Ctrl_s *data)
{
    memcpy(instance->can_ins->tx_buff, data, sizeof(SuperCap_Ctrl_s));
    CANTransmit(instance->can_ins, 1);
}

SuperCap_Msg_s SuperCapGet(SuperCapInstance *instance)
{
    return instance->cap_msg;
}
