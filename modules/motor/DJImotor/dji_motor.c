#include "dji_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/* ==================== 全局变量声明 ==================== */
//用于 yaw1 超限位后补偿速度给 yaw2
float yaw1_aim_speed=0.0f;          ///< yaw1 电机目标速度（用于 yaw2 前馈补偿）

static uint8_t yaw1_motor_id = 0;   ///< yaw1 电机在数组中的索引（固定为 0）
static uint8_t pitch_motor_id = 1;  ///< pitch电机在数组中的索引（固定为 1）

float err_feedback_yaw1_to_yaw2;    ///< 小 yaw 超限时的补偿量（小 yaw 给大 yaw 的前馈）

static uint8_t idx = 0;             ///< 电机注册索引，表示已注册的电机数量

/* ==================== 电机实例数组 ==================== */
/* DJI 电机的实例，此处仅保存指针，内存的分配将通过电机实例初始化时通过 malloc() 进行 */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL}; // 会在 control 任务中遍历该指针数组进行 pid 计算

/* ==================== CAN 发送分组配置 ==================== */
#ifdef FDCAN
/**
 * @brief FDCAN 模式下的发送分组配置（9 个分组）
 *        hfdcan1: 3 组 (0x1ff, 0x200, 0x2ff)
 *        hfdcan2: 3 组 (0x1ff, 0x200, 0x2ff)
 *        hfdcan3: 3 组 (0x1ff, 0x200, 0x2ff)
 */
static CANInstance sender_assignment[9] = {
    [0] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x1ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [1] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x200, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [2] = {.can_handle = &hfdcan1, .txconf.Identifier = 0x2ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [3] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x1ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [4] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x200, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [5] = {.can_handle = &hfdcan2, .txconf.Identifier = 0x2ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [6] = {.can_handle = &hfdcan3, .txconf.Identifier = 0x1ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [7] = {.can_handle = &hfdcan3, .txconf.Identifier = 0x200, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
    [8] = {.can_handle = &hfdcan3, .txconf.Identifier = 0x2ff, .txconf.IdType = FDCAN_STANDARD_ID, .txconf.TxFrameType = FDCAN_DATA_FRAME, .txconf.DataLength = FDCAN_DLC_BYTES_8, .txconf.FDFormat = FDCAN_CLASSIC_CAN,.txconf.BitRateSwitch = FDCAN_BRS_OFF, .tx_buff = {0}},
};
#else

/**
 * @brief CAN 模式下的发送分组配置（6 个分组）
 * 
 * @details 由于 DJI 电机发送以四个一组的形式进行，故对其进行特殊处理，用 6 个 (2CAN×3group) can_instance 专门负责发送
 *          该变量将在 DJIMotorControl() 中使用，分组在 MotorSenderGrouping() 中进行
 * 
 * @note 因为只用于发送，所以不需要在 bsp_can 中注册
 * 
 * **DJI 电机 CAN ID 分配规则**：
 * - C610(M2006)/C620(M3508): 发送 0x1ff/0x200，接收 0x200+id
 * - GM6020: 发送 0x1ff/0x2ff，接收 0x204+id
 * - CAN1: [0]:0x1FF, [1]:0x200, [2]:0x2FF
 * - CAN2: [3]:0x1FF, [4]:0x200, [5]:0x2FF
 */
static CANInstance sender_assignment[6] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0}},
};

#endif

/**
 * @brief 发送使能标志位数组
 * 
 * @details 6 个（或 9 个）用于确认是否有电机注册到 sender_assignment 中的标志位，防止发送空帧
 *          此变量将在 DJIMotorControl() 中使用
 *          flag 的初始化在 MotorSenderGrouping() 中进行
 * 
 * @note 只有当某个分组内至少有一个电机注册时，对应的标志位才会被置 1，才发送 CAN 报文
 */
static uint8_t sender_enable_flag[9] = {0};

/**
 * @brief DJI 电机分组和 ID 配置函数
 * 
 * @param motor 待配置的电机实例指针
 * @param config CAN 初始化配置结构体指针
 * 
 * @details 功能说明：
 *          1. 根据电调/拨码开关上的 ID，根据说明书的默认 id 分配方式计算发送 ID 和接收 ID
 *          2. 对电机进行分组以便处理多电机控制命令（每 4 个电机为一组）
 * 
 * **DJI 电机 CAN 报文分组规则**：
 * - M2006/M3508: ID 1-4 → 组 0(0x1ff), ID 5-8 → 组 1(0x200)
 * - GM6020: ID 1-4 → 组 0(0x1ff), ID 5-8 → 组 2(0x2ff)
 * - CAN1: 偏移量 0, CAN2: 偏移量 3, CAN3: 偏移量 6
 * 
 * **接收 ID 计算公式**：
 * - M2006/M3508: rx_id = 0x200 + motor_id + 1
 * - GM6020: rx_id = 0x204 + motor_id + 1
 * 
 * @note 此函数会自动检测并报告 ID 冲突（同一 CAN 总线上重复的接收 ID）
 */
static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始，先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    uint8_t grouping_offset;
    //通过 CAN 计算分组偏移量
    if(config->can_handle == &hcan1)
    {
        grouping_offset=0;
    }
    else if(config->can_handle == &hcan2)
    {
        grouping_offset=3;
    }
    else
    {
        grouping_offset=6;
    }

    /* ==================== M2006/M3508 电机分组逻辑 ==================== */
    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4) // 根据 ID 分组
        {
            motor_send_num = motor_id;
            motor_grouping = grouping_offset + 1;
            
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = grouping_offset + 0;
        }

        // 计算接收 id 并设置分组发送 id
        config->rx_id = 0x200 + motor_id + 1;   // 把 ID+1，进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 设置发送标志位，防止发送空帧
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        /* --- ID 冲突检测 --- */
        // 检查是否发生 id 冲突
        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1) // 6020的 id 1-4和 2006/3508 的 id 5-8会发生冲突 (若有注册，即 1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                    LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
            }
        }
        break;

    /* ==================== GM6020 电机分组逻辑 ==================== */
    case GM6020:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = grouping_offset + 0;
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = grouping_offset + 2;
        }

        config->rx_id = 0x204 + motor_id + 1;   // 把 ID+1，进行分组设置
        sender_enable_flag[motor_grouping] = 1; // 只要有电机注册到这个分组，置为 1;在发送函数中会通过此标志判断是否有电机注册
        motor->message_num = motor_send_num;
        motor->sender_group = motor_grouping;

        /* --- ID 冲突检测 --- */
        for (size_t i = 0; i < idx; ++i)
        {
            if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle && dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id)
            {
                LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
                uint16_t can_bus = config->can_handle == &hcan1 ? 1 : 2;
                while (1) // 6020 的 id 1-4 和 2006/3508 的 id 5-8会发生冲突 (若有注册，即 1!5,2!6,3!7,4!8) (1!5!,LTC! (((不是)
                    LOGERROR("[dji_motor] id [%d], can_bus [%d]", config->rx_id, can_bus);
            }
        }
        break;

    default: // other motors should not be registered here
        while (1)
            LOGERROR("[dji_motor]You must not register other motors using the API of DJI motor."); // 其他电机不应该在这里注册
    }
}

/**
 * @brief DJI 电机 CAN 反馈报文解析函数
 * 
 * @param _instance 收到数据的 CAN instance 指针
 * 
 * @details 功能说明：
 *          1. 通过 CAN instance 的 id 字段获取对应的电机实例地址（id 在注册时设置为电机实例指针）
 *          2. 解析 CAN 接收缓冲区中的 8 字节数据：
 *             - 字节 0-1: 编码器位置 (ECD)
 *             - 字节 2-3: 电机转速 (RPM)
 *             - 字节 4-5: 电机电流 (mA)
 *             - 字节 6: 电机温度 (°C)
 *             - 字节 7: 保留
 *          3. 对电流和速度进行一阶低通滤波处理
 *          4. 计算多圈绝对角度（基于圈数计数和单圈角度）
 * 
 * **多圈角度计算原理**：
 * - 假设两次采样间电机转过的角度小于 180°（4096 ECD 单位）
 * - 当 ecd 差值 > 4096 时，认为电机逆时针转过一圈，total_round--
 * - 当 ecd 差值 < -4096 时，认为电机顺时针转过一圈，total_round++
 * - total_angle = total_round × 360° + angle_single_round
 * 
 * @note 此函数作为 CAN 接收回调函数被自动调用
 */
static void DecodeDJIMotor(CANInstance *_instance)
{
    // 这里对 can instance 的 id 进行了强制转换，从而获得电机的 instance 实例地址
    // _instance 指针指向的 id 是对应电机 instance 的地址，通过强制转换为电机 instance 的指针，再通过->运算符访问电机的成员 motor_measure，最后取地址获得指针
    uint8_t *rxbuff = _instance->rx_buff;
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
    DJI_Motor_Measure_s *measure = &motor->measure; // measure 要多次使用，保存指针减小访存开销

    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    /* ==================== 解析原始数据并滤波 ==================== */
    // 解析数据并对电流和速度进行滤波，电机的反馈报文具体格式见电机说明手册
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    /* ==================== 多圈角度计算 ==================== */
    // 多圈角度计算，前提是假设两次采样间电机转过的角度小于 180°，自己画个图就清楚计算过程了
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

/**
 * @brief DJI 电机丢失回调函数
 * 
 * @param motor_ptr 电机实例指针（void* 类型，由守护线程系统传递）
 * 
 * @details 当电机通信超时（超过 20ms 未收到反馈数据）时，由守护线程系统自动调用此函数
 *          输出警告信息，包含 CAN 总线编号和电机 ID，便于调试定位问题
 */
static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;
    uint16_t can_bus = motor->motor_can_instance->can_handle == &hcan1 ? 1 : 2;
    LOGWARNING("[dji_motor] Motor lost, can bus [%d] , id [%d]", can_bus, motor->motor_can_instance->tx_id);
}

/* ==================== 公开接口函数实现 ==================== */
/**
 * @brief DJI 电机初始化函数
 * 
 * @param config 电机初始化配置结构体指针
 * @return DJIMotorInstance* 电机实例指针
 * 
 * @details 初始化流程：
 *          1. 动态分配电机实例内存并清零
 *          2. 配置电机基本参数（型号、控制设置等）
 *          3. 初始化三环 PID 控制器（角度环、速度环、电流环）
 *          4. 配置前馈控制指针（速度前馈、电流前馈）
 *          5. 执行电机分组和 ID 配置（MotorSenderGrouping）
 *          6. 注册 CAN 接收回调函数（DecodeDJIMotor）
 *          7. 注册守护线程用于检测通信丢失（20ms 超时）
 *          8. 使能电机并保存到全局实例数组
 * 
 * @note 该函数会动态分配内存，无需手动释放（系统运行期间电机实例持续存在）
 * @note 同一 CAN 总线上不建议挂载超过 6 个电机，否则需降低反馈频率和控制周期
 */
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    memset(instance, 0, sizeof(DJIMotorInstance));

    // motor basic setting 电机基本设置
    instance->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    instance->motor_settings = config->controller_setting_init_config; // 正反转，闭环类型等

    /* ==================== 电机控制器初始化 ==================== */
    // motor controller init 电机控制器初始化
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    // 后续增加电机前馈控制器 (速度和电流)

    /* ==================== 电机分组和 CAN 注册 ==================== */
    // 电机分组，因为至多 4 个电机可以共用一帧 CAN 控制报文
    MotorSenderGrouping(instance, &config->can_init_config);

    // 注册电机到 CAN 总线
    config->can_init_config.can_module_callback = DecodeDJIMotor; // set callback
    config->can_init_config.id = instance;                        // set id,eq to address(it is identity)
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    /* ==================== 守护线程注册 ==================== */
    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback = DJIMotorLostCallback,
        .owner_id = instance,
        .reload_count = 2, // 20ms 未收到数据则丢失
    };
    instance->daemon = DaemonRegister(&daemon_config);

    DJIMotorEnable(instance);
    dji_motor_instance[idx++] = instance;
    return instance;
}

/**
 * @brief 切换电机反馈数据来源
 * 
 * @param motor 电机实例指针
 * @param loop 要切换的闭环类型（角度环或速度环）
 * @param type 目标反馈源（电机编码器反馈或其他传感器反馈）
 * 
 * @details 功能说明：
 *          - 当 loop == ANGLE_LOOP 时，切换角度环的反馈源
 *          - 当 loop == SPEED_LOOP 时，切换速度环的反馈源
 *          - OTHER_FEED: 使用外部传感器（如 IMU）作为反馈
 *          - MOTOR_FEED: 使用电机自身编码器作为反馈
 * 
 * @note 典型应用：小陀螺模式下将云台电机的反馈源切换为 IMU 数据
 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOGERROR("[dji_motor] loop type error, check memory access and func param"); // 检查是否传入了正确的 LOOP 类型，或发生了指针越界
}

/**
 * @brief 停止电机
 * 
 * @param motor 电机实例指针
 * 
 * @details 设置电机停止标志位，DJIMotorControl() 会将该电机的发送数据置零
 * 
 * @note 不是立即切断电流，而是通过控制循环逐步停止
 */
void DJIMotorStop(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

/**
 * @brief 使能电机
 * 
 * @param motor 电机实例指针
 * 
 * @details 清除电机停止标志位，电机恢复正常控制响应
 * 
 * @note 初始化时 stop_flag 默认为 0（使能状态），通常不需要手动调用此函数
 */
void DJIMotorEnable(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/**
 * @brief 修改电机外环控制模式
 * 
 * @param motor 电机实例指针
 * @param outer_loop 外环类型（ANGLE_LOOP 角度环 / SPEED_LOOP 速度环）
 * 
 * @details 动态切换电机的最外层闭环控制模式
 * 
 * @note 典型应用：拨盘电机在位置控制和速度控制之间切换
 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

/**
 * @brief 设置电机前馈控制标志
 * 
 * @param motor 电机实例指针
 * @param feedfoward_loop 前馈类型标志（速度前馈/电流前馈的组合）
 * 
 * @details 启用或禁用电机的前馈控制通道
 * 
 * @note 前馈控制可以提高系统的响应速度和抗扰性能
 */
void DJIMotorSetFeedfoward(DJIMotorInstance *motor, Feedfoward_Type_e feedfoward_loop)
{
    motor->motor_settings.feedforward_flag = feedfoward_loop;
}

/**
 * @brief 设置电机参考输入
 * 
 * @param motor 电机实例指针
 * @param ref 参考值（角度、速度或电流，取决于当前外环类型）
 * 
 * @details 为电机设定目标控制值
 * 
 * @note 对于应用层，可以将电机视为传递函数为 1 的设备，不需要关心底层的闭环控制细节
 */
// 设置参考值
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

/* ==================== 调试变量 ==================== */
int16_t value;

float measure_speed;
float measure_ref;

// 为所有电机实例计算三环 PID，发送控制报文

static uint8_t measure_motor_id = 1;  ///< 用于调试的电机索引（未使用）

/**
 * @brief 电机调试数据结构体
 * 
 * @details 用于存储特定电机的调试数据（角度环、速度环、最终输出等）
 */
typedef struct
{
    float measure_angle_aim_value;    ///< 角度环目标值
    float measure_angle_now_value;    ///< 角度环实际值
    float measure_angle_out_value;    ///< 角度环输出
    float measure_speed_aim_value;    ///< 速度环目标值
    float measure_speed_now_value;    ///< 速度环实际值
    float measure_speed_out_value;    ///< 速度环输出
    float measure_final_out_value;    ///< 最终输出值
} measure_data_s;

measure_data_s measure_data;  ///< 全局调试数据实例

/* ==================== DJI 电机核心控制任务 ==================== */
/**
 * @brief DJI 电机控制主函数 - 三环 PID 计算和 CAN 报文发送
 * 
 * @details 功能流程：
 * 
 *          1. **遍历所有已注册电机**：
 *             - 获取电机实例指针和相关配置参数
 *             - 处理反转标志（若启用则参考值取反）
 * 
 *          2. **三环 PID 计算**（按顺序串联）：
 *             a) **角度环**（外环）：
 *                - 反馈源选择：OTHER_FEED（外部传感器如 IMU）或 MOTOR_FEED（电机编码器）
 *                - 计算位置误差并输出速度指令
 *                - yaw1 电机软件限位保护（10°-60° 和 240°-290°区域禁止正向/反向转动）
 *             
 *             b) **速度环**（中环）：
 *                - 叠加速度前馈量（若启用）
 *                - 反馈源选择：同角度环
 *                - 计算速度误差并输出电流指令
 *             
 *             c) **电流环**（内环）：
 *                - 叠加电流前馈量（若启用）
 *                - 仅支持 MOTOR_FEED（电机电流传感器）
 *                - 计算电流误差并输出最终控制量
 * 
 *          3. **软件限位保护**：
 *             - yaw1 电机：在 10°-60° 和 240°-290° 两个危险区域强制停止
 *             - pitch电机：在 230°-268° 区域强制停止
 *             - 保护逻辑：当检测到进入限位区且输出方向会使情况恶化时，将输出置零
 * 
 *          4. **CAN 报文组装**：
 *             - 根据电机分组（sender_group）和组内编号（message_num）填入对应位置
 *             - 每个 CAN 报文最多包含 4 个电机的控制量（共 8 字节）
 *             - 若电机处于停止状态，直接将对应位置零
 * 
 *          5. **批量发送 CAN 报文**：
 *             - 遍历 sender_enable_flag 数组
 *             - 只发送有电机注册的分组（flag==1）
 *             - 提高总线利用率，避免发送空帧
 * 
 * @note 该函数应被 motor_task 调用，运行在 RTOS 上
 * @note 控制频率建议：M2006/M3508 ≤ 1kHz，GM6020 ≤ 500Hz
 * @note 同一 CAN 总线上的电机数量建议不超过 6 个
 * 
 * @todo 考虑加入力矩传感器、应变片等更精确的电流监测手段
 */
void DJIMotorControl()
{
    // 直接保存一次指针引用从而减小访存的开销，同样可以提高可读性
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // 电机控制 CAN 发送设定值
    DJIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    DJI_Motor_Measure_s *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机 PID 测量值和设定值

    /* ==================== 步骤 1: 遍历所有电机计算三环 PID ==================== */
    // 遍历所有电机实例，进行串级 PID 的计算并设置发送报文的值
    for (size_t i = 0; i < idx; ++i)
    { // 减小访存开销，先保存指针引用
        motor = dji_motor_instance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref; // 保存设定值，防止 motor_controller->pid_ref 在计算过程中被修改
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转

        /* ==================== 步骤 2a: 角度环计算 ==================== */
        // pid_ref 会顺次通过被启用的闭环充当数据的载体
        // 计算位置环，只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle; // MOTOR_FEED，对 total angle 闭环，防止在边界处出现突跃
            
            if(i==measure_motor_id)
            {
                measure_data.measure_angle_aim_value=pid_ref;
                measure_data.measure_angle_now_value=pid_measure;
            }
            // 更新 pid_ref 进入下一个环
            if (motor_setting->angle_feedback_source == OTHER_FEED)
            {pid_ref = DMPIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);}
            else
            {pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);}
            
            if(i==measure_motor_id)
            {measure_data.measure_angle_out_value=pid_ref;}

            if(i==yaw1_motor_id)
            { yaw1_aim_speed=pid_ref;}  // 记录 yaw1 的速度输出用于 yaw2 前馈
        }

        /* ==================== 步骤 2b: 速度环计算 ==================== */
        // 计算速度环，(外层闭环为速度或位置) 且 (启用速度环) 时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed_aps;
            // 更新 pid_ref 进入下一个环
            if(i==measure_motor_id)
            {
                measure_data.measure_speed_aim_value=pid_ref;
                measure_data.measure_speed_now_value=pid_measure;
            }

            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);

            if(i==measure_motor_id)
            {
                measure_data.measure_speed_out_value=pid_ref;
            }

            measure_speed = pid_measure;
            measure_ref = pid_ref;
        }

        /* ==================== 步骤 2c: 电流环计算 ==================== */
        // 计算电流环，目前只要启用了电流环就计算，不管外层闭环是什么，并且电流只有电机自身传感器的反馈
        //if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
        pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 获取最终输出
        set = (int16_t)pid_ref;

        /* ==================== 步骤 3: 软件限位保护 ==================== */
        //对小 yaw1 电机进行软件保护
        if(i==yaw1_motor_id)
        {
            if((motor->measure.angle_single_round>10 && motor->measure.angle_single_round<60) && set>0)
            {
                //err_feedback_yaw1_to_yaw2=0.1*yaw1_aim_speed;//传给 yaw2 的补偿
                set=0;//补偿力，防止在临界位置一直抖动
                pid_ref = 0;
            }
            else if((motor->measure.angle_single_round<290 && motor->measure.angle_single_round>240) && set<0)
            {
                //err_feedback_yaw1_to_yaw2=0.1*yaw1_aim_speed;//传给 yaw2 的补偿
                set=0;//补偿力，防止在临界位置一直抖动
                pid_ref = 0;
            }
            else
            {err_feedback_yaw1_to_yaw2=0;}

            // if(set >= 20000)
            // {set=0;}
        }

        //对 pitch电机进行软件保护
        if(i==pitch_motor_id)
        {
            if(motor->measure.angle_single_round>268 && set>0)
            {
                set=0;//补偿力，防止在临界位置一直抖动
            }
            else if(motor->measure.angle_single_round<230  && set<0)
            {
                set=0;//补偿力，防止在临界位置一直抖动
            }

            // if(set >= 20000)
            // {set=0;}
        }

        if(i==measure_motor_id)
        {measure_data.measure_final_out_value=pid_ref;}

        /* ==================== 步骤 4: CAN 报文组装 ==================== */
        // 分组填入发送数据
        group = motor->sender_group;
        num = motor->message_num;
        sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);         // 低八位
        sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 高八位

        // 若该电机处于停止状态，直接将 buff 置零
        if (motor->stop_flag == MOTOR_STOP)
            memset(sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
    }

    /* ==================== 步骤 5: 批量发送 CAN 报文 ==================== */
    // 遍历 flag，检查是否要发送这一帧报文
#ifdef FDCAN
    for (size_t i = 0; i < 9; ++i)
#else
    for (size_t i = 0; i < 6; ++i)
#endif
    {
        if (sender_enable_flag[i])
        {
            CANTransmit(&sender_assignment[i], 1);
        }
    }
}
