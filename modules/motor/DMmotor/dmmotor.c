#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "dm_imu.h"
#include "ins_task.h"
#include "robot_cmd.h"
#include "robot_def.h"
#include "vofa.h"
#include "gimbal.h"
#include "dji_motor.h"
#include "gain_schedule.h"

/* ==================== 外部依赖变量声明 ==================== */
// float shootData[3]= {0};

//extern imu_t imu;                    ///< 全局 IMU 实例（姿态数据）
//extern INS_t INS;                    ///< 惯性导航系统实例

//extern DJIMotorInstance *yaw1_motor; ///< yaw1 电机实例（用于耦合补偿）
extern float yaw1_motor_zero_position; ///< yaw1 电机零点位置

extern Chassis_Upload_Data_s chassis_fetch_data;  ///< 底盘反馈数据（功率、热量、运动状态等）
//extern Chassis_Ctrl_Cmd_s chassis_cmd_send;       ///< 发送给底盘的控制指令（含 UI 绘制）

/* ==================== 电机实例管理 ==================== */
static uint8_t idx;                              ///< 已注册 DM 电机数量索引
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];  ///< DM 电机实例指针数组
static osThreadId dm_task_handle[DM_MOTOR_CNT];         ///< DM 电机任务句柄数组

/* ==================== 重力补偿拟合函数 ==================== */
/**
 * @brief 重力补偿力矩拟合函数（枪管放下状态）
 * 
 * @param theta_rad 电机角度（弧度制）
 * @return double 补偿力矩值
 * 
 * @details 上部分正弦拟合函数，对应枪管放下工况
 *          公式：τ_lower(θ) = -0.55443236 × sin(θ - (-1.48681688)) + 0.08980941
 * 
 * @note 参数说明：
 *       - mL = -0.55443236: 力矩幅值系数
 *       - theta0 = -1.48681688: 相位偏移（弧度）
 *       - C = 0.08980941: 力矩偏置
 */
double tau_lower(double theta_rad) {
    double mL = -0.55443236;
    double theta0 = -1.48681688;
    double C = 0.08980941;
    return mL * sin(theta_rad - theta0) + C;
}

/**
 * @brief 重力补偿力矩拟合函数（枪管抬起状态）
 * 
 * @param theta_rad 电机角度（弧度制）
 * @return double 补偿力矩值
 * 
 * @details 下部分正弦拟合函数，对应枪管抬起工况
 *          公式：τ_lower(θ) = -0.42157359 × sin(θ - 5.30738329) + (-0.37167813)
 * 
 * @note 参数说明：
 *       - mL = -0.22157359: 力矩幅值系数
 *       - theta0 = 4.80738329: 相位偏移（弧度，已调整）
 *       - C = -0.87167813: 力矩偏置
 */
double tau_upper(double theta_rad) {
    double mL = -0.22157359;
    double theta0 = 4.80738329;//5.30738329
    double C = -0.87167813;
    return mL * sin(theta_rad - theta0) + C;
}

/**
 * @brief 全局重力补偿力矩拟合函数
 * 
 * @param theta_rad 电机角度（弧度制）
 * @return double 重力力矩值
 * 
 * @details 全局正弦拟合函数，仅对应重力分量
 *          公式：τ_global(θ) = -0.50313777 × sin(θ - 6.73306800) + (-0.87631187)
 * 
 * @note 参数说明：
 *       - mL = -0.50313777: 力矩幅值系数
 *       - theta0 = 6.73306800: 相位偏移（弧度）
 *       - C = -0.87631187: 力矩偏置
 */
double tau_global(double theta_rad) {
    double mL = -0.50313777;
    double theta0 = 6.73306800;
    double C = -0.87631187;
    return mL * sin(theta_rad - theta0) + C;
}

/* ==================== 数据类型转换工具函数 ==================== */
/**
 * @brief 浮点数转无符号整数（用于 CAN 报文编码）
 * 
 * @param x 待转换的浮点数值
 * @param x_min 数值范围最小值
 * @param x_max 数值范围最大值
 * @param bits 量化位数
 * @return uint16_t 量化后的整数值
 * 
 * @details 将浮点数线性映射到指定位数的整数范围
 *          公式：uint = (x - offset) × (2^bits - 1) / span
 * 
 * @note 典型应用：电机位置、速度、力矩指令的 CAN 报文编码
 */
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
 * @brief 无符号整数转浮点数（用于 CAN 报文解析）
 * 
 * @param x_int 量化后的整数值
 * @param x_min 数值范围最小值
 * @param x_max 数值范围最大值
 * @param bits 量化位数
 * @return float 还原后的浮点数值
 * 
 * @details 将量化整数还原为实际物理量
 *          公式：float = x_int × span / (2^bits - 1) + offset
 * 
 * @note 典型应用：电机反馈数据的 CAN 报文解析
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* ==================== 角度数据处理工具函数 ==================== */
/**
 * @brief IMU 角度数据范围重置（-180° ~ 180°）
 * 
 * @param data 待处理的角度值（单位：度）
 * @return float 归一化后的角度值，范围 [-180, 180]
 * 
 * @details 将超出范围的角度值通过加减 360° 的方式映射到 [-180°, 180°] 区间内
 *          用于保证云台目标角度的连续性和一致性
 */
float reset_imu_data_range(float data)
{
    return theta_format(data);
}

/* ==================== 内部辅助函数实现 ==================== */
/**
 * @brief DM 电机模式设置函数
 * 
 * @param cmd 模式命令（DMMotor_Mode_e 枚举值）
 * @param motor 电机实例指针
 * 
 * @details 发送模式切换命令到电机驱动器
 *          命令格式：前 7 字节为 0xff，第 8 字节为命令字
 * 
 * **支持的命令**：
 * - DM_CMD_MOTOR_MODE (0xfc): 使能模式，响应控制指令
 * - DM_CMD_RESET_MODE (0xfd): 停止模式
 * - DM_CMD_ZERO_POSITION (0xfe): 编码器零位校准
 * - DM_CMD_CLEAR_ERROR (0xfb): 清除过热错误
 */
static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面 7bytes 都是 0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令 id
    CANTransmit(motor->motor_can_instace, 1);
}

/**
 * @brief DM 电机反馈报文解析函数
 * 
 * @param motor_can 接收数据的 CAN 实例指针
 * 
 * @details 解析 DM 电机返回的 8 字节反馈报文：
 *          - 字节 1-2: 电机位置（16 位）
 *          - 字节 3-4: 电机速度（12 位）
 *          - 字节 4-5: 电机扭矩（12 位）
 *          - 字节 6: MOS 管温度
 *          - 字节 7: 转子温度
 * 
 * **数据解析流程**：
 * 1. 通过 CAN instance 的 id 字段获取电机实例指针
 * 2. 提取原始数据并转换为浮点物理量
 * 3. 更新上次位置记录
 * 4. 刷新守护线程看门狗
 */
static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值，稍后转换成 float 数据，避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将 can 实例中保存的 id 转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

/**
 * @brief DM 电机丢失回调函数（空实现）
 * 
 * @param motor_ptr 电机实例指针
 * 
 * @note 目前为空函数，可扩展为报警或安全保护逻辑
 */
static void DMMotorLostCallback(void *motor_ptr)
{
}

/**
 * @brief DM 电机编码器零位校准
 * 
 * @param motor 电机实例指针
 * 
 * @details 发送零位校准命令并延时等待完成
 * 
 * @note 校准过程中电机会旋转到机械零点并重置编码器计数
 */
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

/* ==================== 公开接口函数实现 ==================== */
/**
 * @brief DM 电机初始化函数
 * 
 * @param config 电机初始化配置结构体指针
 * @return DMMotorInstance* 电机实例指针
 * 
 * @details 初始化流程：
 *          1. 动态分配电机实例内存并清零
 *          2. 配置电机控制参数和三环 PID
 *          3. 注册 CAN 接收回调函数（DMMotorDecode）
 *          4. 注册守护线程检测通信丢失（100ms 超时）
 *          5. 使能电机并切换到电机模式
 *          6. 如需设置零点，执行编码器校准
 * 
 * @note 该函数会动态分配内存，无需手动释放
 * @note 守护线程超时时间比 DJI 电机长（10×10ms = 100ms）
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    motor->lost_cnt = 1;//后面再对这个参数进行调整，暂时先设置为 1

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    if(config->motor_type==set_zero)
    {
        DMMotorCaliEncoder(motor);
    }
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

/**
 * @brief 设置 DM 电机参考输入
 * 
 * @param motor 电机实例指针
 * @param ref 参考值（位置、速度或扭矩，取决于外环类型）
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

/**
 * @brief 使能 DM 电机
 * 
 * @param motor 电机实例指针
 * 
 * @details 清除停止标志位，电机恢复正常响应
 */
void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/**
 * @brief 停止 DM 电机
 * 
 * @param motor 电机实例指针
 * 
 * @details 设置停止标志位，但继续接收反馈数据
 * 
 * @note 与使能模式不同，停止状态下仍需要解析反馈报文用于监测
 */
void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

/**
 * @brief 修改 DM 电机外环控制模式
 * 
 * @param motor 电机实例指针
 * @param type 外环类型（ANGLE_LOOP / SPEED_LOOP）
 */
void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

/* ==================== DM 电机控制任务全局变量 ==================== */
float yaw_2_angle_feedback;          ///< yaw2 电机角度反馈（调试用）
float angle_feedback;                ///< 角度反馈值（用于三环计算）
float speed_ref;                     ///< 速度参考值（位置环输出）

//last_set 用于拨弹盘电机设置零点标志位
float err_feedback_yaw2_to_yaw1;     ///< yaw2 给 yaw1 的前馈补偿量
extern float err_feedback_yaw1_to_yaw2; ///< yaw1 给 yaw2 的前馈补偿量（来自 dji_motor.c）

//@Todo:
uint8_t protect_flag = 0;            ///< 保护标志位（未使用）
float forword_speed_feedback=0.0f;  ///< 前馈速度反馈（未使用）
float err;                           ///< 误差变量（未使用）

/* ==================== 增益调度配置 ==================== */
// 引用 gimbal.c 中的参数配置
extern GainScheduleParams_t yaw2_gain_schedule_params;

/* ==================== DM 电机核心控制任务 ==================== */
/**
 * @brief DM 电机控制任务（独立线程）
 * 
 * @param argument 电机实例指针（由 RTOS 传递）
 * 
 * @details 任务执行流程：
 * 
 *          1. **初始化阶段**：
 *             - 切换到电机模式（DM_CMD_MOTOR_MODE）
 *             - 获取电机控制参数指针
 * 
 *          2. **主循环（10ms 周期）**：
 *             a) **反馈源选择**（根据 angle_feedback_source）：
 *                - MOTOR_FEED: 使用 yaw1 电机编码器反馈
 *                - BM1088_FEED: 使用 BMI088 IMU 的 yaw 角
 *                - IMU_FEED: 使用全局 IMU 的 yaw 角
 *             
 *             b) **方向处理**：
 *                - 若启用反向标志，将反馈值取反
 *             
 *             c) **位置环计算**（外环为 ANGLE_LOOP 时）：
 *                - 计算位置 PID 输出（期望速度）
 *                - 记录 yaw2→yaw1 前馈量
 *                - 叠加底盘角速度补偿（wz）
 *                - 减去 yaw1→yaw2 前馈量（双向耦合补偿）
 *                - 设置 Kd = 0x500（微分增益）
 *             
 *             d) **目标值设置**：
 *                - position_des: 位置目标（固定为 0）
 *                - velocity_des: 速度目标（位置环输出）
 *                - torque_des: 扭矩目标（固定为 0）
 *                - Kp: 比例增益（固定为 0）
 *             
 *             e) **停止/保护处理**：
 *                - 若 stop_flag == MOTOR_STOP 或 protect_flag == 1
 *                - 将所有目标值和增益置零
 * 
 *          3. **CAN 报文组装与发送**：
 *             - 按照 DM 电机协议格式打包 8 字节数据：
 *               - 字节 0-1: 位置目标（16 位）
 *               - 字节 2-3: 速度目标（12 位）+ Kp 高 4 位
 *               - 字节 4: Kp 低 8 位 + Kd 高 4 位
 *               - 字节 5-6: Kd 低 12 位 + 扭矩目标高 4 位
 *               - 字节 7: 扭矩目标低 8 位
 *             - 通过 CAN 总线发送
 * 
 *          4. **任务调度**：
 *             - osDelay(10): 10ms 运行周期（100Hz）
 * 
 * @note 该任务为每个 DM 电机实例独立运行
 * @note yaw2 电机通过前馈补偿抵消对 yaw1 的耦合影响
 * @todo 添加重力补偿力矩计算（使用 tau_global 等拟合函数）
 */
void DMMotorTask(void const *argument)
{
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    DMMotor_Send_s motor_send_mailbox;

    while (1)
    {
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
        
        /* ==================== 步骤 2a: 反馈源选择 ==================== */
        // 根据闭环类型选择不同的控制策略
        if(setting->close_loop_type==ANGLE_AND_SPEED_LOOP){
             // 反馈位置 + 电机速度双环控制模式
             //目前只有用到MOTOR_FEED作为反馈源，后续可以扩展为IMU_FEED或BM1088_FEED
            if (setting->angle_feedback_source == MOTOR_FEED ) 
            {
                angle_feedback = *motor->motor_controller.other_angle_feedback_ptr; // 使用 yaw1 电机编码器反馈
            } 
            
            /* ==================== 步骤 2b: 方向处理 ==================== */
            if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
                angle_feedback *= -1;
                
            /* ==================== 步骤 2c: 位置环计算 ==================== */
            // 计算位置环 PID 输出（期望速度）
            if(setting->outer_loop_type==ANGLE_LOOP){//yaw2   加了反向

                // ========== 增益调度: 动态更新PID参数 ==========
                // 计算误差
                float position_error = yaw1_motor_zero_position - angle_feedback;

                // 角度归一化到 [-180, 180]
                position_error = theta_format(position_error);

                // 根据误差大小动态更新Kp/Ki/Kd
                GainSchedule_UpdatePIDParams(&motor->motor_controller.angle_PID,
                                              &yaw2_gain_schedule_params,
                                              position_error);

                // ========== 原有PID计算 ==========
                // 使用更新后的参数进行PID计算
                speed_ref = -DMPIDCalculate(&motor->motor_controller.angle_PID, angle_feedback, yaw1_motor_zero_position);

                // ========== 前馈补偿 ==========
                err_feedback_yaw2_to_yaw1 = speed_ref;  // 记录 yaw2→yaw1 前馈量
                speed_ref += 1 * chassis_fetch_data.real_wz;  // 叠加底盘角速度补偿
                //这里把speed_ref用局部变量会不会更好
                chassis_fetch_data.real_wz = 0;  // 清除底盘角速度（防止重复使用）
                speed_ref -= err_feedback_yaw1_to_yaw2;  // 减去 yaw1→yaw2 前馈量（未使用）
                err_feedback_yaw1_to_yaw2 = 0.0f;  // 清除前馈量

                // 保存调试数据
                yaw_2_angle_feedback = speed_ref;

                // 设置 MIT 协议参数
                motor_send_mailbox.Kd = 0x500;  // 设置微分增益
            }

            /* ==================== 步骤 2d: 目标值设置 ==================== */
            // 设置位置和速度目标
            motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(speed_ref, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox.Kp = 0; 
           
        }

        /* ==================== 步骤 2e: 停止/保护处理 ==================== */
        if(motor->stop_flag == MOTOR_STOP || protect_flag==1) {
            motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
            motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
            motor_send_mailbox.Kp = 0;
            motor_send_mailbox.Kd = 0;
        }

        /* ==================== 步骤 3: CAN 报文组装与发送 ==================== */
        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

        CANTransmit(motor->motor_can_instace,1);
        
        /* ==================== 步骤 4: 任务调度 ==================== */
        osDelay(10);
    }
}

/**
 * @brief DM 电机控制任务初始化函数
 * 
 * @details 为所有已注册的 DM 电机实例创建独立的控制任务
 * 
 * **初始化流程**：
 * 1. 检查是否有电机注册（idx > 0）
 * 2. 遍历所有电机实例
 * 3. 为每个电机创建 RTOS 线程：
 *    - 任务命名："dm" + 索引号（如 dm0, dm1, dm2...）
 *    - 入口函数：DMMotorTask
 *    - 优先级：osPriorityNormal（普通优先级）
 *    - 栈大小：128 words（512 bytes）
 *    - 参数：电机实例指针
 * 4. 保存任务句柄到 dm_task_handle[] 数组
 * 
 * @note 该函数会被 RobotInit() 调用，无需手动执行
 * @note 每个 DM 电机运行在独立线程中，周期 10ms（100Hz）
 * @see DMMotorTask()
 */
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例，创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}