#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"

#include "bmi088.h"
#include "dmmotor.h"
#include "vofa.h"
#include "robot_cmd.h"

#include "xrobot_imu_driver.h"

#include "buzzer.h"
#include "gain_schedule.h"  // 增益调度模块
#include "user_lib.h"

/* ==================== 外部依赖变量声明 ==================== */
extern float err_feedback_yaw2_to_yaw1; ///< yaw2 到 yaw1 的角度误差反馈
//extern INS_t INS;                      ///< 惯性导航系统实例
//extern uint8_t flag_vision_mode;        ///< 视觉模式标志位（1=哨兵模式）
//extern ImuData imu_data;               ///< IMU 原始数据（包含欧拉角和陀螺仪数据）
//extern Vision_Recv_s *vision_recv_data; ///< 视觉接收数据指针（初始化时返回）
//extern Vision_Send_s vision_send_data;  ///< 视觉发送数据结构

/* ==================== yaw2 增益调度配置 ==================== */
GainScheduleParams_t yaw2_gain_schedule_params = {
    .error_threshold = {6.0f, 12.0f, 30.0f},
    .Kp_values = {0.05f, 0.20f, 0.30f, 0.25f},
    .Ki_values = {0.0f, 0.0f, 0.0f, 0.0f},
    .Kd_values = {0.01f, 0.01f, 0.01f, 0.01f},
    .interp_type = INTERP_S_CURVE_3,
};
    
/* ==================== 模块内部全局变量 ==================== */
static attitude_t *gimba_IMU_data;      ///< 云台 IMU 数据指针，指向解算后的姿态信息
DJIMotorInstance *yaw1_motor;           ///< yaw 轴电机 1 实例（小云台电机）
DJIMotorInstance *pitch_motor;          ///< pitch 轴电机实例（俯仰电机）
DMMotorInstance *dmmotor_yaw2;          ///< yaw 轴电机 2 实例（大云台电机/底盘跟随）

static Publisher_t *gimbal_pub;                   ///< 云台消息发布者（向 cmd 反馈状态）
static Subscriber_t *gimbal_sub;                  ///< cmd 控制指令订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; ///< 回传给 cmd 的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         ///< 来自 cmd 的控制指令

static Subscriber_t *x_imu_sub;                   ///< xrobot_imu 数据订阅者
static ImuData imu_data;                          ///< 来自 xrobot_imu 的 IMU 数据

//static Chassis_Upload_Data_s chassis_real_speed;   ///< 底盘实际速度数据
//static Subscriber_t *chassis_speed_sub;            ///< 底盘反馈信息订阅者

/* ==================== 前馈控制相关变量 ==================== */
// yaw2到yaw1的前馈增益系数应该都为负数，表示yaw2的正向运动会产生对yaw1的负向补偿
static float yaw2_to_yaw1_speed_ff_gain = -55.0f;    ///< yaw2 到 yaw1 的速度前馈增益
static float yaw2_to_yaw1_current_ff_gain = 0.0f;   ///< yaw2 到 yaw1 的电流前馈增益

static float yaw_gyro_feedforward = -0.0f;          ///< yaw 轴陀螺仪前馈量（预留）

static float yaw_current_forward_abs = 0.0f;        ///< yaw 轴电流前馈绝对值（未使用）
static float pitch_current_forward_abs = 0.0f;      ///< pitch 轴电流前馈绝对值（未使用）

static float yaw1_current_feedforward = 0.0f;       ///< yaw1 电机电流前馈量
static float yaw1_speed_feedforward = 0.0f;         ///< yaw1 电机速度前馈量（用于 yaw2→yaw1 耦合补偿）

static float pitch_current_feedforward = 0.0f;      ///< pitch电机电流前馈量
static float pitch_speed_feedforward = 0.0f;        ///< pitch电机速度前馈量

static float yaw2_angle_feedback = 0.0f;         ///< yaw2 电机角度前馈量（用于 yaw1→yaw2 耦合补偿）

float yaw1_motor_zero_position = 330.0f;            ///< yaw1 电机机械零点位置（单位：度）

static float yaw1_state_error = 0.0f;                 ///< yaw1 电机状态误差（用于调试）

//static float yaw1_begin_angle = 150.0f;            ///< yaw1 初始角度（已废弃）

/* ==================== 告警装置 ==================== */
BuzzzerInstance* gimbal_motor_alarm;                ///< 云台电机故障蜂鸣器实例
BuzzzerInstance* imu_alarm;                         ///< IMU 故障蜂鸣器实例

/* ==================== IMU 零点基准 ==================== */
float begin_xrimu_yaw = 0;                          ///< IMU yaw 轴零点偏移（记录初始姿态）
float begin_xrimu_pitch = 0;                        ///< IMU pitch 轴零点偏移（记录初始姿态）
float yaw1_aim_angle = 0;                           ///< yaw1 目标角度（经过零点补偿）
float pitch_aim_angle = 0;                          ///< pitch 目标角度（经过零点补偿）

float shootData[2]= {0};
// 定义 VOFA+ USART 实例
// static USARTInstance *vofa_usart_instance;

/* ==================== 辅助函数 ==================== */
/**
 * @brief IMU 角度数据范围重置（-180° ~ 180°）
 * 
 * @param data 待处理的角度值（单位：度）
 * @return float 归一化后的角度值，范围 [-180, 180]
 * 
 * @details 将超出范围的角度值通过加减 360° 的方式映射到 [-180°, 180°] 区间内
 *          用于保证云台目标角度的连续性和一致性
 * 
 * @note 该函数不处理角度跳变问题，仅做简单的范围限制
 */
float reset_xrimu_data_range(float data) // -180~180
{
    return theta_format(data);
}

/**
 * @brief 云台初始化函数
 * 
 * @details 初始化流程：
 *          1. IMU 初始化（获取姿态数据指针）
 *          2. 电机参数配置与初始化：
 *             - yaw1 电机：GM6020，CAN1 ID=1，角度环 + 速度环双闭环，IMU 反馈
 *             - pitch电机：GM6020，CAN1 ID=2，角度环 + 速度环双闭环，IMU 反馈
 *             - yaw2 电机：DM4310，CAN2 ID=8，角度环，编码器反馈
 *          3. 蜂鸣器告警装置注册（分高/低优先级）
 *          4. 消息中心发布者和订阅者注册
 *          5. 初始状态设置为停止
 * 
 * @note 该函数会被 RobotInit() 调用，无需手动执行
 * @see RobotInit()
 */
void GimbalInit()
{
    gimba_IMU_data = INS_Init(); // IMU 先初始化，获取姿态数据指针赋给 yaw 电机的其他数据来源，

    /* ==================== yaw1 电机初始化（小云台yaw轴） ==================== */
    Motor_Init_Config_s yaw1_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
            .rx_id = 1,
        },  
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 23, // 10
                .Ki = 10,
                .Kd = 1,
                .Improve = PID_Trapezoid_Intergral | PID_ChangingIntegrationRate | PID_Integral_Limit | PID_OutputFilter | PID_DerivativeFilter,
                .CoefA = 0.5, // 0.5
                .CoefB = 0.6, // 0.6
                .Output_LPF_RC = 0, // 0
                .Derivative_LPF_RC = 0.015, // 0.008 0.01
                .IntegralLimit = 100,
                .MaxOut = 600,
            },
            .speed_PID = {
                .Kp = 100, // 0
                .Ki = 0, // 0
                .Kd = 0,
                .Improve = PID_Integral_Limit | PID_ErrorHandle  ,
                .IntegralLimit = 200,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &(imu_data.eulr.yaw),
            .other_speed_feedback_ptr=&(imu_data.gyro.z),
            .speed_feedforward_ptr = &yaw1_speed_feedforward,
            .current_feedforward_ptr = &yaw1_current_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED, 
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_AND_SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, 
            .feedforward_flag = SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD,
        },
        .motor_type = GM6020
    };
    yaw1_motor = DJIMotorInit(&yaw1_config);


    /* ==================== pitch电机初始化（云台俯仰轴） ==================== */
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
            .rx_id = 2,
        },  
        .controller_param_init_config = {
            .angle_PID = {
                .Kp =0.8,//0.45
                .Ki = 0.5,//0.5
                .Kd = 0.005,//0.005
                .CoefA = 0.7,//0.7
                .CoefB = 0.6,//0.6
                .DeadBand = 0,//0
                .Output_LPF_RC = 0.01,//0.01
                .Improve = PID_Trapezoid_Intergral |PID_ChangingIntegrationRate | PID_Integral_Limit| PID_OutputFilter|PID_DerivativeFilter,
                .IntegralLimit =30,//5
                .MaxOut = 1200,//600
            },
            .speed_PID = {
                .Kp=8000,//3500
                .Ki =0,//0
                .Kd =0.0005,//0.0005
                .CoefA =1500,//1500
                .CoefB =2000,//2000
                .Output_LPF_RC = 0.005,//0.005
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit  | PID_OutputFilter,
                .IntegralLimit =3000,//3000
                .MaxOut = 20000,//20000
            },
            .other_angle_feedback_ptr = &(imu_data.eulr.pitch),
            .other_speed_feedback_ptr=&(imu_data.gyro.y),
            .speed_feedforward_ptr = &pitch_speed_feedforward,
            .current_feedforward_ptr = &pitch_current_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED, 
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_AND_SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, 
            .feedforward_flag = SPEED_FEEDFORWARD | CURRENT_FEEDFORWARD,
        },
        .motor_type = GM6020
    };
    pitch_motor = DJIMotorInit(&pitch_config);

    /* ==================== yaw2 电机初始化（大云台yaw 轴/底盘跟随） ==================== */
    // 注意: yaw2 使用增益调度 + 原有PID集成
    Motor_Init_Config_s dm4310_yaw2={
       .can_init_config ={
         .can_handle = &hcan2,
         .tx_id = 8 ,
         .rx_id = 0x18
       },
       .controller_param_init_config = {
           .angle_PID = {
                .Kp = 0.3,           // 基础Kp，会被增益调度动态调整
                .Ki = 0.0,           // 基础Ki，会被增益调度动态调整
                .Kd = 0.01,          // 基础Kd，会被增益调度动态调整
                .CoefA = 0.5,        // 变积分系数A
                .CoefB = 0.6,        // 变积分系数B
                .DeadBand = 0.01,    // 死区
                .Output_LPF_RC = 0.01, // 输出低通滤波
                .Improve = PID_Trapezoid_Intergral | PID_ChangingIntegrationRate | PID_Integral_Limit | PID_OutputFilter | PID_DerivativeFilter,
                .IntegralLimit = 1,  // 积分限幅
                .MaxOut = 45,        // 最大输出
            },
            .other_angle_feedback_ptr = &yaw2_angle_feedback, // yaw2 电机角度反馈指针
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,          //这里改成前馈计算和使用的标志位了，不是原来的意思，ANGLE_LOOP 是被计算反馈，SPEED_LOOP 是使用反馈
            .close_loop_type = ANGLE_AND_SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL
        },
        .motor_type = no_set_zero
    };
    dmmotor_yaw2 = DMMotorInit(&dm4310_yaw2);

    /* ==================== 蜂鸣器告警装置初始化 ==================== */
    BuzzerInit();

    Buzzer_config_s buzzer_config ={
        .alarm_level = ALARM_LEVEL_HIGH, //设置警报等级 同一状态下 高等级的响应
        .loudness=  0.0, //设置响度
        .octave=  OCTAVE_1, // 设置音阶
    };
    gimbal_motor_alarm = BuzzerRegister(&buzzer_config);
    buzzer_config.octave = OCTAVE_8;
    imu_alarm = BuzzerRegister(&buzzer_config);


    /* ==================== 消息中心注册 ==================== */
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    x_imu_sub = SubRegister("x_imu_feed", sizeof(ImuData)); // 订阅来自 INS_Task 的 IMU 数据
    //chassis_speed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    /* ==================== 初始状态设置为停止 ==================== */
    DJIMotorStop(yaw1_motor);
    DJIMotorStop(pitch_motor);
    DMMotorStop(dmmotor_yaw2);
}

/**
 * @brief 检查电机和 IMU 在线状态
 * 
 * @return uint8_t 在线状态标志位：1=全部在线，0=有设备离线
 * 
 * @details 检测逻辑：
 *          1. IMU 离线检测：当 yaw、roll、pitch 三个欧拉角同时为 0 时判定为离线
 *          2. 电机离线检测：
 *             - yaw1/pitch电机：dt（通信周期）为 0 表示离线
 *             - yaw2 电机：lost_cnt（丢失计数）为 0 表示在线（注意逻辑反向）
 *          3. 告警触发：
 *             - IMU 离线 → 触发 imu_alarm 高音报警
 *             - 电机离线 → 触发 gimbal_motor_alarm 低音报警
 * 
 * @note 该函数会直接操作蜂鸣器硬件，检测到离线时会立即报警
 */
uint8_t cheek_motor_and_imu_online()
{ 
    uint8_t flag = 1;

    if(imu_data.eulr.yaw==0 && imu_data.eulr.roll==0 && imu_data.eulr.pitch==0)
    {
        flag = 0;AlarmSetStatus(imu_alarm, ALARM_ON);
    }
    else if(yaw1_motor->dt==0 || pitch_motor->dt==0 || dmmotor_yaw2->lost_cnt==0)
    {
        flag = 0;AlarmSetStatus(gimbal_motor_alarm, ALARM_ON);
    }
    else
    {
        AlarmSetStatus(gimbal_motor_alarm, ALARM_OFF);AlarmSetStatus(imu_alarm, ALARM_OFF);
    }

    return flag;
}

/* ==================== 云台核心控制任务 ==================== */
/**
 * @brief 机器人云台控制核心任务
 * 
 * @details 任务执行流程：
 *          1. 数据获取：
 *             - 从消息总线订阅云台控制指令（gimbal_cmd）
 *             - 订阅底盘速度反馈（chassis_feed）
 * 
 *          2. 模式切换（根据 gimbal_cmd_recv.gimbal_mode）：
 *             a) GIMBAL_ZERO_FORCE（停止模式）：
 *                - 记录 IMU 初始位置作为零点偏移
 *                - 计算目标角度并设置电机参考值
 *                - 停止所有电机输出
 *             
 *             b) GIMBAL_GYRO_MODE（陀螺仪反馈模式 - 主要工作模式）：
 *                - 执行在线检测，异常时停机保护
 *                - 计算 yaw/pitch目标角度（含零点补偿）
 *                - 设置 yaw2→yaw1 耦合前馈补偿（yaw2 到 yaw1 的速度前馈增益 * 速度误差）
 *                - 视觉模式：使用 IMU 反馈控制 pitch
 *                - 遥控模式：使用编码器反馈，带死区判断防止抖动
 *             
 *             c) GIMBAL_FREE_MODE（自由模式 - 已废弃）：
 *                - 原用于吊射/能量机关等特殊场景
 *                - 当前代码已注释，后续考虑删除
 * 
 *          3. 状态反馈：
 *             - 打包 IMU 数据和电机角度信息
 *             - 发布给视觉模块和 cmd 模块
 * 
 * @note 控制策略说明：
 *       - 目前主要使用 GIMBAL_GYRO_MODE，其他模式为历史遗留代码
 *       - yaw 电机采用 IMU 陀螺仪反馈实现自稳
 *       - pitch电机在视觉模式下用 IMU 反馈，遥控模式下用电机编码器反馈
 *       - yaw2 电机通过前馈补偿抵消对 yaw1 的耦合影响
 * 
 * @todo 添加 pitch 重力补偿前馈力矩计算
 */
void GimbalTask()
{
    uint8_t remote_pitch_contral_flage = 0;///< pitch 轴遥控映射使能标志位
    float err_remote_pitch; ///< 遥控 pitch 指令与电机实际位置的误差（用于死区判断）

    /* ==================== 步骤 1: 获取云台控制数据 ==================== */
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    SubGetMessage(x_imu_sub, &imu_data); // 获取最新 IMU 姿态数据
    //SubGetMessage(chassis_speed_sub, &chassis_real_speed);

    // //判断 xrimu 是否离线
    // while(begin_xrimu_yaw==0 || begin_xrimu_pitch==0)
    // {
    //     begin_xrimu_yaw=imu_data.eulr.yaw;
    //     begin_xrimu_pitch=imu_data.eulr.pitch;
    // }

    // @todo:现在已不再需要电机反馈，实际上可以始终使用 IMU 的姿态数据来作为云台的反馈，yaw 电机的 offset 只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡，视觉模式在 robot_cmd 模块就已经设置好，gimbal 只看 yaw_ref 和 pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
        /* ==================== 模式 A: 零力矩停止模式 ==================== */
        // 停止
    case GIMBAL_ZERO_FORCE:
        remote_pitch_contral_flage=0;
        DJIMotorStop(yaw1_motor);
        DJIMotorStop(pitch_motor);
        DMMotorStop(dmmotor_yaw2);
        begin_xrimu_yaw=imu_data.eulr.yaw - gimbal_cmd_recv.yaw;
        begin_xrimu_pitch=imu_data.eulr.pitch - gimbal_cmd_recv.pitch;

        yaw1_aim_angle = reset_xrimu_data_range(gimbal_cmd_recv.yaw + begin_xrimu_yaw);
        pitch_aim_angle = reset_xrimu_data_range(gimbal_cmd_recv.pitch + begin_xrimu_pitch);

        //yaw1_speed_feedforward = -err_feedback_yaw2_to_yaw1*50;//大 yaw2 给小 yaw1 的前馈

        DJIMotorSetRef(yaw1_motor, yaw1_aim_angle);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorSetRef(pitch_motor, pitch_aim_angle);

        // if(flag_vision_mode==1)//哨兵模式
        // {
        //     DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        //     DJIMotorSetRef(pitch_motor, pitch_aim_angle);
        // }
        // else//非哨兵模式
        // {
        //     DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, MOTOR_FEED);
        //     DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        // }

        break;

        /* ==================== 模式 B: 陀螺仪反馈模式（主要工作模式） ==================== */
        // 使用陀螺仪的反馈，底盘根据 yaw 电机的 offset 跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        /* --- 在线检测与使能控制 --- */
        if(cheek_motor_and_imu_online())
        {
            DJIMotorEnable(yaw1_motor);
            DJIMotorEnable(pitch_motor);
            DMMotorEnable(dmmotor_yaw2);
            // DJIMotorStop(yaw1_motor);
            // DJIMotorStop(pitch_motor);
            // DMMotorStop(dmmotor_yaw2);
        }
        else
        { 
            /* 设备离线，立即停机保护 */
            DJIMotorStop(yaw1_motor);
            DJIMotorStop(pitch_motor);
            DMMotorStop(dmmotor_yaw2);           
        }

        /* --- 目标角度计算（含零点补偿） --- */
        yaw1_aim_angle = reset_xrimu_data_range(gimbal_cmd_recv.yaw + begin_xrimu_yaw);
        pitch_aim_angle = reset_xrimu_data_range(gimbal_cmd_recv.pitch + begin_xrimu_pitch);

        /* --- yaw2 电机角度反馈 --- */
        yaw2_angle_feedback = yaw1_motor->measure.angle_single_round;

        /* --- yaw2→yaw1 耦合前馈补偿 --- */
        yaw1_speed_feedforward = err_feedback_yaw2_to_yaw1 * yaw2_to_yaw1_speed_ff_gain;//大 yaw2 给小 yaw1 的前馈

        /* --- yaw2→yaw1 二阶电流前馈（惯性力矩补偿）--- */
        //测试前先查看dmmotor_yaw2->measure.torque是否有数值，再调整参数
        yaw1_current_feedforward = dmmotor_yaw2->measure.torque * yaw2_to_yaw1_current_ff_gain;
        /* --- 测试变量(用于调试) --- */
        yaw1_state_error = yaw1_aim_angle - yaw1_motor->measure.angle_single_round; // yaw1 电机状态误差

        DJIMotorSetRef(yaw1_motor, yaw1_aim_angle);

        /* --- pitch 轴双模式控制 --- */
        if(gimbal_cmd_recv.flag_vision_mode==1)//哨兵模式
        {   
            /* 视觉模式：使用 IMU 反馈实现自稳控制 */
            DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
            DJIMotorSetRef(pitch_motor, pitch_aim_angle);
        }
        else//非哨兵模式
        {
            /* 遥控模式：使用编码器反馈，带死区判断防止抖动 */
            DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, MOTOR_FEED);
            err_remote_pitch=gimbal_cmd_recv.pitch - pitch_motor->measure.angle_single_round;
            if(err_remote_pitch<=60.0 && err_remote_pitch>=-60.0)
            {remote_pitch_contral_flage=1;}  // 误差在±60°内，允许遥控
            else
            {remote_pitch_contral_flage=0;}  // 误差过大，锁定遥控

            if(remote_pitch_contral_flage)
            {
                DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch-360.0f);
            }
            else
            {
                DJIMotorSetRef(pitch_motor, pitch_motor->measure.angle_single_round);
            } 
        }
        
        break;
        
        /* ==================== 模式 C: 自由模式（已废弃） ==================== */
        // 云台自由模式，使用编码器反馈，底盘和云台分离，仅云台旋转，一般用于调整云台姿态 (英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除，或加入云台追地盘的跟随模式 (响应速度更快)
        // DJIMotorEnable(yaw1_motor);
        // DJIMotorEnable(pitch_motor);
        // // DMMotorEnable(dmmotor_yaw2);

        // DMMotorStop(dmmotor_yaw2);
        // yaw1_aim_angle = reset_xrimu_data_range(gimbal_cmd_recv.yaw + begin_xrimu_yaw);
        // DJIMotorSetRef(yaw1_motor, yaw1_aim_angle);
        // // pitch_aim_angle = reset_xrimu_data_range(gimbal_cmd_recv.pitch + begin_xrimu_pitch);
        // // DJIMotorSetRef(pitch_motor, pitch_aim_angle); 
        // //遥控器模式
        // DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
        
    default:
        /* 未知模式，安全停机 */
        DMMotorStop(dmmotor_yaw2);
        DJIMotorStop(yaw1_motor);
        DJIMotorStop(pitch_motor);
        break;
    }
    
    /* ==================== 预留功能：pitch 重力补偿前馈 ==================== */
    // 在合适的地方添加 pitch 重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    /* ==================== 步骤 3: 状态反馈发布 ==================== */ 
    //gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = dmmotor_yaw2->measure.position;
    ////////////////////////////车的朝向

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
    // vision_send_data.robotPitch=dmmotor_pitch->measure.position;//传电机的 pitch 位置
    // vision_send_data.robotYaw=INS.Yaw+dmmotor_yaw1->measure.position;//传 yaw 位置
    ///////////////////////////////////////
    // shootData[0]=dmmotor_pitch->measure.position;
    // shootData[1]=dmmotor_pitch->measure.torque;
    // vofa_justfloat_output(shootData, 2, &huart10);

    // cmdData[0] = yaw1_aim_angle;
    // cmdData[1] = imu_data.eulr.yaw;cmdData[2] = gimbal_cmd_recv.pitch;
    
    // cmdData[3] = imu_data.eulr.pitch;
    // vofa_justfloat_output(cmdData, 4 , &huart7);
}
