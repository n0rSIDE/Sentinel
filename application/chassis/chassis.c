/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"

#include "user_lib.h"
#include "bsp_rng.h"
#include "slope.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

/* 超级电筒相关 */
static SuperCapInstance *cap;        // 超级电容
static SuperCap_Ctrl_s supercap_msg; // 超级电容控制信息

/* 底盘运动相关 */
static float chassis_SPEED_PID[3] = {5.5, 0, 0};
static float chassis_CURRENT_PID[3] = {0.8, 0, 0};
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
static Slope_s w_slope;                                             // 自旋变速斜坡

/* 裁判系统相关数据 */
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

/* 用于底盘控制的超电增益系数 */
static float chassis_move_buff = 1, chassis_rotate_buff = 1; // 底盘运动和旋转的增益系数
static float chassis_move_buff_list[2] = {1.0, 0.7};         // 底盘运动增益系数列表, buff_h, buff_l
static float chassis_rotate_buff_list[2] = {1.0, 0.8};       // 底盘自旋增益系数列表, buff_h, buff_l
static float real_wz;                                        // 实际底盘自旋速度

/* 用于自旋变速策略变量,使用查表加速，后续考虑直接读取RNG值 */
static float rotate_speed = 0;                                                    // 上一次的旋转速度
static float pos_variable_rotate_speed[8] = {3.6, 3.8, 4, 4.2, 4.4, 4.6, 4.8, 5}; // 正向旋转速度变量

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

static float rotate_speed_buff = 0;
void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = chassis_SPEED_PID[0], // 4.5,
                .Ki = chassis_SPEED_PID[1], // 0
                .Kd = chassis_SPEED_PID[2], // 0
                .IntegralLimit = 3000,
                .Output_LPF_RC = 0.02,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = chassis_CURRENT_PID[0], // 0.4
                .Ki = chassis_CURRENT_PID[1], // 0
                .Kd = chassis_CURRENT_PID[2],
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    /**
     * 步兵一
     * 4 1 3 2
     * 步兵二
     * 1 2 4 3
     */
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_param_init_config.speed_PID.Kp = 7.8;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_param_init_config.speed_PID.Kp = 7.8;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    SlopeInit(&w_slope, 30, 40); // 自旋变速斜坡初始化

    // referee_data = Referee_Interactive_init(&huart6, &ui_data); // 裁判系统初始化

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x061, // 超级电容默认接收id通信can1
            .rx_id = 0x051, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // ww超级电容初始化
    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    // Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = chassis_vx + chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy + chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy + chassis_cmd_recv.wz * RB_CENTER;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    static float chassis_power_buff = 1;

    supercap_msg.feedbackRefereePowerLimit = referee_data->GameRobotState.chassis_power_limit;
    // supercap_msg.feedbackRefereePowerLimit = 10;
    supercap_msg.feedbackRefereeEnergyBuffer = referee_data->PowerHeatData.chassis_power_buffer;
    SuperCapSend(cap, &supercap_msg);

    // 运动速率限制
    switch (referee_data->GameRobotState.chassis_power_limit)
    {
    // 烧饼暂时只有100w，只留一个case
    case 100:
        rotate_speed_buff = 2.4;
        chassis_power_buff = 1;
        break;
    default:
        rotate_speed_buff = 2.4;
        chassis_power_buff = 1;
        break;
    }

    if (referee_data->GameState.game_progress == 0)
    {
        DJIMotorSetRef(motor_lf, vt_lf * chassis_power_buff);
        DJIMotorSetRef(motor_rf, vt_rf * chassis_power_buff);
        DJIMotorSetRef(motor_lb, vt_lb * chassis_power_buff);
        DJIMotorSetRef(motor_rb, vt_rb * chassis_power_buff);
    }
    else if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE_2)
    {
        DJIMotorSetRef(motor_lf, vt_lf * chassis_power_buff * 0.85);
        DJIMotorSetRef(motor_rf, vt_rf * chassis_power_buff * 0.85);
        DJIMotorSetRef(motor_lb, vt_lb * chassis_power_buff * 0.85);
        DJIMotorSetRef(motor_rb, vt_rb * chassis_power_buff * 0.85);
    }
    else
    {
        DJIMotorSetRef(motor_lf, vt_lf * chassis_power_buff);
        DJIMotorSetRef(motor_rf, vt_rf * chassis_power_buff);
        DJIMotorSetRef(motor_lb, vt_lb * chassis_power_buff);
        DJIMotorSetRef(motor_rb, vt_rb * chassis_power_buff);
    }
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // °/s
    real_wz = (motor_lf->measure.speed_aps + motor_lb->measure.speed_aps +
               motor_rb->measure.speed_aps + motor_rf->measure.speed_aps) /
              4 / REDUCTION_RATIO_WHEEL / HALF_WHEEL_BASE * RADIUS_WHEEL;
}
/* 飞坡控制 */
static void Fly()
{
}

/* 烧饼决策树相关处理 */
#if (DECISION_TREE_ACTIVE == 1)
static void ChassisDecisionTreeProcecss()
{
    // /* 超电能量规划 */
    // switch (chassis_cmd_recv.transfer_state)
    // {
    // case MOVE_OCCUPY: // 占点，优先小陀螺
    //     if (cap->cap_msg.capEnergy > UINT8_MAX * 0.30f)
    //     {
    //         /* vh,wh */
    //         chassis_move_buff = chassis_move_buff_list[0];
    //         chassis_rotate_buff = chassis_rotate_buff_list[0];
    //     }
    //     else
    //     {
    //         /* vl,wh */
    //         chassis_move_buff = chassis_move_buff_list[1];
    //         chassis_rotate_buff = chassis_rotate_buff_list[0];
    //     }
    //     break;
    // case MOVE_TRANSFER: // 转移，优先移动
    //     if (cap->cap_msg.capEnergy > UINT8_MAX * 0.45f)
    //     {
    //         /* vh,wh */
    //         chassis_move_buff = chassis_move_buff_list[0];
    //         chassis_rotate_buff = chassis_rotate_buff_list[0];
    //     }
    //     else
    //     {
    //         /* vh,wl */
    //         chassis_move_buff = chassis_move_buff_list[0];
    //         chassis_rotate_buff = chassis_rotate_buff_list[1];

    //         if (cap->cap_msg.capEnergy < UINT8_MAX * 0.20f && chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE)
    //         {
    //             if (cap->cap_msg.capEnergy != 0) // 确保已经连接上超电
    //                 /* 超电消耗过多，转入跟头模式 */
    //                 chassis_cmd_recv.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    //         }
    //     }
    //     break;
    // }

    // /* 血量反馈 */
    // float hp_prop = (referee_data->GameRobotState.maximum_HP != 0)
    //                     ? (float)referee_data->GameRobotState.current_HP / (float)referee_data->GameRobotState.maximum_HP
    //                     : 0;
    // if (hp_prop > 0.95f)
    // {
    //     chassis_feedback_data.blood_warn_state = ROBOT_BLOOD_HEALTH;
    // }
    // else if (hp_prop > 0.50f)
    // {
    //     chassis_feedback_data.blood_warn_state = ROBOT_BLOOD_HIGH;
    // }
    // else if (hp_prop > 0.33f)
    // {
    //     chassis_feedback_data.blood_warn_state = ROBOT_BLOOD_LOW;
    // }
    // else
    // {
    //     chassis_feedback_data.blood_warn_state = ROBOT_BLOOD_DANGER;
    // }

    // /* 中心增益点占领状态、比赛阶段和本阶段剩余时间反馈 */
    // chassis_feedback_data.is_enemy_preempt = (referee_data->EventData.event_type & 0x600000) >> 21;
    // chassis_feedback_data.game_process = referee_data->GameState.game_progress;
    // chassis_feedback_data.current_state_left_time = referee_data->GameState.stage_remain_time;
}
#endif

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // BuzzerOn(); // 初始化后蜂鸣器响
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE_2 || chassis_cmd_recv.chassis_mode == CHASSIS_NO_FOLLOW || chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
        supercap_msg.enableDCDC = 1;
    }
    else
    { // 正常工作
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);

        // 超级电容过放保护
        if (cap->cap_msg.capEnergy <= 75) // 
        {
            supercap_msg.enableDCDC = 1; // 强制开启DCDC进行充电
        }
        else if (cap->cap_msg.capEnergy >= 128) // 15V左右
        {
            supercap_msg.enableDCDC = 0; // 允许关闭DCDC
        }
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
        chassis_cmd_recv.wz = chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle) * rotate_speed_buff * 0.1;
        VAL_LIMIT(chassis_cmd_recv.wz, -4000, 4000);
        break;
    case CHASSIS_ROTATE_2: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        static uint32_t cnt = 0;
        if ((cnt++) % 40 == 0)
        {
            rotate_speed = pos_variable_rotate_speed[GetRandomInt(0, 7)];
        }
        // chassis_cmd_recv.wz=1000*rotate_speed_buff;
        chassis_cmd_recv.wz = 250 * rotate_speed_buff * rotate_speed;
        // chassis_cmd_recv.wz=(2000+1000*abs(arm_cos_f32(time*DEGREE_2_RAD)))*rotate_speed_buff;
        break;
    default:
        break;
    }

    // 决策树处理
#if (DECISION_TREE_ACTIVE == 1)
    ChassisDecisionTreeProcecss();

    chassis_cmd_recv.vx *= chassis_move_buff;
    chassis_cmd_recv.vy *= chassis_move_buff;
    chassis_cmd_recv.wz *= chassis_rotate_buff;
#endif

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘***逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)*
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);

    chassis_vx = (chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta);
    chassis_vy = (chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta);

    chassis_cmd_recv.wz = SlopeUpdate(&w_slope, chassis_cmd_recv.wz,
                                      real_wz * REDUCTION_RATIO_WHEEL);

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
    LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色, 0:red , 1:blue
    // chassis_feedback_data.self_color = referee_data->GameRobotState.robot_id > 7 ? COLOR_BLUE : COLOR_RED;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->ShootData.bullet_speed;
    // chassis_feedback_data.rest_heat = (int16_t)((int32_t)referee_data->GameRobotState.shooter_barrel_heat_limit -
    //                                             (int32_t)referee_data->PowerHeatData.shooter_heat0);

    // ui_data.chassis_mode = chassis_cmd_recv.chassis_mode;
    // ui_data.friction_mode = chassis_cmd_recv.friction_mode;
    // ui_data.lid_mode = chassis_cmd_recv.lid_mode;
    // ui_data.shoot_mode = chassis_cmd_recv.load_mode;
    // ui_data.ui_mode = chassis_cmd_recv.ui_mode;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}