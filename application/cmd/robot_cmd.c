// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "main.h"
#include "et_remote.h"
#include <string.h>

#include "xrobot_imu_driver.h"

#include "vofa.h"

/*
 * 本文件负责机器人高层控制指令的组织与分发：
 * 1. 汇总遥控、视觉和各执行模块反馈
 * 2. 根据当前模式生成底盘、云台和发射机构控制量
 * 3. 将状态回传给视觉侧，并统一下发控制指令
 */

extern DJIMotorInstance *yaw1_motor;         // yaw 电机实例
extern DJIMotorInstance *pitch_motor;        // pitch 电机实例
extern float yaw1_motor_zero_position;       // yaw 电机机械零点
extern float begin_xrimu_yaw;                // 模式切换时记录的 yaw 参考值
extern float begin_xrimu_pitch;              // 模式切换时记录的 pitch 参考值

//uint8_t flag_vision_mode = 0;                // 0: 非视觉模式, 1: 视觉模式

// 编码器零点换算得到的机械参考角
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI)
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI)

/* cmd 应用内的通信对象与共享数据 */
#ifdef GIMBAL_BOARD
#include "can_comm.h"
static CANCommInstance *cmd_can_comm;        // 双板通信句柄
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;         // 底盘控制发布者
static Subscriber_t *chassis_feed_sub;       // 底盘反馈订阅者
#endif

Chassis_Ctrl_Cmd_s chassis_cmd_send;         // 下发到底盘的控制指令
Chassis_Upload_Data_s chassis_fetch_data;    // 底盘反馈数据

static RC_ctrl_t *rc_data;                   // 通用遥控数据入口
static RC_ctrl_t trans_rc_data;              // 天地飞遥控转存结构
static ETRC_Ctrl_s *et_rc_data;              // 天地飞遥控原始数据
Vision_Recv_s *vision_recv_data;             // 视觉接收数据
Vision_Send_s vision_send_data;              // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;          // 云台控制发布者
static Subscriber_t *gimbal_feed_sub;        // 云台反馈订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;    // 云台控制指令
static Gimbal_Upload_Data_s gimbal_fetch_data; // 云台反馈数据

static Publisher_t *shoot_cmd_pub;           // 发射控制发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 发射控制指令
static Shoot_Upload_Data_s shoot_fetch_data; // 发射反馈数据

static Subscriber_t *x_imu_sub;              // xrobot_imu 数据订阅者
static ImuData imu_data;                     // 来自 xrobot_imu 的 IMU 数据（消息订阅获取）

static Robot_Status_e robot_state;           // 机器人整体工作状态

BMI088Instance *bmi088_test;                 // 云台 IMU 实例
BMI088_Data_t bmi088_data;

uint8_t stop_flag = 0;
uint8_t image_flag = 0;

float yaw2_to_imu_position = 0.0f;

float error_yaw = 0;
float error_pitch = 0;

/**
 * @brief 将角度约束到 [-180, 180]，用于处理跨圈误差。
 */
float rerange_imu_data_range(float data)
{
    if (data > 180) {
        data = data - 360;
    } else if (data < -180) {
        data = data + 360;
    }
    return data;
}

/**
 * @brief 将天地飞遥控数据转成通用 RC_ctrl_t 结构。
 *
 * 后续控制逻辑统一从 rc_data 取值，这里负责做字段映射。
 */
void et_rc_data_transform(ETRC_Ctrl_s *et_rc_data)
{
    trans_rc_data.rc.rocker_l_ = et_rc_data->rocker_l_;
    trans_rc_data.rc.rocker_l1 = et_rc_data->rocker_l1;
    trans_rc_data.rc.rocker_r_ = et_rc_data->rocker_r_;
    trans_rc_data.rc.rocker_r1 = et_rc_data->rocker_r1;
    trans_rc_data.rc.switch_left = et_rc_data->switch_left_3;
    trans_rc_data.rc.switch_right = et_rc_data->switch_right_3;
}

/**
 * @brief 初始化输入源，并注册本模块与各执行模块之间的消息通道。
 */
void RobotCMDInit()
{
    // 当前工程使用天地飞遥控，保留 DBUS 初始化接口仅作兼容参考。
    //rc_data = RemoteControlInit(&huart5);
    vision_recv_data = VisionInit(&huart9);
    et_rc_data = ETRemoteInit(&huart5);
    rc_data = &trans_rc_data;

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    x_imu_sub = SubRegister("x_imu_feed", sizeof(ImuData)); // 订阅来自 INS_Task 的 IMU 数据
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x06,
            .rx_id = 0x05,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif

    gimbal_cmd_send.pitch = 0;
    robot_state = ROBOT_READY;
}

/**
 * @brief 将弧度转换为角度，并归一化到 [-180, 180]。
 */
float radian_to_degree_180(float radian)
{
    // 先做单位换算，再归一化，避免跨圈时出现角度跳变。
    float degree = radian * 180.0f / 3.1415926f;

    degree = fmodf(degree + 180.0f, 360.0f);
    if (degree < 0) {
        degree += 360.0f;
    }
    degree -= 180.0f;

    return degree;
}

/**
 * @brief 根据云台 yaw 单圈角反馈，计算底盘相对云台前向的偏置角。
 */
static void CalcOffsetAngle()
{
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle;

#if YAW_ECD_GREATER_THAN_4096
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE) {
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    } else if (angle > 180.0f + YAW_ALIGN_ANGLE) {
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    } else {
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    }
#else
    angle = radian_to_degree_180(angle);
    chassis_cmd_send.offset_angle = -angle;
#endif
}

/**
 * @brief 遥控模式下生成底盘、云台和发射机构控制量。
 *
 * 右侧三档决定底盘跟随关系；左摇杆控制云台，右摇杆控制底盘平移。
 */
static void RemoteControlSet()
{
    if (rc_data[TEMP].rc.switch_right == 1) {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    } else if (rc_data[TEMP].rc.switch_right == 2) {
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_2;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    } else if (rc_data[TEMP].rc.switch_right == 3) {
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    }

    // 视觉模式下，这里生成的部分控制量可能会被 SentinelSet() 覆盖。
    if ((rc_data[TEMP].rc.rocker_l_ >= 10 || rc_data[TEMP].rc.rocker_l_ <= -10 ||
         rc_data[TEMP].rc.rocker_l1 >= 10 || rc_data[TEMP].rc.rocker_l1 <= -10 ||
         et_rc_data->switch_right_2 == 1) &&
        gimbal_cmd_send.gimbal_mode != GIMBAL_ZERO_FORCE) {
        if (et_rc_data->switch_left_3 != 1) {
            gimbal_cmd_send.yaw -= 0.002f * (float)rc_data[TEMP].rc.rocker_l_;
            gimbal_cmd_send.pitch = -0.023485f * (float)rc_data[TEMP].rc.rocker_l1 + 247.5f;

            if (gimbal_cmd_send.yaw >= 180) {
                gimbal_cmd_send.yaw -= 360;
            } else if (gimbal_cmd_send.yaw <= -180) {
                gimbal_cmd_send.yaw += 360;
            }

            chassis_cmd_send.vx = 30.0f * (float)rc_data[TEMP].rc.rocker_r_;
            chassis_cmd_send.vy = -30.0f * (float)rc_data[TEMP].rc.rocker_r1;

            if (et_rc_data->switch_left_3 == 2 && shoot_cmd_send.friction_mode == FRICTION_ON) {
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            } else {
                shoot_cmd_send.load_mode = LOAD_STOP;
            }
        }
    }

    // 发射总开关默认打开，是否真正出弹由摩擦轮和 load_mode 共同决定。
    shoot_cmd_send.shoot_mode = SHOOT_ON;

    if (et_rc_data->switch_left_2 == 2) {
        shoot_cmd_send.friction_mode = FRICTION_ON;
    } else {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    }

    shoot_cmd_send.shoot_rate = 30;
}

int time_count = 100;
int shoot_continue_time_count = 5;

/**
 * @brief 视觉模式逻辑：跟踪、搜索、导航与开火控制。
 */
static void SentinelSet()
{
    static float pitch_search_angle = 0.1f;

    // 视觉模式优先使用识别结果控制云台，丢失目标后切换为扫描搜索。
    if (vision_recv_data->tracking == 1) {
        error_yaw = rerange_imu_data_range(vision_recv_data->aimYaw - imu_data.eulr.yaw);
        error_pitch = rerange_imu_data_range(vision_recv_data->aimPitch - imu_data.eulr.pitch);

        // 角度保护，防止目标跳变导致机构突转。
        if (error_yaw <= 45 && error_yaw >= -45 && error_pitch <= 20 && error_pitch >= -20) {
            gimbal_cmd_send.yaw = vision_recv_data->aimYaw - begin_xrimu_yaw;
            gimbal_cmd_send.pitch = vision_recv_data->aimPitch - begin_xrimu_pitch;
        }
        time_count = 0;
    } else if (time_count > 50) {
        if (chassis_fetch_data.game_process == 4) {
            gimbal_cmd_send.yaw += 0.4f;
            gimbal_cmd_send.pitch += pitch_search_angle;

            if (gimbal_cmd_send.yaw >= 180) {
                gimbal_cmd_send.yaw -= 360;
            } else if (gimbal_cmd_send.yaw <= -180) {
                gimbal_cmd_send.yaw += 360;
            }

            if (pitch_motor->measure.angle_single_round > 260) {
                pitch_search_angle = -0.17f;
            } else if (pitch_motor->measure.angle_single_round < 241) {
                pitch_search_angle = 0.17f;
            }
        }
    } else {
        time_count++;
    }

    // 将视觉给出的导航量映射为底盘平移速度。
    chassis_cmd_send.vy = -10000.0f * vision_recv_data->vx;
    chassis_cmd_send.vx = 10000.0f * vision_recv_data->vy;

    if (chassis_fetch_data.game_process == 4) {
        switch (chassis_fetch_data.self_position) {
        case 0:
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_2;
            break;
        case 1:
            chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
        case 2:
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_4;
            break;
        default:
            break;
        }
    }

    if (vision_recv_data->fire == 1 && shoot_cmd_send.friction_mode == FRICTION_ON) {
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        shoot_continue_time_count = 0;
    } else if (shoot_continue_time_count < 5) {
        // 给拨弹机构保留一个短暂续转时间。
        shoot_continue_time_count++;
    } else {
        shoot_cmd_send.load_mode = LOAD_STOP;
    }

    // 清零一次性导航输入，避免视觉数据断更后沿用旧速度。
    vision_recv_data->vx = 0;
    vision_recv_data->vy = 0;
}

/**
 * @brief 急停与异常保护入口。
 *
 * 当前函数预留为空，后续可在这里统一处理遥控失联、
 * 关键模块离线、双板通信超时和人工急停等保护逻辑。
 */
static void EmergencyHandler()
{

}

/* 主控制任务：读取反馈、解析模式、生成指令并统一下发。 */
void RobotCMDTask()
{
    // 先同步 ET 遥控数据到通用遥控结构，便于复用现有控制流程。
    et_rc_data_transform(et_rc_data);
    SubGetMessage(x_imu_sub, &imu_data); // 获取最新 IMU 姿态数据

#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    CalcOffsetAngle(); // 更新底盘与云台的相对朝向

    if (et_rc_data->switch_right_2 == 1) {
        RemoteControlSet();
        if (et_rc_data->switch_left_3 == 1) {
            SentinelSet();
            gimbal_cmd_send.flag_vision_mode = 1;
        } else {
            gimbal_cmd_send.flag_vision_mode = 0;
            begin_xrimu_pitch = imu_data.eulr.roll - gimbal_cmd_send.pitch;
        }
    }

    EmergencyHandler();

    // 失能或急停时，强制所有机构进入安全状态。
    if (et_rc_data->switch_right_2 != 1 || stop_flag) {
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
    }

    // 组织视觉回传数据，包括敌方颜色、弹速和姿态信息。
    VisionSetFlag(chassis_fetch_data.enemy_color, vision_send_data.mode, chassis_fetch_data.bullet_speed);
    yaw2_to_imu_position = rerange_imu_data_range(
        imu_data.eulr.yaw + yaw1_motor->measure.angle_single_round - yaw1_motor_zero_position);
    VisionSetAltitude(imu_data.eulr.yaw, imu_data.eulr.pitch, yaw2_to_imu_position);

    shoot_cmd_send.rest_heat = chassis_fetch_data.rest_heat;

#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);

    VisionSend();
}
