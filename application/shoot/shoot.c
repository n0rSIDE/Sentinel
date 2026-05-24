#include "shoot.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "servo_motor.h"
#include "cmsis_os.h"
#include "vofa.h"
#include "robot_cmd.h"

/* ==================== 调试数据 ==================== */
float cmdData[4]= {0};              ///< VOFA+ 调试数据（摩擦轮转速、弹速等）

extern Chassis_Upload_Data_s chassis_fetch_data;  ///< 底盘状态数据（包含弹速信息）

/* ==================== 电机实例定义 ==================== */
/* 对于双发射机构的机器人，将下面的数据封装成结构体即可，生成两份 shoot 应用实例 */
static DJIMotorInstance *friction_l;   ///< 左摩擦轮电机实例（M3508）
static DJIMotorInstance *friction_r;   ///< 右摩擦轮电机实例（M3508）
static DJIMotorInstance *loader;       ///< 拨盘电机实例（M2006）

/* ==================== 消息通信相关 ==================== */
static Publisher_t *shoot_pub;                      ///< 发射状态发布者（向 cmd 反馈）
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;             ///< 来自 cmd 的发射控制指令
static Subscriber_t *shoot_sub;                     ///< cmd 控制指令订阅者
static Shoot_Upload_Data_s shoot_feedback_data;     ///< 回传给 cmd 的发射状态信息

/* ==================== 前馈控制变量 ==================== */
static float loader_current_forward = 0;            ///< 拨盘电机电流前馈量

/* ==================== 定时和冷却变量 ==================== */
// dwt 定时，计算冷却用
static float hibernate_time = 0;    ///< 上次触发发射指令的时间戳（ms）
static float dead_time = 0;         ///< 冷却时间/不响应期（ms），防止连续触发
static float load_angle_set = 0;    ///< 拨盘目标角度（用于单发/三连发模式）

// float shootData[2]= {0};

/* ==================== 摩擦轮速度监测 ==================== */
float L_speed = 0, R_speed= 0;      ///< 左右摩擦轮实际转速（用于调试输出）

/* ==================== 发射任务状态变量 ==================== */
int begin_flag=1;           ///< 初始化标志位（未使用）
float begin_angle=0;        ///< 初始角度（未使用）
int count=0;                ///< 计数器（未使用）
double angle=0.0f;          ///< 角度变量（未使用）
int turn_tranlastion_count=0;   ///< 反转计数（未使用）
uint8_t turn_tranlastion_flag=0;  ///< 反转允许标志（0:不能反转 1:可以反转）

static uint16_t loader_error_cnt = 0;   ///< 拨盘堵转错误计数

uint8_t LOAD_REVERSE_flag=0;            ///< 拨盘反转使能标志

uint8_t shoot_over_flag=0;              ///< 射击完成标志（未使用）

float now_shoot_rate=0.0f;              ///< 当前实际射速（考虑热量限制后的射速）

/**
 * @brief 发射机构初始化函数
 * 
 * @details 初始化流程：
 *          1. 摩擦轮电机配置与初始化（左右各一个 M3508）：
 *             - 左摩擦轮：CAN3 ID=1，速度环 + 电流环双闭环，反向
 *             - 右摩擦轮：CAN3 ID=8，速度环 + 电流环双闭环，正向
 *             - 速度 PID：Kp=20, Ki=5, Kd=0
 *             - 电流 PID：Kp=1, Ki=0, Kd=0
 *          
 *          2. 拨盘电机配置与初始化（M2006）：
 *             - CAN1 ID=3，角度环 + 速度环 + 电流环三闭环
 *             - 角度 PID：Kp=10, Ki=25, Kd=0（大 Ki 保证力矩线性度）
 *             - 速度 PID：Kp=1, Ki=0, Kd=0
 *             - 电流 PID：Kp=1, Ki=0, Kd=0
 *             - 初始化为速度环模式，防止上电乱转
 *          
 *          3. 消息中心注册：
 *             - 发布者："shoot_feed"（发射状态反馈）
 *             - 订阅者："shoot_cmd"（发射控制指令）
 * 
 * @note 摩擦轮采用速度环控制，拨盘根据模式切换角度环/速度环
 * @note 该函数会被 RobotInit() 调用，无需手动执行
 */
void ShootInit()
{
    // 左摩擦轮，两个 can3
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan3,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 5, // 1
                .Kd = 0,
                .Derivative_LPF_RC = 0.02,
                .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral | PID_DerivativeFilter,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 1, // 0.7
                .Ki = 0,   // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },

        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,

        },
        .motor_type = M3508,
    };
    friction_config.can_init_config.tx_id = 1; // 左摩擦轮
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 8; // 右摩擦轮，改 txid 和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    /* ==================== 拨盘电机初始化（M2006） ==================== */
    //拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 3,
            .rx_id = 3,
        },  
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹，需要较大的 I 值来保证输出力矩的线性度，否则出现接近拨出的力矩大幅下降
                .Kp = 10, // 10
                .Ki = 25,
                .Kd = 0,
                .MaxOut = 20000,
            },
            .speed_PID = {
                .Kp = 1, // 0
                .Ki = 0, // 0
                .Kd = 0,
                .Improve = PID_Integral_Limit | PID_ErrorHandle,
                .IntegralLimit = 200,
                .MaxOut = 20000,
            },
            .current_PID = {
                .Kp = 1, // 0
                .Ki = 0,   // 0
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 10000,
            },

            .current_feedforward_ptr = &loader_current_forward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成 SPEED_LOOP，让拨盘停在原地，防止拨盘上电时乱转
            .close_loop_type = ANGLE_AND_SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向

            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = M2006 // 使用 2006
    };
    loader = DJIMotorInit(&loader_config);

    /* ==================== 消息中心注册 ==================== */
    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* ==================== 发射核心控制任务 ==================== */
/**
 * @brief 机器人发射机构控制核心任务
 * 
 * @details 任务执行流程：
 * 
 *          1. **数据获取**：
 *             - 从消息总线订阅发射控制指令（shoot_cmd）
 *             - 包含射击模式、摩擦轮开关、弹速设定、拨盘模式、剩余热量等
 * 
 *          2. **紧急停止处理**：
 *             - 当 shoot_mode == SHOOT_OFF 时，立即停止所有电机
 *             - 用于紧急情况或系统关闭
 * 
 *          3. **摩擦轮控制**：
 *             a) 开启摩擦轮（FRICTION_ON）：
 *                - 根据目标弹速设置左右摩擦轮参考转速
 *                - 支持多档弹速：15m/s、18m/s、30m/s
 *                - 默认值 50000 RPM（调试用）
 *             
 *             b) 关闭摩擦轮：
 *                - 停止摩擦轮和拨盘电机
 * 
 *          4. **冷却时间检查**：
 *             - 如果当前时间 < 上次触发时间 + 冷却时间，直接返回
 *             - 防止单发/三连发模式下连续触发
 * 
 *          5. **堵转检测和保护**：
 *             - 监测拨盘电流 >= 2500mA 时判定为堵转
 *             - 自动切换到 LOAD_REVERSE 模式退弹
 *             - 记录堵转次数，每 1000ms 允许一次反转
 * 
 *          6. **拨盘模式控制**：
 *             a) LOAD_STOP（停止）：
 *                - 切换到速度环，设置参考值为 0
 *             
 *             b) LOAD_1_BULLET（单发）：
 *                - 切换到角度环，拨盘旋转 1 颗弹丸的角度
 *                - 记录触发时间，设置 500ms 冷却时间
 *                - 用于能量机关激活、英雄点射
 *             
 *             c) LOAD_3_BULLET（三连发）：
 *                - 切换到角度环，拨盘旋转 3 颗弹丸的角度
 *                - 记录触发时间，设置 1800ms 冷却时间
 *             
 *             d) LOAD_BURSTFIRE（连发）：
 *                - 保持速度环，根据目标射速设置拨盘转速
 *                - 射速计算：考虑剩余热量限制
 *                - 公式：now_shoot_rate = shoot_rate - rest_heat × 0.1
 *             
 *             e) LOAD_REVERSE（反转）：
 *                - 切换到角度环，拨盘反向旋转 1 颗弹丸的角度
 *                - 记录触发时间，设置 100ms 冷却时间
 *                - 用于退弹或排除卡弹
 * 
 *          7. **热量管理**：
 *             - 当剩余热量 <= 220 时，允许正常发射
 *             - 当剩余热量 > 220 时，强制停止拨盘（保护机制）
 * 
 *          8. **状态反馈**：
 *             - 发布发射状态到消息总线（目前为空数据结构）
 * 
 *          9. **调试输出**：
 *             - VOFA+ 输出左右摩擦轮转速、目标转速、底盘弹速
 * 
 * @note 摩擦轮始终工作在速度环模式
 * @note 拨盘根据模式动态切换角度环/速度环
 * @note 单发和三连发模式通过 dead_time 防止连续触发
 * @todo 增加应用离线监测和卡弹反馈功能
 */
void ShootTask()
{   
    /* ==================== 步骤 1: 获取发射控制数据 ==================== */
    // 从 cmd 获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    
    /* ==================== 步骤 2: 紧急停止处理 ==================== */
    // 对 shoot mode 等于 SHOOT_STOP 的情况特殊处理，直接停止所有电机 (紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
         DJIMotorStop(loader);
         DJIMotorStop(friction_l);
         DJIMotorStop(friction_r);
    }
    else // 恢复运行
    {
        //DMMotorEnable(dmmotor_loader);
        DJIMotorEnable(loader);
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
    
        /* ==================== 步骤 3: 摩擦轮控制 ==================== */
        if (shoot_cmd_recv.friction_mode == FRICTION_ON )
        {
            /* --- 根据目标弹速设定摩擦轮参考转速 --- */
            switch (shoot_cmd_recv.bullet_speed)
            {
            case SMALL_AMU_15:
                DJIMotorSetRef(friction_l, 26800);
                DJIMotorSetRef(friction_r, 26800);
                break;
            case SMALL_AMU_18:
                DJIMotorSetRef(friction_l, 30000);
                DJIMotorSetRef(friction_r, 30000);
                break;
            case SMALL_AMU_30:
                DJIMotorSetRef(friction_l, 46500);
                DJIMotorSetRef(friction_r, 46500);
                break;
            default: // 当前为了调试设定的默认值 40000，因为还没有加入裁判系统无法读取弹速.
                 // DJIMotorSetRef(friction_l, 26800);
                 // DJIMotorSetRef(friction_r, 26800);
                DJIMotorSetRef(friction_l, 50000);
                DJIMotorSetRef(friction_r, 50000);
                break;
            }
            DJIMotorSetRef(friction_l, 35000);
            DJIMotorSetRef(friction_r, 35000);
        }
        else // 关闭摩擦轮/
        {
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            DJIMotorSetRef(loader, 0);
            DJIMotorStop(loader);
            DJIMotorStop(friction_l);
            DJIMotorStop(friction_r);
        }

        /* ==================== 步骤 4: 冷却时间检查 ==================== */
        //如果上一次触发单发或 3 发指令的时间加上不应期仍然大于当前时间 (尚未休眠完毕),直接返回即可
        //单发模式主要提供给能量机关激活使用 (以及英雄的射击大部分处于单发)
        if (hibernate_time + dead_time > DWT_GetTimeline_ms())
            return;

        /* ==================== 步骤 5: 热量检查和堵转处理 ==================== */
        if (shoot_cmd_recv.rest_heat <= 220)
        {
            /* --- 堵转检测与自动反转 --- */
            if ((loader->measure.real_current >= 2500)  && LOAD_REVERSE_flag)
            {
                shoot_cmd_recv.load_mode = LOAD_REVERSE;
                LOAD_REVERSE_flag=0;
                loader_error_cnt++;  // 记录堵转次数
            // 超过堵转次数，恢复正转
            }
            else
            {
               static int i = 0;
               i++;
               if(i==1000)
               {
                    i=0;
                    LOAD_REVERSE_flag=1;  // 每 1000ms 允许一次反转
               }
            }
            
            /* ==================== 步骤 6: 拨盘模式控制 ==================== */
            switch (shoot_cmd_recv.load_mode)
            {
                /* --- 模式 A: 停止拨盘 --- */
                case LOAD_STOP:
                    DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
                    DJIMotorSetRef(loader, 0);
                    break;
                    
                /* --- 模式 B: 单发模式 --- */
                // 单发模式，根据鼠标按下的时间，触发一次之后需要进入不响应输入的状态 (否则按下的时间内可能多次进入，导致多次发射)
                case LOAD_1_BULLET: // 激活能量机关/干扰对方用，英雄用.
                    load_angle_set = loader->measure.total_angle + (360.f / NUM_PER_CIRCLE * REDUCTION_RATIO_LOADER);
                    DJIMotorOuterLoop(loader, ANGLE_LOOP);  // 切换到角度环
                    DJIMotorSetRef(loader, load_angle_set); // 控制量增加一发弹丸的角度
                    hibernate_time = DWT_GetTimeline_ms();  // 记录触发指令的时间
                    dead_time = 500;                        // 完成 1 发弹丸发射的时间
                    break;
                    
                /* --- 模式 C: 三连发模式 --- */
                // 三连发，如果不需要后续可能删除
                case LOAD_3_BULLET:
                    load_angle_set = loader->measure.total_angle + (360.f / NUM_PER_CIRCLE * REDUCTION_RATIO_LOADER * 3);
                    DJIMotorOuterLoop(loader, ANGLE_LOOP);  // 切换到角度环
                    DJIMotorSetRef(loader, load_angle_set); // 增加 3 发
                    hibernate_time = DWT_GetTimeline_ms();  // 记录触发指令的时间
                    dead_time = 1800;                       // 完成 3 发弹丸发射的时间
                    break;
                    
                /* --- 模式 D: 连发模式 --- */
                // 连发模式，对速度闭环，射频后续修改为可变，目前固定为 1Hz
                case LOAD_BURSTFIRE:
                    now_shoot_rate = shoot_cmd_recv.shoot_rate - shoot_cmd_recv.rest_heat*0.1;
                    if(now_shoot_rate<0){now_shoot_rate=0;}
                    DJIMotorOuterLoop(loader, SPEED_LOOP);
                    DJIMotorSetRef(loader, now_shoot_rate * (360.f * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE));
                    // x 颗/秒换算成速度：已知一圈的载弹量，由此计算出 1s 需要转的角度，注意换算角速度 (DJIMotor 的速度单位是 angle per second)
                    break;
                    
                /* --- 模式 E: 反转模式 --- */
                // 拨盘反转，对速度闭环，后续增加卡弹检测 (通过裁判系统剩余热量反馈和电机电流)
                // 也有可能需要从 switch-case 中独立出来
                case LOAD_REVERSE:
                    load_angle_set = loader->measure.total_angle - (360.f / NUM_PER_CIRCLE * REDUCTION_RATIO_LOADER); // Enable load angle set calculation
                    DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                          // Change back to angle loop for reverse loading
                    DJIMotorSetRef(loader, load_angle_set);
                    hibernate_time = DWT_GetTimeline_ms(); // 记录触发指令的时间
                    dead_time = 100;                       // 完成 1 发弹丸发射的时间
                    // ...
                    break;
                    
                default:
                    while (1)
                    ; // 未知模式，停止运行，检查指针越界，内存溢出等问题
            }
        }
        else
        {
           /* ==================== 步骤 7: 热量保护 - 强制停止 ==================== */
           DJIMotorStop(loader);
        }
        
        /* ==================== 步骤 8: 状态反馈发布 ==================== */
        // 反馈数据，目前暂时没有要设定的反馈数据，后续可能增加应用离线监测以及卡弹反馈
        PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
    }
    
    /* ==================== 步骤 9: VOFA+ 调试输出 ==================== */
    // L_speed =friction_l->measure.speed_aps , R_speed = friction_r->measure.speed_aps;
    // shootData[0] = -L_speed;
    // shootData[1] = R_speed;
    // shootData[0] = dmmotor_loader->measure.torque;
    // shootData[1] = angle;
    // vofa_justfloat_output(shootData, 2, &huart7);
    cmdData[0]=40000.0;
    cmdData[1]=-friction_l->measure.speed_aps;
    cmdData[2]=friction_r->measure.speed_aps;
    cmdData[3]=chassis_fetch_data.bullet_speed;
    vofa_justfloat_output(cmdData, 4 , &huart7);
}
