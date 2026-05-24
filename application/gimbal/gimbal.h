#ifndef GIMBAL_H
#define GIMBAL_H

/**
 * @file gimbal.h
 * @brief 云台控制模块接口定义
 * 
 * @details 本模块负责机器人云台的姿态控制，包括：
 *          - yaw 轴（水平旋转）和 pitch 轴（俯仰）的电机控制
 *          - 支持多种控制模式：零力矩、陀螺仪反馈、自由模式
 *          - IMU姿态解算与电机反馈融合
 *          - 视觉/遥控双模式支持
 * 
 * @note 云台模块通过消息中心与 cmd 模块通信，不直接处理遥控输入
 */

/**
 * @brief 云台初始化函数
 * 
 * @details 初始化流程：
 *          1. 初始化 IMU 传感器获取姿态数据指针
 *          2. 配置并初始化 yaw1、pitch、yaw2 三个电机实例
 *          3. 注册消息发布者和订阅者
 *          4. 配置蜂鸣器告警装置
 * 
 * @note 该函数会被 RobotInit() 调用，无需手动执行
 */
void GimbalInit();

/**
 * @brief 云台控制任务主循环
 * 
 * @details 任务执行流程：
 *          1. 从消息总线订阅云台控制指令和底盘速度反馈
 *          2. 根据控制模式切换不同的控制策略：
 *             - GIMBAL_ZERO_FORCE: 停止模式，记录初始位置
 *             - GIMBAL_GYRO_MODE: 陀螺仪反馈模式（主要工作模式）
 *             - GIMBAL_FREE_MODE: 自由模式（待删除）
 *          3. 计算目标角度并设置电机参考值
 *          4. 执行在线检测，异常时触发声光报警
 *          5. 发布云台状态反馈给视觉和 cmd 模块
 * 
 * @note 该任务应在 FreeRTOS 中作为独立任务运行
 * @see GimbalInit()
 */
void GimbalTask();

#endif // GIMBAL_H