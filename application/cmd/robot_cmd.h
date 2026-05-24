#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

/**
 * @brief 机器人指令模块初始化。
 *
 * 完成遥控、视觉、底盘/云台/发射 pub-sub 通道，以及双板 CAN 通信的初始化。
 * 该函数通常由 RobotInit() 在系统启动阶段调用。
 */
void RobotCMDInit();

/**
 * @brief 机器人主控制任务。
 *
 * 该任务以 200Hz 周期运行，负责：
 * 1. 拉取底盘、云台、发射机构反馈
 * 2. 解析遥控/视觉输入
 * 3. 生成控制指令
 * 4. 执行急停保护
 * 5. 将指令发布到各执行模块
 */
void RobotCMDTask();

/**
 * @brief 将弧度转换为角度，并归一化到 [-180, 180]。
 * @param radian 输入弧度值
 * @return 归一化后的角度值
 */
float radian_to_degree_180(float radian);

#endif // !ROBOT_CMD_H
