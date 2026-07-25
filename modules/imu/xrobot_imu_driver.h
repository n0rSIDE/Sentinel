
#ifndef __XROBOT_IMU_DRIVER_H
#define __XROBOT_IMU_DRIVER_H

#include "main.h"

typedef struct __attribute__((packed)) {
  float x;
  float y;
  float z;
} Vector3;

typedef struct __attribute__((packed)) {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

typedef struct __attribute__((packed)) {
  float yaw;
  float pit;
  float rol;
} EulerAngles;

typedef struct __attribute__((packed)) {
  uint64_t time : 48;
  uint64_t sync : 48;
  Quaternion quat_;
  Vector3 gyro_;
  Vector3 accl_;
  EulerAngles eulr_;
} Data;

/* Decoded IMU Data */
typedef struct {
  struct {
    float x, y, z;
  } accl;
  struct {
    float x, y, z;
  } gyro;
  struct {
    float pitch, roll, yaw;
  } eulr;
  struct {
    float w, x, y, z;
  } quat;
  uint64_t timestamp;
  uint64_t sync_time;
} ImuData;

void register_imu_can_receiver(FDCAN_HandleTypeDef* fdcan_id);

/**
 * @brief 初始化 IMU 数据发布者（由 INS_Init 调用，仅执行一次）
 */
void IMU_PubInit(void);

/**
 * @brief 发布最新的 IMU 数据到消息总线（由 INS_Task 以 1kHz 调用）
 * @note 必须在 RTOS 任务上下文中调用，不能在 ISR 中调用
 */
void IMU_PublishData(void);

#endif