#include "xrobot_imu_driver.h"
#include <math.h>
#include <string.h>
#include "bsp_can.h"
#include <stdio.h>
#include "message_center.h"

#define IMU_DEVICE_ID (0x30)

#define M_PI 3.14159265358979323846
#define M_2PI 6.28318530717958647692

/* CAN Packet IDs */
#define CAN_PACK_ID_ACCL 0
#define CAN_PACK_ID_GYRO 1
#define CAN_PACK_ID_EULR 3
#define CAN_PACK_ID_QUAT 4

/* Encoder constants */
#define ENCODER_21_MAX_INT ((1u << 21) - 1)

CANInstance* imu_can_instance1;
CANInstance* imu_can_instance2;
CANInstance* imu_can_instance3;
CANInstance* imu_can_instance4;

static Publisher_t *x_imu_pub; ///< IMU 数据发布者（向 gimbal 发送解算后的姿态数据）

typedef struct {
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[64];
} can_raw_rx_t;

/* CANFD Data Structure - matches C++ DataCanfd struct */
typedef struct __attribute__((packed)) {
  uint64_t time : 48;
  uint64_t sync : 48;
  float quat[4]; /* w, x, y, z */
  float gyro[3]; /* x, y, z */
  float accl[3]; /* x, y, z */
  float eulr[3]; /* pitch, roll, yaw */
} CanfdData;

/* CAN Classic Data Structures */
typedef union {
  struct __attribute__((packed)) {
    int32_t data1 : 21;
    int32_t data2 : 21;
    int32_t data3 : 21;
    int32_t res : 1;
  };
  struct __attribute__((packed)) {
    uint32_t data1_unsigned : 21;
    uint32_t data2_unsigned : 21;
    uint32_t data3_unsigned : 21;
    uint32_t res_unsigned : 1;
  };
  uint8_t raw[8];
} CanData3;

typedef struct __attribute__((packed)) {
  union {
    int16_t data[4];
    uint16_t data_unsigned[4];
  };
} CanData4;


/**
 * @brief Decode 21-bit unsigned integer to float
 * @param encoded 21-bit encoded value
 * @param min Minimum float value
 * @param max Maximum float value
 * @return Decoded float value
 */
static float DecodeFloat21(uint32_t encoded, float min, float max) {
  float norm =
      (float)(encoded & ENCODER_21_MAX_INT) / (float)ENCODER_21_MAX_INT;
  return min + norm * (max - min);
}

/**
 * @brief Decode int16 normalized value to float (for quaternion)
 * @param value int16 value
 * @return Normalized float [-1, 1]
 */
static float DecodeInt16Normalized(int16_t value) {
  return (float)value / (float)INT16_MAX;
}
can_raw_rx_t rx_buff;
ImuData imu_data;
uint32_t pack_count = 0;

/**
 * @brief Process CAN-FD packet
 */
static void ProcessCanfdPacket(uint8_t *data) {
  CanfdData *canfd_data = (CanfdData *)data;

  imu_data.timestamp = canfd_data->time;
  imu_data.sync_time = canfd_data->sync;

  /* Quaternion */
  imu_data.quat.w = canfd_data->quat[0];
  imu_data.quat.x = canfd_data->quat[1];
  imu_data.quat.y = canfd_data->quat[2];
  imu_data.quat.z = canfd_data->quat[3];

  /* Gyroscope */
  imu_data.gyro.x = canfd_data->gyro[0];
  imu_data.gyro.y = canfd_data->gyro[1];
  imu_data.gyro.z = canfd_data->gyro[2];

  /* Accelerometer */
  imu_data.accl.x = canfd_data->accl[0];
  imu_data.accl.y = canfd_data->accl[1];
  imu_data.accl.z = canfd_data->accl[2];

  /* Euler angles */
  imu_data.eulr.pitch = canfd_data->eulr[0];
  imu_data.eulr.roll = canfd_data->eulr[1];
  imu_data.eulr.yaw = canfd_data->eulr[2];
}

/**
 * @brief Process Classic CAN packet
 */
static void ProcessClassicCanPacket(uint32_t id, uint8_t *data) {
  uint32_t packet_type = id - IMU_DEVICE_ID;

  switch (packet_type) {
  case CAN_PACK_ID_ACCL: {
    /* Accelerometer data: ±24g range */
    CanData3 *can_data = (CanData3 *)data;
    imu_data.accl.x = DecodeFloat21(can_data->data1_unsigned, -24.0f, 24.0f);
    imu_data.accl.y = DecodeFloat21(can_data->data2_unsigned, -24.0f, 24.0f);
    imu_data.accl.z = DecodeFloat21(can_data->data3_unsigned, -24.0f, 24.0f);
    break;
  }

  case CAN_PACK_ID_GYRO: {
    /* Gyroscope data: ±2000 deg/s converted to rad/s */
    CanData3 *can_data = (CanData3 *)data;
    float min_gyro = -2000.0f * M_PI / 180.0f;
    float max_gyro = 2000.0f * M_PI / 180.0f;
    imu_data.gyro.x =
        DecodeFloat21(can_data->data1_unsigned, min_gyro, max_gyro);
    imu_data.gyro.y =
        DecodeFloat21(can_data->data2_unsigned, min_gyro, max_gyro);
    imu_data.gyro.z =
        DecodeFloat21(can_data->data3_unsigned, min_gyro, max_gyro);
    break;
  }

  case CAN_PACK_ID_EULR: {
    /* Euler angles: ±π rad */
    CanData3 *can_data = (CanData3 *)data;
    imu_data.eulr.pitch = DecodeFloat21(can_data->data1_unsigned, -M_PI, M_PI)/M_PI*180;
    imu_data.eulr.roll = DecodeFloat21(can_data->data2_unsigned, -M_PI, M_PI)/M_PI*180;
    imu_data.eulr.yaw = DecodeFloat21(can_data->data3_unsigned, -M_PI, M_PI)/M_PI*180;
    break;
  }

  case CAN_PACK_ID_QUAT: {
    /* Quaternion data: normalized int16 */
    CanData4 *can_data = (CanData4 *)data;
    imu_data.quat.w = DecodeInt16Normalized(can_data->data[0]);
    imu_data.quat.x = DecodeInt16Normalized(can_data->data[1]);
    imu_data.quat.y = DecodeInt16Normalized(can_data->data[2]);
    imu_data.quat.z = DecodeInt16Normalized(can_data->data[3]);
    break;
  }

  default:
    /* Unknown packet type */
    break;
  }
}
void imu_classic_can_receive_callback(CANInstance* instance) {
  ProcessClassicCanPacket(instance->rx_id, instance->rx_buff);
}
//注册消息与发布消息接口，在INS里面调用
void IMU_PubInit(void)
{
    x_imu_pub = PubRegister("x_imu_feed", sizeof(ImuData));
}

void IMU_PublishData(void)
{
    if (x_imu_pub != NULL)
        PubPushMessage(x_imu_pub, (void *)&imu_data);
}

void register_imu_can_receiver(FDCAN_HandleTypeDef* fdcan_id ) {

    // 定义CAN初始化配置结构体
    CAN_Init_Config_s imu_can_config1 = {
        .can_handle = fdcan_id,                
        .tx_id = 0x30,                         
        .rx_id = 0x30,                        
        .can_module_callback = imu_classic_can_receive_callback,
        .id = NULL                  
    };
    imu_can_instance1 = CANRegister(&imu_can_config1);
    CAN_Init_Config_s imu_can_config2 = {
        .can_handle = fdcan_id,                
        .tx_id = 0x31,                         
        .rx_id = 0x31,                        
        .can_module_callback = imu_classic_can_receive_callback,
        .id = NULL                  
    };
    imu_can_instance2 = CANRegister(&imu_can_config2);
    CAN_Init_Config_s imu_can_config3 = {
        .can_handle = fdcan_id,                
        .tx_id = 0x33,                         
        .rx_id = 0x33,                        
        .can_module_callback = imu_classic_can_receive_callback,
        .id = NULL                  
    };
    imu_can_instance3 = CANRegister(&imu_can_config3);
    CAN_Init_Config_s imu_can_config4 = {
        .can_handle = fdcan_id,                
        .tx_id = 0x34,                         
        .rx_id = 0x34,                        
        .can_module_callback = imu_classic_can_receive_callback,
        .id = NULL                  
    };
    imu_can_instance4 = CANRegister(&imu_can_config4);
    
    if (imu_can_instance1 != NULL && imu_can_instance2 != NULL && imu_can_instance3 != NULL && imu_can_instance4 != NULL) {
        printf("IMU CAN receiver registered successfully\n");
    } else {
        printf("Failed to register IMU CAN receiver\n");
    }
}


