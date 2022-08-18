#ifndef __SR_LS_IMU_H
#define __SR_LS_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 注：
 * 1. 加速度值单位是g，角速度值单位是度/秒，姿态角度单位是度.
 * 2. 串口配置是默认波特率115200，1bit 起始位，8bit 数据，无校验位，1bit 停止位。
 * 3. Accel_Scale = 20. Rate_Scale = 1260. Angle_Scale = 360. Sensor_Scale = 65536.
 */

typedef struct {
    double Ax;                // Accel x axis
    double Ay;                // Accel y axis
    double Az;                // Accel z axis
    double Gx;                // Gyro x axis
    double Gy;                // Gyro y axis
    double Gz;                // Gyro z axis
    double Roll;              // Roll
    double Pitch;             // Pitch
    double Yaw;               // Yaw
    double Temp;              // Temperature
    long long raw_timestamp;  // timestamp, ms 模块原始时间戳
    long long timestamp;      // timestamp, ms 驱动层自增加时间戳
} lsimu_data_t;

typedef int32_t (*imu_handler)(lsimu_data_t* data);

/**
 * 打开IMU，初始化设备
 *
 * @param[in]	dev  : 设备名，如"/dev/ttyUSB0"
 * @param[in]	cb: IMU数据回调函数
 *
 * @return		0成功，其他失败
 */
int32_t imu_open(const char* dev, imu_handler cb);

/**
 * 关闭IMU设备
 *
 * @param[in]	None.
 *
 * @return		0成功，其他失败
 */
int32_t imu_close(void);

#ifdef __cplusplus
}
#endif

#endif /* __SR_LS_IMU_H */
