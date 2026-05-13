/*
 * imu.h
 *
 *  Created on: 6 May 2026
 *      Author: adria
 */

#ifndef IMU_H_
#define IMU_H_

#include <ti/drivers/I2C.h>
#include <stdint.h>
#include <stdbool.h>

// IMU Specific Macros extracted from main.c
#define IMU_ADDR      0x68
#define ACCEL_XOUT_H  0x3B
#define GYRO_CONFIG   0x1B
#define FS_SEL        1     // +- 500 deg/s
#define ACCEL_CONFIG  0x1C
#define AFS_SEL       1     // +- 4g

// Scaling factors
constexpr float ACC_CTT = 16384.0f / 2.0f;
constexpr float GY_CTT = 131.0f / 2.0f;

// IMU Data Structure
typedef struct {
    int16_t accX, accY, accZ;
    //int16_t temperature;
    int16_t gyX, gyY, gyZ;
} IMU_data;

class IMU {
public:
    IMU(I2C_Handle handle);

    // Initializes the IMU (wakes it up and sets scaling)
    bool init();

    // Reads the latest data from the IMU
    bool read();

    // Returns the latest parsed data
    IMU_data getData() const;

private:
    I2C_Handle i2cHandle;
    I2C_Transaction i2cTransaction;

    IMU_data data;

    uint8_t txBuffer[1];
    uint8_t rxBuffer[14];
};

#endif /* IMU_H_ */
