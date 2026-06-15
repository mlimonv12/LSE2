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

typedef struct {
    int32_t accX = 0, accY = 0, accZ = 0;
    //int16_t temperature;
    int32_t gyX = 0, gyY = 0, gyZ = 0;
} IMU_offset;

class IMU {
public:
    IMU();

    // Initializes the IMU (wakes it up and sets scaling)
    bool init(I2C_Handle i2cHandle);

    // Checks whether the swing is leaning right or left
    //bool checkRight();

    // Calibrates IMU
    float calibrate(int N);

    // Reads the latest data from the IMU
    bool read();

    // Returns the latest parsed data
    IMU_data getData() const;

private:
    I2C_Handle i2cHandle;
    I2C_Transaction i2cTransaction;


    IMU_data data;
    IMU_offset ofs;
    float restAngle;

    uint8_t txBuffer[1];
    uint8_t rxBuffer[14];
};

#endif /* IMU_H_ */
