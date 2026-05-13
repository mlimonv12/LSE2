/*
 * imu.c
 *
 *  Created on: 6 May 2026
 *      Author: adria
 */


#include "imu.h"
#include <cstddef> // for NULL

IMU::IMU(I2C_Handle handle) : i2cHandle(handle) {
    // Initialize the data structure to 0
    data = {0, 0, 0, 0, 0, 0};
}

bool IMU::init() {
    if (!i2cHandle) {
        return false;
    }

    uint8_t wakeTxBuffer[2];
    I2C_Transaction wakeTransaction;

    wakeTransaction.slaveAddress = IMU_ADDR;
    wakeTransaction.writeBuf = wakeTxBuffer;
    wakeTransaction.writeCount = 2;
    wakeTransaction.readBuf = NULL;
    wakeTransaction.readCount = 0;

    // Wake up IMU (PWR_MGMT_1 register)
    wakeTxBuffer[0] = 0x6B;
    wakeTxBuffer[1] = 0x00;
    if (!I2C_transfer(i2cHandle, &wakeTransaction)) return false;

    // Gyro scaling config
    wakeTxBuffer[0] = GYRO_CONFIG;
    wakeTxBuffer[1] = FS_SEL;
    if (!I2C_transfer(i2cHandle, &wakeTransaction)) return false;

    // Accel scaling config
    wakeTxBuffer[0] = ACCEL_CONFIG;
    wakeTxBuffer[1] = AFS_SEL;
    if (!I2C_transfer(i2cHandle, &wakeTransaction)) return false;

    return true;
}

bool IMU::read() {
    if (!i2cHandle) {
        return false;
    }

    // Refresh transaction parameters before transfer
    txBuffer[0] = ACCEL_XOUT_H;
    i2cTransaction.slaveAddress = IMU_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 14;

    // Perform the transfer
    bool transferOK = I2C_transfer(i2cHandle, &i2cTransaction);

    if (transferOK) {
        // Update struct
        data.accX = (rxBuffer[0] << 8) | rxBuffer[1];
        data.accY = (rxBuffer[2] << 8) | rxBuffer[3];
        data.accZ = (rxBuffer[4] << 8) | rxBuffer[5];

        // Skip rxBuffer[6] and [7] (Temperature)

        data.gyX = (rxBuffer[8] << 8) | rxBuffer[9];
        data.gyY = (rxBuffer[10] << 8) | rxBuffer[11];
        data.gyZ = (rxBuffer[12] << 8) | rxBuffer[13];

        return true;
    }

    return false; // Transfer failed
}

IMU_data IMU::getData() const {
    return data;
}
