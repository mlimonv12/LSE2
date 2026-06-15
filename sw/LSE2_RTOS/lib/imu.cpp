/*
 * imu.c
 *
 *  Created on: 6 May 2026
 *      Author: adria
 */


#include "imu.h"
#include <xdc/runtime/System.h>
#include <cstddef> // for NULL
#include <math.h>

IMU::IMU() : i2cHandle(NULL) {
    // Initialize the data structure to 0
    data = {0, 0, 0, 0, 0, 0};
}

bool IMU::init(I2C_Handle handle) {

    i2cHandle = handle;

    if (!i2cHandle) {
        return false;
    }

    uint8_t wakeTxBuffer[2];
    I2C_Transaction wakeTransaction;

    // Wake up IMU (PWR_MGMT_1 register)
    wakeTxBuffer[0] = 0x6B;
    wakeTxBuffer[1] = 0x00;

    wakeTransaction.slaveAddress = IMU_ADDR;
    wakeTransaction.writeBuf = wakeTxBuffer;
    wakeTransaction.writeCount = 2;
    wakeTransaction.readBuf = NULL;
    wakeTransaction.readCount = 0;

    if (!I2C_transfer(i2cHandle, &wakeTransaction)) return false;

    // Gyro scaling config
    wakeTxBuffer[0] = GYRO_CONFIG;
    wakeTxBuffer[1] = FS_SEL;
    if (!I2C_transfer(i2cHandle, &wakeTransaction)) return false;

    // Accel scaling config
    wakeTxBuffer[0] = ACCEL_CONFIG;
    wakeTxBuffer[1] = AFS_SEL;
    if (!I2C_transfer(i2cHandle, &wakeTransaction)) return false;

    // DLPF
    wakeTxBuffer[0] = 0x1A;
    wakeTxBuffer[1] = 0x03;
    if (!I2C_transfer(i2cHandle, &wakeTransaction)) return false;

    txBuffer[0] = ACCEL_XOUT_H;
    i2cTransaction.slaveAddress = IMU_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 14;

    return true;
}

/*
bool IMU::leanRight() {

    int32_t checkSide = 0;

    for (int i = 0; i < 10; i++) {
        if (I2C_transfer(i2cHandle, &i2cTransaction)) {

            checkSide += (rxBuffer[2] << 8) | rxBuffer[3];
            return true;
        }
    }

    if (checkSide < 0) {
        restAngle *= -1.0f;
        return false;
    } else {
        return true;
    }
}*/


float IMU::calibrate(int N) {

    for (int i = 0; i < N; i++) {

        if (I2C_transfer(i2cHandle, &i2cTransaction)) {
            // Update struct
            ofs.accX += (rxBuffer[0] << 8) | rxBuffer[1];
            ofs.accY += (rxBuffer[2] << 8) | rxBuffer[3];
            ofs.accZ += (rxBuffer[4] << 8) | rxBuffer[5];

            ofs.gyX += (rxBuffer[8] << 8) | rxBuffer[9];
            ofs.gyY += (rxBuffer[10] << 8) | rxBuffer[11];
            ofs.gyZ += (rxBuffer[12] << 8) | rxBuffer[13];
        }
    }

    ofs.accX /= N;
    //ofs.accY = (ofs.accY / N) + ((int16_t) (ACC_CTT * sinf(restAngle)));
    //ofs.accZ = (ofs.accZ / N) - ((int16_t) (ACC_CTT * cosf(restAngle)));
    ofs.accY /= N;
    ofs.accZ = (ofs.accZ / N) - ((int16_t) ACC_CTT);
    ofs.gyX /= N;
    ofs.gyY /= N;
    ofs.gyZ /= N;

    restAngle = 28.75f * (3.14159265f / 180.0f);

    int32_t checkSide = 0;
    int Ncheck = 10;

    for (int i = 0; i < Ncheck; i++) {
        if (I2C_transfer(i2cHandle, &i2cTransaction)) {

            checkSide += ((rxBuffer[2] << 8) | rxBuffer[3]);// - ofs.accY;
        }
    }

    checkSide /= Ncheck;

    System_printf("Checkside = %d \n", checkSide);
    System_flush();

    if (checkSide > 10000) {
        restAngle *= -1.0f;
        System_printf("LEANING LEFT \n");
    } else {
        System_printf("LEANING RIGHT \n");
    }
    System_flush();


    return restAngle;
}


//bool IMU::read(I2C_Transaction i2cTransaction) {
bool IMU::read() {

    if (!i2cHandle) {
        return false;
    }

    // Perform the transfer
    bool transferOK = I2C_transfer(i2cHandle, &i2cTransaction);

    if (transferOK) {
        // Update struct
        data.accX = ((rxBuffer[0] << 8) | rxBuffer[1]) - ofs.accX;
        data.accY = ((rxBuffer[2] << 8) | rxBuffer[3]) - ofs.accY;
        data.accZ = ((rxBuffer[4] << 8) | rxBuffer[5]) - ofs.accZ;

        // Skip rxBuffer[6] and [7] (Temperature)

        data.gyX = ((rxBuffer[8] << 8) | rxBuffer[9]) - ofs.gyX;
        data.gyY = ((rxBuffer[10] << 8) | rxBuffer[11]) - ofs.gyY;
        data.gyZ = ((rxBuffer[12] << 8) | rxBuffer[13]) - ofs.gyZ;

        return true;
    }

    return false; // Transfer failed
}

IMU_data IMU::getData() const {
    return data;
}
