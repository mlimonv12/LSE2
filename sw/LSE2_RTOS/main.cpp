/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>

/* TI Drivers */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Board.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>

/* Standard C/C++ */
#include <stdio.h>
#include <string.h>
#include <math.h>

/* My Board */
#include "ti_drivers_config.h"

/* Custom libs */
#include "lib/imu.h"

/* Prototypes */
// Threads
extern "C" {
    void HwiTimer(UArg arg1);
    void SwiClock(UArg arg1);
    void TaskIMU(UArg arg1, UArg arg2);
    void TaskPID(UArg arg1, UArg arg2);
    void TaskSPI_TX(UArg arg1, UArg arg2);
    void uartRxCallback(UART_Handle handle, void *buf, size_t count);
    void spiRxCallback(SPI_Handle handle, SPI_Transaction *transaction);
    void Send_data_idle_fxn(void);
    void Wait_cycles(uint32_t numberCycles);
}

typedef uint16_t ConsData;

// Standard functions
I2C_Handle init_I2C(void);

/* Global variables */
volatile uint16_t cnt_glb = 0;
volatile uint8_t emer_glb = 0;

float alpha = 0.04f;
float dt = 0.1f;

// RX Global Variables
UART_Handle uartHandle_rx;
SPI_Handle spiHandle_rx;
ConsData uartRxBuf;
ConsData spiRxBuf;
SPI_Transaction spiRxTransaction;

// volatile uint8_t emergency_glb = 0;
// volatile uint16_t cnt_glb = 0;

#define FLAG_IMU Event_Id_00
#define FLAG_CONS Event_Id_01

/*
 *  ======== main ========
 */
int main()
{
    /* Call driver init functions */
    Board_init();
    GPIO_init();

    // Set Chip Select High initially (deselected)
    GPIO_write(SS_R, 1);

    // Initialize UART for Callback RX
    UART_init();
    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = uartRxCallback;
    uartHandle_rx = UART_open(CONFIG_UART_0, &uartParams);
    if (uartHandle_rx) {
        UART_read(uartHandle_rx, &uartRxBuf, sizeof(ConsData));
    }

    // Initialize SPI Slave for Callback RX (assuming CONFIG_SPI_1)
    SPI_init();
    SPI_Params spiSlaveParams;
    SPI_Params_init(&spiSlaveParams);
    spiSlaveParams.mode = SPI_SLAVE;
    spiSlaveParams.transferMode = SPI_MODE_CALLBACK;
    spiSlaveParams.transferCallbackFxn = spiRxCallback;
    spiSlaveParams.dataSize = 16;
    // spiHandle_rx = SPI_open(CONFIG_SPI_1, &spiSlaveParams); 
    // Uncomment above when CONFIG_SPI_1 is added to SysConfig
    if (spiHandle_rx) {
        spiRxTransaction.count = 1;
        spiRxTransaction.rxBuf = &spiRxBuf;
        spiRxTransaction.txBuf = NULL;
        SPI_transfer(spiHandle_rx, &spiRxTransaction);
    }

    System_printf("Inicialitzacio BIOS...\n");
    System_flush();

    /*
     *  normal BIOS programs, would call BIOS_start() to enable interrupts
     *  and start the scheduler and kick BIOS into gear.  But, this program
     *  is a simple sanity test and calls BIOS_exit() instead.
     */
    BIOS_start();  /* terminates program and dumps SysMin output */
    return(0);
}

/* Threads */
void HwiTimer(UArg arg1) {
    GPIO_toggle(CONFIG_GPIO_0);
}

void SwiClock(UArg arg1)
{
    // Clock_clearInt()
    Semaphore_post(semaphore_imu); // Cada X temps, llegim IMU
}

void TaskIMU(UArg arg1, UArg arg2)
{
    // Declarations
    I2C_Handle i2cHandle = init_I2C();

    if (i2cHandle == NULL) {
        System_printf("Error: I2C Init Failed\n");
        System_flush();
        Task_exit();
    }

    IMU imu(i2cHandle);

    // Initialization
    if (!imu.init()) {
        System_printf("Warning: IMU Init Failed on startup\n");
        System_flush();
        Task_exit();
    }

    float angle = 0.0f;

    while(1)
    {
        Semaphore_pend(semaphore_imu, BIOS_WAIT_FOREVER);
        if (imu.read()) {
            IMU_data currentData = imu.getData();
            
            float acc_angle_pitch = atan2((float)currentData.accX, (float)currentData.accZ) * (180.0f / 3.14159265f);
            angle = alpha * (angle + ((float)currentData.gyY) / GY_CTT * dt) + (1.0f - alpha) * acc_angle_pitch;

            Mailbox_post(mailbox_imu, &angle, BIOS_NO_WAIT);
            Event_post(event_data, FLAG_IMU);
        }
    }
}

void uartRxCallback(UART_Handle handle, void *buf, size_t count)
{
    ConsData data = *(ConsData*)buf;
    Mailbox_post(mailbox_cons, &data, BIOS_NO_WAIT);
    Event_post(event_data, FLAG_CONS);
    
    // Re-prime UART to listen for the next transmission
    UART_read(handle, buf, count);
}

void spiRxCallback(SPI_Handle handle, SPI_Transaction *transaction)
{
    if (transaction->status == SPI_TRANSFER_COMPLETED) {
        ConsData data = *(ConsData*)transaction->rxBuf;
        Mailbox_post(mailbox_cons, &data, BIOS_NO_WAIT);
        Event_post(event_data, FLAG_CONS);
    }
    
    // Re-prime SPI to listen for the next transmission
    SPI_transfer(handle, transaction);
}

void TaskPID(UArg arg1, UArg arg2)
{
    ConsData currentCons = 130; // Default target mapped from old.c.bak
    float currentAngle = 0.0f;
    
    // PID State
    float Kp = 22.75f;
    float Ki = 0.0f;
    float Kd = 0.0f;
    float integral = 0.0f;
    float last_error = 0.0f;

    while(1)
    {
        UInt events = Event_pend(event_data, Event_Id_NONE, FLAG_IMU | FLAG_CONS, BIOS_WAIT_FOREVER);

        if (events & FLAG_CONS) {
            // Update target from mailbox
            Mailbox_pend(mailbox_cons, &currentCons, BIOS_NO_WAIT);
        }

        if (events & FLAG_IMU) {
            // Read IMU angle from mailbox
            if (Mailbox_pend(mailbox_imu, &currentAngle, BIOS_NO_WAIT)) {
                
                float error = (float)currentCons - currentAngle;
                integral += error * dt;
                float derivative = (error - last_error) / dt;
                
                float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
                last_error = error;
                
                ConsData pidOutput = (ConsData)fabs(output); 
                
                // Send computed value to SPI TX task
                Mailbox_post(mailbox_pid, &pidOutput, BIOS_NO_WAIT);
            }
        }
    }
}

void TaskSPI_TX(UArg arg1, UArg arg2)
{
    ConsData pidInput;
    uint32_t spiReceive;
    
    SPI_init();
    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 100000;
    spiParams.dataSize = 16;
    
    SPI_Handle spiHandle = SPI_open(CONFIG_SPI_0, &spiParams);
    if (!spiHandle) {
        System_printf("Error: SPI Init Failed\n");
        System_flush();
        Task_exit();
    }

    SPI_Transaction transaction;
    transaction.count = 1;
    transaction.txBuf = (void*)&pidInput;
    transaction.rxBuf = (void*)&spiReceive;

    while(1)
    {
        Mailbox_pend(mailbox_pid, &pidInput, BIOS_WAIT_FOREVER);

        GPIO_write(SS_R, 0); // Manual CS Low
        SPI_transfer(spiHandle, &transaction);
        GPIO_write(SS_R, 1); // Manual CS High
    }
}

void Send_data_idle_fxn(void)
{
    // System_printf("LED: %d \n", cnt);
    //System_flush();
    Wait_cycles(960000);     //esperem un numero de cicles qualsevol
}

void Wait_cycles(uint32_t numberCycles)
{
    volatile uint32_t i = 0; //variable comptador
    while (i < numberCycles) {i ++;} //fem bucle buit
}


/* Standard functions */
I2C_Handle init_I2C(void)
{
    I2C_init();

    // Configure params
    I2C_Params i2cParams;
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open interface and get handle
    I2C_Handle i2cHandle = I2C_open(CONFIG_I2C_0, &i2cParams);

    return i2cHandle;
}
