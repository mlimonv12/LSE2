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
    void TaskConsole(UArg arg1, UArg arg2);
    void Send_data_idle_fxn(void);
    void Wait_cycles(uint32_t numberCycles);
}

// Standard functions
I2C_Handle init_I2C(void);

/* Global variables */
volatile uint16_t cnt_glb = 0;
volatile uint8_t emer_glb = 0;

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

    while(1)
    {
        Semaphore_pend(semaphore_imu, BIOS_WAIT_FOREVER);
        if (imu.read()) {
            IMU_data currentData = imu.getData();
            Mailbox_post(mailbox_imu, &currentData, BIOS_NO_WAIT);
            Event_post(event_data, FLAG_IMU);
        }
    }
}

void TaskConsole(UArg arg1, UArg arg2)
{

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
