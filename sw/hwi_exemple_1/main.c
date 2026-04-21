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

/*
 *  ======== hello.c ========
 */

/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Board.h>
#include <ti/drivers/I2C.h>

/* My Board */
#include "ti_drivers_config.h"

/*prototips funcions*/


void sendDataIdleFxn(void);

void readIMUData();

void myHwiButton0ISR(UArg arg);

void Wait_cycles(uint32_t numberCycles);

/* Variables globals */
#define IMU_ADDR
#define

/*
 *  ======== main ========
 */
int main()
{
    /* Call driver init functions */
    Board_init();
    GPIO_init();

    GPIO_write(CONFIG_GPIO_1, CONFIG_GPIO_LED_ON);

    // Habilitem la interrupcio del boto 0
    GPIO_enableInt(CONFIG_BUTTON_0);

    BIOS_start();

    return (0);
}



// Cal que sigui el mes rapid possible
void myHwiButton0ISR(UArg arg)
{
    GPIO_clearInt(CONFIG_BUTTON_0); // Neteja el flag
    GPIO_toggle(CONFIG_GPIO_1); // Commuta el LED
    cnt++; // Incrementa comptador
}


void sendDataIdleFxn(void)
{

    System_printf("El LED ha commutat %d vegades\n", cnt);
    System_flush();
    Wait_cycles(960000*10);     //esperem un numero de cicles qualsevol
}

void Wait_cycles(uint32_t numberCycles)
{
    volatile uint32_t i = 0; //variable comptador
    while (i < numberCycles) {i ++;} //fem bucle buit
}


void readIMUData() {

    // https://software-dl.ti.com/dsps/dsps_public_sw/sdo_sb/targetcontent/tirtos/2_20_00_06/exports/tirtos_full_2_20_00_06/products/tidrivers_full_2_20_00_08/docs/doxygen/html/_i2_c_8h.html

    // Estructures predefinides
    I2C_Handle      i2cHandle;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

    //uint8_t         txBuffer[1]; // Buffer for the register address
    //uint8_t         rxBuffer[6]; // Buffer for the incoming IMU data

    I2C_init(); // Inicialitzem I2C

    I2C_Params_init(&i2cParams); // Inicialitzem parametres I2C
    i2cParams.bitRate = I2C_400kHz; // definim bitRate

    // 3. Open the I2C interface (Board_I2C0 is mapped in your board file)
    i2cHandle = I2C_open(Board_I2C0, &i2cParams);
    if (i2cHandle == NULL) {
        // Handle initialization error (e.g., System_printf("Error Initializing I2C\n"); )
        while (1);
    }

    // 4. Set up the I2C transaction
    txBuffer[0] = ACCEL_XOUT_H;         // The register we want to read from

    i2cTransaction.slaveAddress = IMU_SLAVE_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;      // Send 1 byte (the register address)
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 6;       // Read 6 bytes back

    // 5. Perform the transfer
    bool transferOK = I2C_transfer(i2cHandle, &i2cTransaction);

    if (transferOK) {
        // Success! rxBuffer now holds your IMU data
        // int16_t accelX = (rxBuffer[0] << 8) | rxBuffer[1];
    } else {
        // Handle transfer error (e.g., device not connected or bad wiring)
    }

    // 6. Close the driver if no longer needed
    // I2C_close(i2cHandle);
}
