/*
 *
 * PROJECTE DE PROVES PER LES COMUNICACIONS AMB LA IMU
 *
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


typedef struct {
    int16_t accX, accY, accZ;
    //int16_t temperature;
    int16_t gyX, gyY, gyZ;
} IMU_data;




// Estructures predefinides
I2C_Handle      i2cHandle;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;
IMU_data IMU;

uint8_t     txBuffer[1]; // Buffer for the register address
uint8_t     rxBuffer[14]; // Buffer for the incoming IMU data

void sendDataIdleFxn(void);

void readIMUData(void);

void myHwiButton0ISR(UArg arg);

void Wait_cycles(uint32_t numberCycles);

/* Variables globals */
#define IMU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B

/*
 *  ======== main ========
 */
int main()
{
    /* Call driver init functions */
    Board_init();
    GPIO_init();

    I2C_init(); // Inicialitzem I2C

    I2C_Params_init(&i2cParams); // Inicialitzem parametres I2C
    i2cParams.bitRate = I2C_400kHz; // definim bitRate

    // 3. Open the I2C interface (Board_I2C0 is mapped in your board file)
    i2cHandle = I2C_open(CONFIG_I2C_0, &i2cParams);
    //if (i2cHandle == NULL) {
        // Handle initialization error (e.g., System_printf("Error Initializing I2C\n"); )
    //    while (1);
    //}


    uint8_t wakeTxBuffer[2];
    I2C_Transaction wakeTransaction;

    wakeTxBuffer[0] = 0x6B; // PWR_MGMT_1 register address
    wakeTxBuffer[1] = 0x00; // Value to write: 0 clears the SLEEP bit

    wakeTransaction.slaveAddress = IMU_ADDR;
    wakeTransaction.writeBuf     = wakeTxBuffer;
    wakeTransaction.writeCount   = 2;
    wakeTransaction.readBuf      = NULL;
    wakeTransaction.readCount    = 0;

    //if (!I2C_transfer(i2cHandle, &wakeTransaction)) {
    //    System_printf("Failed to wake up the IMU!\n");
    //    System_flush();
    //    while(1); // Stop here if IMU doesn't respond
    //}


    // 4. Set up the I2C transaction
    txBuffer[0] = ACCEL_XOUT_H;         // Comencem a llegir a aquest registre

    i2cTransaction.slaveAddress = IMU_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;      // Send 1 byte (the register address)
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 14;       // Demanem 14 bytes per tenir tots els valors de cop


    //GPIO_write(CONFIG_GPIO_1, CONFIG_GPIO_LED_ON);

    // Habilitem la interrupcio del boto 0
    //GPIO_enableInt(CONFIG_BUTTON_0);

    BIOS_start();

    return (0);
}



void sendDataIdleFxn(void)
{

    // Check IMU values
    readIMUData();

    System_printf("ACCELEROMETRE - X: %d, Y: %d, Z: %d ----- GIROSCOPI - X: %d, Y: %d, Z: %d \n", IMU.accX, IMU.accY, IMU.accZ, IMU.gyX, IMU.gyY, IMU.gyZ);

    sprintf(txString, "%d,%d,%d,%d,%d,%d\n", IMU.accX, IMU.accY, IMU.accZ, IMU.gyX, IMU.gyY, IMU.gyZ);
    UART_write(uartHandle, txString, strlen(txString));
    System_flush();
    Wait_cycles(960000);     //esperem un numero de cicles qualsevol
}

void Wait_cycles(uint32_t numberCycles)
{
    volatile uint32_t i = 0; //variable comptador
    while (i < numberCycles) {i ++;} //fem bucle buit
}


void readIMUData(void) {

    // 5. Perform the transfer
    // bool transferOK = I2C_transfer(i2cHandle, &i2cTransaction);
    I2C_transfer(i2cHandle, &i2cTransaction);

    IMU.accX = (rxBuffer[0] << 8) | rxBuffer[1];
    IMU.accY = (rxBuffer[2] << 8) | rxBuffer[3];
    IMU.accZ = (rxBuffer[4] << 8) | rxBuffer[5];
    IMU.gyX = (rxBuffer[8] << 8) | rxBuffer[9];
    IMU.gyY = (rxBuffer[10] << 8) | rxBuffer[11];
    IMU.gyZ = (rxBuffer[12] << 8) | rxBuffer[13];

    /*
    if (transferOK) {
        dataBuf = (rxBuffer[0] << 8);
        // Success! rxBuffer now holds your IMU data
        // int16_t accelX = (rxBuffer[0] << 8) | rxBuffer[1];
    } else {
        // Handle transfer error (e.g., device not connected or bad wiring)
    }
    */
    // 6. Close the driver if no longer needed
    // I2C_close(i2cHandle);
}
