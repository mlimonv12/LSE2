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
#include <ti/sysbios/knl/Task.h>
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

/*prototips funcions*/

typedef struct
{
    int16_t accX, accY, accZ;
    //int16_t temperature;
    int16_t gyX, gyY, gyZ;
} IMU_data;

// Estructures predefinides
I2C_Handle i2cHandle;
I2C_Params i2cParams;
I2C_Transaction i2cTransaction;
IMU_data IMU;

SPI_Handle spiHandle;
SPI_Params spiParams;
SPI_Transaction spiTransaction;

UART_Handle uartHandle;
UART_Params uartParams;
char txString[128];

uint8_t txBuffer[1]; // Buffer for the register address
uint8_t rxBuffer[14]; // Buffer for the incoming IMU data
float velocity;
uint8_t *bytePointer = (uint8_t*) &velocity;
uint32_t spiReceive;
float alpha = 0.04f;
float angle = 0.0f;
float dt = 0.1f;
float t_prev = 0;
uint16_t setpoint = 130;

// -----------------------------------------
int i2c = 1;
// -----------------------------------------
int spi = 1;
// -----------------------------------------
int uart = 1;
// -----------------------------------------

//void sendDataIdleFxn(void);
void imuCommunicationTask(UArg arg0, UArg arg1);

void readIMUData(void);

//void myHwiButton0ISR(UArg arg);

void Wait_cycles(uint32_t numberCycles);

/* Variables globals */
#define IMU_ADDR 0x68
#define ACCEL_XOUT_H 0x3B
#define GYRO_CONFIG 0x1B
#define FS_SEL 1 // +- 500^o/s
#define ACCEL_CONFIG 0x1C
#define AFS_SEL 1 // +- 4g
float ACC_CTT = 16384.0f / 2.0f;
float GY_CTT = 131.0f / 2.0f;

/*
 *  ======== main ========
 */
int main()
{
    /* Call driver init functions */
    Board_init();
    GPIO_init();

    // Slave select (active low)
    GPIO_write(SS_R, 1);

    if (i2c)
    {
        I2C_init(); // Inicialitzem I2C

        I2C_Params_init(&i2cParams); // Inicialitzem parametres I2C
        i2cParams.bitRate = I2C_400kHz; // definim bitRate

        // 3. Open the I2C interface (Board_I2C0 is mapped in your board file)
        i2cHandle = I2C_open(CONFIG_I2C_0, &i2cParams);
        //if (i2cHandle == NULL) {
        // Handle initialization error (e.g., System_printf("Error Initializing I2C\n"); )
        //    while (1);
    }

    /*
     uint8_t wakeTxBuffer[2];
     I2C_Transaction wakeTransaction;

     wakeTxBuffer[0] = 0x6B; // PWR_MGMT_1 register address
     wakeTxBuffer[1] = 0x00; // Value to write: 0 clears the SLEEP bit

     wakeTransaction.slaveAddress = IMU_ADDR;
     wakeTransaction.writeBuf = wakeTxBuffer;
     wakeTransaction.writeCount = 2;
     wakeTransaction.readBuf = NULL;
     wakeTransaction.readCount = 0;

     while(!I2C_transfer(i2cHandle, &wakeTransaction));


     // Gyro scaling
     wakeTxBuffer[0] = GYRO_CONFIG;
     wakeTxBuffer[1] = FS_SEL;

     wakeTransaction.slaveAddress = IMU_ADDR;
     wakeTransaction.writeBuf = wakeTxBuffer;
     wakeTransaction.writeCount = 2;
     wakeTransaction.readBuf = NULL;
     wakeTransaction.readCount = 0;

     while(!I2C_transfer(i2cHandle, &wakeTransaction));


     // Accel scaling

     wakeTxBuffer[0] = ACCEL_CONFIG;
     wakeTxBuffer[1] = AFS_SEL;

     wakeTransaction.slaveAddress = IMU_ADDR;
     wakeTransaction.writeBuf = wakeTxBuffer;
     wakeTransaction.writeCount = 2;
     wakeTransaction.readBuf = NULL;
     wakeTransaction.readCount = 0;

     while(!I2C_transfer(i2cHandle, &wakeTransaction));



     //if (!I2C_transfer(i2cHandle, &wakeTransaction)) {
     //    System_printf("Failed to wake up the IMU!\n");
     //    System_flush();
     //    while(1); // Stop here if IMU doesn't respond
     //}

     // 4. Set up the I2C transaction
     txBuffer[0] = ACCEL_XOUT_H;       // Comencem a llegir a aquest registre

     i2cTransaction.slaveAddress = IMU_ADDR;
     i2cTransaction.writeBuf = txBuffer;
     i2cTransaction.writeCount = 1;     // Send 1 byte (the register address)
     i2cTransaction.readBuf = rxBuffer;
     i2cTransaction.readCount = 14; // Demanem 14 bytes per tenir tots els valors de cop
     }*/

    if (spi)
    {
        SPI_init();
        SPI_Params_init(&spiParams);
        spiParams.bitRate = 100000;
        spiParams.dataSize = 16; // El setpoint es un uint32_t
        spiHandle = SPI_open(CONFIG_SPI_0, &spiParams);
        if (!spiHandle)
        {
            System_printf("SPI did not open");
        }

    }

    if (uart)
    {
        // Initialize the UART driver
        UART_init();
        UART_Params_init(&uartParams);

        // Configure UART for 115200 baud rate (standard for serial monitors)
        uartParams.writeDataMode = UART_DATA_BINARY;
        uartParams.readDataMode = UART_DATA_BINARY;
        uartParams.readReturnMode = UART_RETURN_FULL;
        uartParams.readEcho = UART_ECHO_OFF;
        uartParams.baudRate = 115200;

        // Open the UART interface (assuming your board file uses CONFIG_UART_0)
        uartHandle = UART_open(CONFIG_UART_0, &uartParams);
        /*if (uartHandle == NULL)
        {
            System_printf("UART did not open\n");
            System_flush();
            while (1)
                ; // Trap to prevent a crash if UART fails
        }*/
    }
    /*

     Task_Params taskParams;
     Task_Params_init(&taskParams);
     taskParams.stackSize = 1024; // Allocate stack memory for the task
     taskParams.priority = 1;     // Normal priority

     Task_create((Task_FuncPtr)imuCommunicationTask, &taskParams, NULL);

     spiTransaction.count = 1;
     spiTransaction.txBuf = (void *)&setpoint;
     spiTransaction.rxBuf = (void *)&spiReceive;

     // Cal enviar 4 bytes
     /*
     for (int i = 0; i < 4; i++)
     {
     ret = SPI_transfer(bytePointer[4], &spiTransaction);
     if (!ret)
     {
     System_printf("Unsuccessful SPI transfer");
     }
     }

     }*/

    //GPIO_write(CONFIG_GPIO_1, CONFIG_GPIO_LED_ON);
    // Habilitem la interrupcio del boto 0
    //GPIO_enableInt(CONFIG_BUTTON_0);
    BIOS_start();

    return (0);
}

void imuCommunicationTask(UArg arg0, UArg arg1)
{
    uint8_t wakeTxBuffer[2];
    I2C_Transaction wakeTransaction;

    if (i2c)
    {
        // --- DO INITIAL I2C SETUP HERE ONCE OS IS RUNNING ---
        wakeTransaction.slaveAddress = IMU_ADDR;
        wakeTransaction.writeBuf = wakeTxBuffer;
        wakeTransaction.writeCount = 2;
        wakeTransaction.readBuf = NULL;
        wakeTransaction.readCount = 0;

        // PWR_MGMT_1: Wake up
        wakeTxBuffer[0] = 0x6B;
        wakeTxBuffer[1] = 0x00;
        I2C_transfer(i2cHandle, &wakeTransaction);

        // Gyro scaling
        wakeTxBuffer[0] = GYRO_CONFIG;
        wakeTxBuffer[1] = FS_SEL;
        I2C_transfer(i2cHandle, &wakeTransaction);

        // Accel scaling
        wakeTxBuffer[0] = ACCEL_CONFIG;
        wakeTxBuffer[1] = AFS_SEL;
        I2C_transfer(i2cHandle, &wakeTransaction);
    }

    if (spi)
    {
        // Setup recurring SPI transaction
        spiTransaction.count = 1;
        spiTransaction.txBuf = (void*) &setpoint;
        spiTransaction.rxBuf = (void*) &spiReceive;
    }

    while (1)
    {
        if (i2c)
        {
            readIMUData();

            if (uart)
            {
                float acc_angle_pitch = atan2((float)IMU.accX, (float)IMU.accZ) * (180.0f / 3.14159265f);
                angle = alpha * (angle + ((float) IMU.gyY) / GY_CTT * dt)
                        + (1.0f - alpha) * acc_angle_pitch;
                setpoint = (uint16_t) (abs(angle) * 22.75f);//* 45.5f); // = / 90.0f * 4095.0f
                                //+ (1.0f - alpha) * ((float) IMU.accZ) / ACC_CTT;
                //sprintf(txString, "ACCEL - X: %d, Y: %d, Z: %d ----- GYRO - X: %d, Y: %d, Z: %d\r\n",
                //        IMU.accX, IMU.accY, IMU.accZ, IMU.gyX, IMU.gyY, IMU.gyZ);
                /*sprintf(txString, "Angle = %f, enviem setpoint = %d \r\n",
                        angle, setpoint);

                // Send the string over UART
                UART_write(uartHandle, txString, strlen(txString));*/
            }

            // Also send to system console
            //System_printf(
            //        "ACCEL - X: %d, Y: %d, Z: %d ----- GYRO - X: %d, Y: %d, Z: %d \n",
            //        IMU.accX, IMU.accY, IMU.accZ, IMU.gyX, IMU.gyY, IMU.gyZ);
        }

        if (spi)
        {
            // Calcul bytes
            //setpoint = 60000; // (uint16_t) (((angle + 3.0f) / 5.0f) * 65536.0f);

            GPIO_write(SS_R, 0);
            //Wait_cycles(100);
            bool success = SPI_transfer(spiHandle, &spiTransaction);
            //Wait_cycles(100);
            GPIO_write(SS_R, 1);
            //Wait_cycles(100);

            /*if (!success)
            {
                sprintf(txString, "SPI Transfer hardware blocked!\r\n");
                UART_write(uartHandle, txString, strlen(txString));
            }*/

        }

        System_flush();

        Task_sleep(100); // Sleeps for 100 system ticks (usually 1 millisecond per tick)
    }
}

/*
 void sendDataIdleFxn(void)
 {

 if (i2c)
 {
 // Check IMU values
 readIMUData();

 System_printf(
 "ACCELEROMETRE - X: %d, Y: %d, Z: %d ----- GIROSCOPI - X: %d, Y: %d, Z: %d \n",
 IMU.accX, IMU.accY, IMU.accZ, IMU.gyX, IMU.gyY, IMU.gyZ);

 //sprintf(txString, "%d,%d,%d,%d,%d,%d\n", IMU.accX, IMU.accY, IMU.accZ,
 //IMU.gyX, IMU.gyY, IMU.gyZ);
 }

 angle = alpha * (angle + ((float) IMU.gyZ) * dt)
 + (1 - alpha) * ((float) IMU.accZ);


 // Send SPI data
 // while (!SPI_transfer(bytePointer[4], &spiTransaction));
 while (!SPI_transfer(spiHandle, &spiTransaction));
 //UART_write(uartHandle, txString, strlen(txString));
 System_flush();
 Wait_cycles(960000);     //esperem un numero de cicles qualsevol
 }
 */
void Wait_cycles(uint32_t numberCycles)
{
    volatile uint32_t i = 0; //variable comptador
    while (i < numberCycles)
    {
        i++;
    } //fem bucle buit
}

void readIMUData(void)
{
    // 1. Refresh the transaction parameters BEFORE the transfer
    txBuffer[0] = ACCEL_XOUT_H;
    i2cTransaction.slaveAddress = IMU_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 14;

    // 2. Perform the transfer and check for failure
    bool transferOK = I2C_transfer(i2cHandle, &i2cTransaction);

    if (transferOK)
    {
        // 3. Only update the IMU struct if the transfer succeeded
        IMU.accX = (rxBuffer[0] << 8) | rxBuffer[1];
        IMU.accY = (rxBuffer[2] << 8) | rxBuffer[3];
        IMU.accZ = (rxBuffer[4] << 8) | rxBuffer[5];
        IMU.gyX = (rxBuffer[8] << 8) | rxBuffer[9];
        IMU.gyY = (rxBuffer[10] << 8) | rxBuffer[11];
        IMU.gyZ = (rxBuffer[12] << 8) | rxBuffer[13];
    }
    else
    {
        System_printf("I2C Transfer Failed!\n");
        // Don't update IMU data so you don't process garbage bytes
    }
}
