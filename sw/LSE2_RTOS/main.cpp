/* XDC Module Headers */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <xdc/cfg/global.h>

/* BIOS Module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Task.h>

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
//    void HwiTimer(UArg arg1);
void SwiClock(UArg arg1);
void TaskIMU(UArg arg1, UArg arg2);
void TaskPID(UArg arg1, UArg arg2);
void TaskSPI_TX(UArg arg1, UArg arg2);
void uartRxCallback(UART_Handle handle, void *buf, size_t count);
void spiRxCallback(SPI_Handle handle, SPI_Transaction *transaction);
void Send_data_idle_fxn(void);
void Wait_cycles(uint32_t numberCycles);
float IMU2angle(IMU_data IMUdata, float prevAngle, float dt);
int16_t constrainSpeed(int16_t inputVal);
}

typedef int16_t ConsData;

// Standard functions
I2C_Handle init_I2C(void);

/* Global variables */
volatile uint16_t cnt_glb = 0;
volatile uint8_t emer_glb = 0;

char uartTxBuf[128];

float alpha = 0.02f;
uint16_t setpoint = 130;

// RX Global Variables
UART_Handle uartHandle_rx;
SPI_Handle spiHandle_rx;
ConsData uartRxBuf;
ConsData spiRxBuf;
SPI_Transaction spiRxTransaction;

// Variables I2C
I2C_Handle i2cHandle;
IMU imu;
/*uint8_t txBuffer[1]; // Buffer for the register address
 uint8_t rxBuffer[14]; // Buffer for the incoming IMU data
 I2C_Transaction i2cTransaction;*/

// Master Handles
I2C_Handle i2cHandle_master;
SPI_Handle spiHandle_master;

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
    if (uartHandle_rx)
    {
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
    int16_t initialSpeed = 0;
    //spiHandle_rx = SPI_open(CONFIG_SPI_0, &spiSlaveParams);
    // Uncomment above when CONFIG_SPI_1 is added to SysConfig
    if (spiHandle_rx)
    {
        spiRxTransaction.count = 1;
        spiRxTransaction.rxBuf = &spiRxBuf;
        spiRxTransaction.txBuf = &initialSpeed;
        SPI_transfer(spiHandle_rx, &spiRxTransaction);
    }

    // Close SPI again

    //spiHandle_rx = SPI_close(CONFIG_SPI_0, &spiSlaveParams);

    //Task_sleep(10);

    //SPI_transfer(spiHandle_rx, &spiRxTransaction);
    //GPIO_write(SS_R, 1); // Manual CS High

    System_printf("Inicialitzacio BIOS...\n");
    System_flush();

    /*
     *  normal BIOS programs, would call BIOS_start() to enable interrupts
     *  and start the scheduler and kick BIOS into gear.  But, this program
     *  is a simple sanity test and calls BIOS_exit() instead.
     */
    BIOS_start(); /* terminates program and dumps SysMin output */
    return (0);
}

/* Threads */
/*void HwiTimer(UArg arg1) {
 GPIO_toggle(CONFIG_GPIO_0);
 }*/

void SwiClock(UArg arg1)
{
    // Clock_clearInt()
    //System_printf("Clock int called \n");
    //System_flush();
    Semaphore_post(semaphore_imu); // Cada X temps, llegim IMU
}

void TaskIMU(UArg arg1, UArg arg2)
{

    i2cHandle = init_I2C();

    if (i2cHandle == NULL)
    {
        System_printf("Error: I2C Init Failed\n");
        System_flush();
        Task_sleep(100);
        //Task_exit();
    }

    System_printf("About to initialise IMU \n");
    System_flush();

    if (!imu.init(i2cHandle))
    {
        System_printf("IMU mal inicialitzada \n");
    }

    System_printf("I2C initialised \n");
    System_flush();

    float offset = 0.0f;
    float angle = 0.0f;
    float currentAngle = 0.0f;
    int posNew = 0, posOldest = 1;
    int N = 40;
    //int N = 300;
    float angles[N];
    // Inicialitzacio timestamp
    float dt = 0.0f;
    Types_FreqHz freq;
    Timestamp_getFreq(&freq);
    float cpuFreq = (float) freq.lo;
    uint32_t oldTimeMeas = Timestamp_get32();

    IMU_data currentData;

    System_printf("Abans de restAngle \n");
    System_flush();

    // Calibracio IMU
    float restAngle = imu.calibrate(1000) * (180.0f / 3.14159265f) - 3.0f;
    //restAngle = 0.0f;

    System_printf("Despres de restAngle \n");
    System_flush();

    // Inicialitzacio vector promig
    for (int i = 0; i < N; i++)
    {
        uint32_t newTimeMeas = Timestamp_get32();
        uint32_t dTicks = newTimeMeas - oldTimeMeas;
        dt = (float) dTicks / cpuFreq;
        oldTimeMeas = newTimeMeas;

        currentData = imu.getData();
        angle = IMU2angle(currentData, angle, dt) + restAngle;
        angles[i] = angle;
    }

    System_printf("IMU calibrated \n");
    System_flush();

    while (1)
    {
        Semaphore_pend(semaphore_imu, BIOS_WAIT_FOREVER);
        //if (imu.read(i2cTransaction))
        if (imu.read())
        {
            //System_printf("YES\n");
            //System_flush();

            uint32_t newTimeMeas = Timestamp_get32();
            uint32_t dTicks = newTimeMeas - oldTimeMeas;
            dt = (float) dTicks / cpuFreq;
            oldTimeMeas = newTimeMeas;

            currentData = imu.getData();

            angles[posNew] = IMU2angle(currentData, angle, dt) + restAngle;

            angle += (angles[posNew] - angles[posOldest]) / ((float) N);

            posNew = posOldest;
            posOldest++;
            if (posOldest >= N)
                posOldest -= N;

            /*System_printf(
             "ACCEL - X: %d, Y: %d, Z: %d ----- GYRO - X: %d, Y: %d, Z: %d\r\n",
             currentData.accX, currentData.accY, currentData.accZ,
             currentData.gyX, currentData.gyY, currentData.gyZ);*/
            //System_printf("Angle = %f | ", angle);
            //System_flush();
            //snprintf(uartTxBuf, sizeof(uartTxBuf), "%f | ", angle);
            //UART_write(uartHandle_rx, uartTxBuf, strlen(uartTxBuf));

            snprintf(uartTxBuf, sizeof(uartTxBuf), "angle = %.2f | ",
                     angle, currentData.accY);

            UART_write(uartHandle_rx, uartTxBuf, strlen(uartTxBuf));

            // CODI BO
            Mailbox_post(mailbox_imu, &angle, BIOS_NO_WAIT);
            Event_post(event_data, FLAG_IMU);
        }
        else
        {
            //System_printf("IMU READING FAILED");
            //System_flush();
        }
    }
}

void uartRxCallback(UART_Handle handle, void *buf, size_t count)
{
    ConsData data = *(ConsData*) buf;
    Mailbox_post(mailbox_cons, &data, BIOS_NO_WAIT);
    Event_post(event_data, FLAG_CONS);

    // Re-prime UART to listen for the next transmission
    UART_read(handle, buf, count);
}

void spiRxCallback(SPI_Handle handle, SPI_Transaction *transaction)
{
    if (transaction->status == SPI_TRANSFER_COMPLETED)
    {
        ConsData data = *(ConsData*) transaction->rxBuf;
        Mailbox_post(mailbox_cons, &data, BIOS_NO_WAIT);
        Event_post(event_data, FLAG_CONS);
    }

    // Re-prime SPI to listen for the next transmission
    SPI_transfer(handle, transaction);
}

void TaskPID(UArg arg1, UArg arg2)
{
    float currentCons = 0.0f;
    float currentAngle = 0.0f;

    System_printf("Entrem a PID \n");
    System_flush();

    // Inicialitzacio timestamp
    float dt = 0.0f;
    Types_FreqHz freq;
    Timestamp_getFreq(&freq);
    float cpuFreq = (float) freq.lo;
    uint32_t oldTimeMeas = Timestamp_get32();

    // Constants PID
    // decent, pero amb offset:
    /*float Kp = 30.0f;
    float Ki = 0.0f;
    float Kd = 0.05f;*/

    float Kp = 20.0f;
    float Ki = 0.0f; // * erracum * dt
    float Kd = 0.0f;
    /*float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;*/
    float integral = 0.0f;
    float intMax = 50.0f;
    float intMin = -intMax;
    float last_error = 0.0f;

    while (1)
    {
        UInt events = Event_pend(event_data, Event_Id_NONE,
        FLAG_IMU | FLAG_CONS,
                                 BIOS_WAIT_FOREVER);

        if (events & FLAG_CONS)
        {
            // Update target from mailbox
            Mailbox_pend(mailbox_cons, &currentCons, BIOS_NO_WAIT);
        }

        if (events & FLAG_IMU)
        {
            // Read IMU angle from mailbox
            if (Mailbox_pend(mailbox_imu, &currentAngle, BIOS_NO_WAIT))
            {

                /*System_printf("Calculem PID \n");
                 System_flush();*/

                // Cal calcular dt
                uint32_t newTimeMeas = Timestamp_get32();
                uint32_t dTicks = newTimeMeas - oldTimeMeas;
                dt = (float) dTicks / cpuFreq;
                oldTimeMeas = newTimeMeas;

                float error = currentCons - currentAngle;
                integral += error * dt;
                if (integral > intMax) integral = intMax;
                if (integral < intMin) integral = intMin;

                snprintf(uartTxBuf, sizeof(uartTxBuf), "intTerm = %.3f | ", integral);

                UART_write(uartHandle_rx, uartTxBuf, strlen(uartTxBuf));

                float derivative = (error - last_error) / dt;

                int16_t output = (int16_t) ((Kp * error) + (Ki * integral)
                        + (Kd * derivative));
                last_error = error;

                //ConsData pidOutput = (ConsData) fabsf(output);
                //pidOutput = 1000;

                /*System_printf("PID computed: output = %d, dt = %f, Angle = %f \n", output, dt, currentAngle);
                 System_flush();*/

                // Send computed value to SPI TX task
                Mailbox_post(mailbox_pid, &output, BIOS_NO_WAIT);
            }
        }
    }
}

void TaskSPI_TX(UArg arg1, UArg arg2)
{
    ConsData sendOutput;
    ConsData pidOutput;
    uint32_t spiReceive;

    // Condicionament variable
    int16_t baseSpeedRight = 2400;
    int16_t baseSpeedLeft = 2650;

    SPI_init();
    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 100000;
    spiParams.dataSize = 16;

    SPI_Handle spiHandle = SPI_open(CONFIG_SPI_0, &spiParams);
    if (!spiHandle)
    {
        System_printf("Error: SPI Init Failed\n");
        System_flush();
        Task_exit();
    }

    SPI_Transaction transaction = { 0 }; // MUST 0-initialize to prevent driver crash
    transaction.count = 1;
    transaction.txBuf = (void*) &sendOutput;
    transaction.rxBuf = (void*) &spiReceive;

    sendOutput = 0; // Apaguem motors inicialment per permetre configuracio IMU
    GPIO_write(SS_L, 0); // Manual CS Low
    GPIO_write(SS_R, 0); // Manual CS Low
    SPI_transfer(spiHandle, &transaction);
    GPIO_write(SS_L, 1); // Manual CS High
    GPIO_write(SS_R, 1); // Manual CS High

    System_printf("SPI done \n");
    System_flush();

    while (1)
    {
        Mailbox_pend(mailbox_pid, &pidOutput, BIOS_WAIT_FOREVER);
        //pidOutput = 0;

        /*System_printf("L: %d | R: %d | PID: %d \n",
        constrainSpeed(baseSpeed + pidOutput),
        constrainSpeed(baseSpeed - pidOutput), pidOutput);
        System_flush();*/

        snprintf(uartTxBuf, sizeof(uartTxBuf), "PID: %d \n", pidOutput);

        UART_write(uartHandle_rx, uartTxBuf, strlen(uartTxBuf));

        //pidOutput = 0;

        sendOutput = constrainSpeed(baseSpeedLeft + pidOutput);
        GPIO_write(SS_L, 0); // Manual CS Low
        SPI_transfer(spiHandle, &transaction);
        GPIO_write(SS_L, 1); // Manual CS High

        //Task_sleep(10);

        sendOutput = constrainSpeed(baseSpeedRight - pidOutput);
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
    while (i < numberCycles)
    {
        i++;
    } //fem bucle buit
}

/* Standard functions */
I2C_Handle init_I2C(void)
{
    I2C_init();

    static I2C_Params i2cParams;
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    I2C_Handle i2cHandle = I2C_open(CONFIG_I2C_0, &i2cParams);
    return i2cHandle;
}

float IMU2angle(IMU_data IMUdata, float prevAngle, float dt)
{
    float acc_angle = atan2f((float) IMUdata.accY, (float) IMUdata.accZ)
            * (180.0f / 3.14159265f);
    float newAngle = alpha * (prevAngle + ((float) IMUdata.gyX) / GY_CTT * dt)
            + (1.0f - alpha) * acc_angle;
    return newAngle;
}

int16_t constrainSpeed(int16_t inputVal)
{
    int16_t minVal = 1;
    int16_t maxVal = 3800;

    if (inputVal < minVal)
        return minVal;
    if (inputVal > maxVal)
        return maxVal;
    return inputVal;
}
