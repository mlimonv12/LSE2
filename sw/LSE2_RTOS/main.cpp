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
#include <stdlib.h>
#include <math.h>

/* My Board */
#include "ti_drivers_config.h"

/* Custom libs */
#include "lib/imu.h"
#include "lib/modulRF.h"

/* Prototypes */
// Threads
extern "C" {
//    void HwiTimer(UArg arg1);
void HwiRF(uint_least8_t index);
void SwiClock(UArg arg1);
void TaskIMU(UArg arg1, UArg arg2);
void TaskPID(UArg arg1, UArg arg2);
void TaskRF(UArg arg1, UArg arg2);
void TaskSPI_TX(UArg arg1, UArg arg2);
void task_readUART(UArg arg1, UArg arg2);
void spiRxCallback(SPI_Handle handle, SPI_Transaction *transaction);
void Send_data_idle_fxn(void);
void Wait_cycles(uint32_t numberCycles);
float IMU2angle(IMU_data IMUdata, float prevAngle, float dt);
int16_t constrainSpeed(int16_t inputVal);
float constrainAngle(float inputVal);
}

typedef int16_t ConsData;

typedef struct
{
    float angle;
    float wang;
} IMU_Message;

// Standard functions
I2C_Handle init_I2C(void);

/* Global variables */
volatile uint16_t cnt_glb = 0;
volatile uint8_t emer_glb = 0;

float Kp = 30.0f;
float Ki = 80.0f;
//float Kd = 2.5f;
float Kd = 5.0f;
float intMax = 5.0f;

//float Kp = 35.0f;
//float Ki = 90.0f;
//float Kd = 5.0f;
//float intMax = 8.0f;

float intFrac = 1.0f;
float intMin = -intMax;
int16_t offset = 0; //150;
float goalCons = 0.0f;
float consSlewRate = 0.2f;
int16_t minSpeed = 2500;
int16_t maxSpeed = 3500;
ConsData baseSpeed = 3000;
float propLeft = 1.0f;
float angleOffset = 0.0f;//15.0f;
int N = 15; // 40

float angleScale = 1.5f;

char singleRxChar;
char uartTxBuf[128];
char uartRxBuf[128];

float alpha = 0.02f;
uint16_t setpoint = 130;

// RX Global Variables
UART_Handle uartHandle_rx;
SPI_Handle spiHandle_rx;

uint8_t rxIndex = 0;
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
#define FLAG_PRINT Event_Id_02

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
    //uartParams.readMode = UART_MODE_CALLBACK;
    //uartParams.readCallback = uartRxCallback;
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartHandle_rx = UART_open(CONFIG_UART_0, &uartParams);
    /*if (uartHandle_rx)
     {
     UART_read(uartHandle_rx, &uartRxBuf, 1);
     }*/

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

    // Configuracions modul RF
    GPIO_write(CE_RF, 0);

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

    // NEW: Count up to 1 second
    static uint16_t tickCounter = 0;
    tickCounter++;

    if (tickCounter == 20000)
    {
        //Semaphore_post(semaphore_test);
        tickCounter = 0;      // Reset the counter for the next second
    }
}

// HWI de recepcio del modul RF
void HwiRF(uint_least8_t index)
{
    Semaphore_post(semaphore_RF_irq);
}

void TaskIMU(UArg arg1, UArg arg2)
{

    // Esperem a que el SPI estigui preparat, per tenir els motors apagats
    Semaphore_pend(semaphore_SPI_ready, BIOS_WAIT_FOREVER);

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

    float angle = 0.0f;
    float currentAngle = 0.0f;
    IMU_Message sendToPID;
    int posNew = 0, posOldest = 1;
    //int N = 300;
    float angles[N];
    // Inicialitzacio timestamp
    float dt = 0.0f;
    Types_FreqHz freq;
    Timestamp_getFreq(&freq);
    float cpuFreq = (float) freq.lo;
    uint32_t oldTimeMeas = Timestamp_get32();

    IMU_data currentData;

    System_printf("Abans de calibrar \n");
    System_flush();

    // Give the user time to hold the beam level before sampling starts,
    // counting down over UART so it's visible on the terminal being watched.
    for (int c = 5; c > 0; c--)
    {
        snprintf(uartTxBuf, sizeof(uartTxBuf), "Calibrating in %d... hold it level!\n", c);
        UART_write(uartHandle_rx, uartTxBuf, strlen(uartTxBuf));
        Task_sleep(1000);
    }

    snprintf(uartTxBuf, sizeof(uartTxBuf), "Calibrating now, keep it still (~1s)...\n");
    UART_write(uartHandle_rx, uartTxBuf, strlen(uartTxBuf));

    // Calibracio IMU (assumes it is being held level / 0 degrees right now)
    imu.calibrate(1000);

    snprintf(uartTxBuf, sizeof(uartTxBuf), "Calibration done!\n");
    UART_write(uartHandle_rx, uartTxBuf, strlen(uartTxBuf));

    System_printf("Despres de calibrar \n");
    System_flush();

    // Inicialitzacio vector promig
    for (int i = 0; i < N; i++)
    {
        uint32_t newTimeMeas = Timestamp_get32();
        uint32_t dTicks = newTimeMeas - oldTimeMeas;
        dt = (float) dTicks / cpuFreq;
        oldTimeMeas = newTimeMeas;

        if(imu.read()){
        currentData = imu.getData();
        angle = IMU2angle(currentData, angle, dt);
        angles[i] = angle;
        }
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

            angles[posNew] = IMU2angle(currentData, angle, dt) + angleOffset;

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
            //snprintf(uartTxBuf, sizeof(uartTxBuf), "wangX = %d | ", currentData.gyX);
            //UART_write(uartHandle_rx, uartTxBuf, strlen(uartTxBuf));
            sendToPID.angle = constrainAngle(angle);
            sendToPID.wang = ((float) currentData.gyY) / GY_CTT;

            //snprintf(uartTxBuf, sizeof(uartTxBuf), "angle = %.2f | ", angle);

            //UART_write(uartHandle_rx, uartTxBuf, strlen(uartTxBuf));

            // CODI BO
            Mailbox_post(mailbox_imu, &sendToPID, BIOS_NO_WAIT);
            Event_post(event_data, FLAG_IMU);
        }
        else
        {
            //System_printf("IMU READING FAILED");
            //System_flush();
        }
    }
}

void task_readUART(UArg arg1, UArg arg2)
{
    char singleRxChar;
    char uartRxBufLocal[32]; // Local buffer for the command string
    uint8_t localRxIndex = 0;

    while (1)
    {
        // Block and wait here until 1 character is received
        UART_read(uartHandle_rx, &singleRxChar, 1);

        // Assign the received character to your data variable
        char data = singleRxChar;

        // Process the character
        if (data == '\n' || data == '\r')
        {
            if (localRxIndex > 0)
            {
                uartRxBufLocal[localRxIndex] = '\0'; // Null-terminate safely

                // Commands understood:
                //   "angle"        -> print the current PID telemetry once
                //   "angle <val>"  -> directly set the target angle (goalCons)
                if (strncmp(uartRxBufLocal, "angle", 5) == 0)
                {
                    char *args = uartRxBufLocal + 5;
                    while (*args == ' ')
                    {
                        args++; // skip separating whitespace, if any
                    }

                    if (*args == '\0')
                    {
                        // No value given -> request a one-shot telemetry print
                        Event_post(event_data, FLAG_PRINT);
                    }
                    else
                    {
                        // Value given, in real-world degrees -> convert to
                        // the internal (PID-facing) scale before storing it
                        goalCons = (float) atof(args) * angleScale;
                    }
                }

                localRxIndex = 0; // Reset index for the next command
            }
        }
        else if (data == '\b' || data == 127)
        {
            if (localRxIndex > 0)
            {
                localRxIndex--;
            }
        }
        else
        {
            // Accumulate any other printable character (letters, digits, '.', '-', spaces)
            if (localRxIndex < sizeof(uartRxBufLocal) - 1)
            {
                uartRxBufLocal[localRxIndex++] = data;
            }
        }
    }
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

    Semaphore_pend(semaphore_SPI_ready, BIOS_WAIT_FOREVER);

    //float currentCons = 0.0f;
    uint8_t currentCons_int = 128;
    float currentCons = 0.0f;
    float currentAngle = 0.0f;
    IMU_Message receivedData;

    System_printf("Entrem a PID \n");
    System_flush();

// Inicialitzacio timestamp
    float dt = 0.006f;
    Types_FreqHz freq;
    Timestamp_getFreq(&freq);
    float cpuFreq = (float) freq.lo;
    uint32_t oldTimeMeas = Timestamp_get32();

// Constants PID

    // oscil�la una mica a kp=28
    // Si agafem mesura wang, kd ha de ser negativa!

    //float Kp = 20.0f;
    //float Ki = 20.0f;
    //float Kd = -9.0f; // fem -30.0f


    //float Kp = 27.5f;
    //float Ki = 28.7f;
    //float Kd = -16.0f; // fem -30.0f

    float integral = 0.0f;
    float restAngle = 0.0f;
    Mailbox_pend(mailbox_cons, &restAngle, BIOS_NO_WAIT);
    float last_error = -restAngle;
    int16_t output = 0;

    while (1)
    {
        UInt events = Event_pend(event_data, Event_Id_NONE,
        FLAG_IMU | FLAG_CONS | FLAG_PRINT,
                                 BIOS_WAIT_FOREVER);

        if (events & FLAG_CONS)
        {
            // Update target from mailbox
            Mailbox_pend(mailbox_cons, &currentCons_int, BIOS_NO_WAIT);

            // Radio
            //goalCons = ((float) currentCons_int) / 127.0f * 25.0f;

            // UART, 0.8
            goalCons = ( ((float) currentCons_int) / 255.0f * 40.0f - 20.0f );

            __nop();
        }

        if (events & FLAG_IMU)
        {
            // Read IMU angle from mailbox
            if (Mailbox_pend(mailbox_imu, &receivedData, BIOS_NO_WAIT))
            {

                /*System_printf("Calculem PID \n");
                 System_flush();*/

                if (goalCons > currentCons)
                    currentCons += consSlewRate;
                if (goalCons < currentCons)
                    currentCons -= consSlewRate;

                if (Semaphore_pend(semaphore_test, BIOS_NO_WAIT))
                {
                    goalCons = 0.0f;
                }

                // Cal calcular dt

                uint32_t newTimeMeas = Timestamp_get32();
                uint32_t dTicks = newTimeMeas - oldTimeMeas;
                dt = (float) dTicks / cpuFreq;
                oldTimeMeas = newTimeMeas;

                //float error = currentCons - currentAngle;
                float error = currentCons - receivedData.angle;
                integral += error * dt;
                intMin = -intMax/intFrac;//
                if (integral > intMax)
                    integral = intMax;
                if (integral < intMin)
                    integral = intMin;

                float derivative = receivedData.wang; //(error - last_error) / dt;
                //float der_old = (error - last_error) / dt;
                output = (int16_t) ((Kp * error) + (Ki * integral)
                        + (Kd * derivative));
                last_error = error;

                // Send computed value to SPI TX task
                Mailbox_post(mailbox_pid, &output, BIOS_NO_WAIT);
                //Mailbox_post(mailbox_pid, &currentCons, BIOS_NO_WAIT);
            }
        }

        if (events & FLAG_PRINT)
        {
            // One-shot telemetry dump, triggered by sending "angle" (no value) over UART.
            // setpoint/angle are converted back from internal PID units to
            // real-world degrees (inverse of the scaling applied on input).
            snprintf(
                    uartTxBuf,
                    sizeof(uartTxBuf),
                    "setpoint = %.2f | angle = %.2f | wang = %.3f | dt = %.5f | PID = %d \n",
                    currentCons / angleScale, receivedData.angle / angleScale, receivedData.wang, dt, output);

            UART_write(uartHandle_rx, uartTxBuf, strlen(uartTxBuf));
        }
    }
}

void TaskRF(UArg arg1, UArg arg2)
{
    SPI_Handle spiHandle;

    Mailbox_pend(mailbox_spiHandle, &spiHandle, BIOS_WAIT_FOREVER);

    // Inicialitzem modul RF
    NRF24L01 modulRF;

    modulRF.init(spiHandle, semaphore_SPI_available);
    modulRF.powerUP(RTXmode);

    // --- DIAGNOSTIC CHECK START ---
    uint8_t check_rf_ch = 0;
    uint8_t check_setup = 0;

    // Read the RF_CH register (You set it to 0x07 in init)
    modulRF.readReg(nrf24l01_RF_CH, &check_rf_ch, 1);

    // Read SETUP_AW (Address Width, you set it to 0x01 in init)
    modulRF.readReg(nrf24l01_SETUP_AW, &check_setup, 1);

    // --- DIAGNOSTIC CHECK END ---

    System_printf("RF initialised \n");
    System_flush();

    Task_sleep(5);

    GPIO_setCallback(INT_RF, HwiRF);
    GPIO_enableInt(INT_RF);
    GPIO_write(CE_RF, 1); // Comencem a escoltar

    while (1)
    {
        Semaphore_pend(semaphore_RF_irq, BIOS_WAIT_FOREVER);
        int8_t setpoint = modulRF.readData();

        /*if (modulRF.isDataAvailable())
         {
         uint8_t setpoint = modulRF.readData();
         System_printf("RECEIVED! \n");
         System_flush();
         }*/

        System_printf("RECEIVED! setpoint = %d \n", setpoint);
        System_flush();

        Mailbox_post(mailbox_cons, &setpoint, BIOS_NO_WAIT);
        Event_post(event_data, FLAG_CONS);

        Task_sleep(10);
    }
}

void TaskSPI_TX(UArg arg1, UArg arg2)
{
    int8_t sendOutput[2];
    ConsData pidOutput;
    uint32_t spiReceive;
    ConsData LeftSpeed = 0;
    ConsData RightSpeed = 0;

// Condicionament variable
    //int16_t baseSpeedRight = 2400;
    //int16_t baseSpeedLeft = 2650;

    SPI_init();
    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 100000;
    spiParams.dataSize = 8;

    SPI_Handle spiHandle = SPI_open(CONFIG_SPI_0, &spiParams);
    if (!spiHandle)
    {
        System_printf("Error: SPI Init Failed\n");
        System_flush();
        Task_exit();
    }

    SPI_Transaction transaction = { 0 }; // MUST 0-initialize to prevent driver crash
    transaction.count = 2;
    transaction.txBuf = (void*) &sendOutput;
    transaction.rxBuf = (void*) &spiReceive;

    sendOutput[0] = 0; // Apaguem motors inicialment per permetre configuracio IMU
    sendOutput[1] = 0;
    GPIO_write(SS_L, 0); // Manual CS Low
    GPIO_write(SS_R, 0); // Manual CS Low
    SPI_transfer(spiHandle, &transaction);
    GPIO_write(SS_L, 1); // Manual CS High
    GPIO_write(SS_R, 1); // Manual CS High

    Task_sleep(5000);

    Semaphore_post(semaphore_SPI_ready);
    Semaphore_post(semaphore_SPI_ready); // Fem el post dos cops, per task_imu i task_pid


    Mailbox_post(mailbox_spiHandle, &spiHandle, BIOS_WAIT_FOREVER);


    Task_sleep(2000);

    int commute = 0;

    System_printf("SPI done \n");
    System_flush();

    while (1)
    {
        Mailbox_pend(mailbox_pid, &pidOutput, BIOS_WAIT_FOREVER);
        //pidOutput = 0;
        //baseSpeed = 2500;

        /*System_printf("L: %d | R: %d | PID: %d \n",
         constrainSpeed(baseSpeed + pidOutput),
         constrainSpeed(baseSpeed - pidOutput), pidOutput);
         System_flush();*/

        LeftSpeed = constrainSpeed(baseSpeed + pidOutput + offset);
        RightSpeed = constrainSpeed(baseSpeed - pidOutput - offset);
        LeftSpeed = (int16_t) (propLeft * ( (float) LeftSpeed));

        //snprintf(uartTxBuf, sizeof(uartTxBuf), "PID: %d \n", pidOutput);

        //UART_write(uartHandle_rx, uartTxBuf, strlen(uartTxBuf));

        // Bloquegem comm SPI per motors
        Semaphore_pend(semaphore_SPI_available, BIOS_WAIT_FOREVER);

        GPIO_write(SS_L, 0); // Manual CS Low
        sendOutput[0] = (LeftSpeed >> 8) & 0xFF; // Primer byte
        sendOutput[1] = LeftSpeed & 0xFF; // Segon byte
        SPI_transfer(spiHandle, &transaction);
        GPIO_write(SS_L, 1); // Manual CS High

        //Task_sleep(10);

        GPIO_write(SS_R, 0); // Manual CS Low
        sendOutput[0] = (RightSpeed >> 8) & 0xFF; // Primer byte
        sendOutput[1] = RightSpeed & 0xFF; // Segon byte
        SPI_transfer(spiHandle, &transaction);
        GPIO_write(SS_R, 1); // Manual CS High

        // Alliberem SPI
        Semaphore_post(semaphore_SPI_available);
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
    float acc_angle = atan2f((float) IMUdata.accX, (float) IMUdata.accZ)
            * (180.0f / 3.14159265f);
    float newAngle = alpha * (prevAngle + ((float) IMUdata.gyY) / GY_CTT * dt)
            + (1.0f - alpha) * acc_angle;
    return newAngle;
}

int16_t constrainSpeed(int16_t inputVal)
{
    //int16_t minVal = 1200; // 300
    //int16_t maxVal = 3400; // 3200
    int16_t minVal = minSpeed; // 300
    int16_t maxVal = maxSpeed; // 3200

    if (inputVal < minVal)
        return minVal;
    if (inputVal > maxVal)
        return maxVal;
    return inputVal;
}


float constrainAngle(float inputVal)
{
    float minVal = -90.0f;
    float maxVal = 90.0f;

    if (inputVal < minVal)
        return minVal;
    if (inputVal > maxVal)
        return maxVal;
    return inputVal;
}
