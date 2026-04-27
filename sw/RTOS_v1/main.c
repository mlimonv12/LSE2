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
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Board.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>

/* My Board */
#include "ti_drivers_config.h"

/*prototips funcions*/

void Wait_cycles(uint32_t numberCycles);

// HWI per quan es premen els botons
void SW1Pressed(UArg arg);
void SW2Pressed(UArg arg);

// SWI per dur a terme el calcul
void countPresses(UArg arg0, UArg arg1);

// Task per mostrar el missatge per pantalla
void displayInfo(UArg arg0, UArg arg1);

// Semafor per controlar la tasca d'impressio
extern Semaphore_Handle semConsole;

extern Swi_Handle swiCalc;


/* Variables globals */
volatile int comptador = 0;
volatile int pressed = 0;

/*
 *  ======== main ========
 */
int main()
{
    /* Call driver init functions */
    Board_init();
    GPIO_init();

    System_printf("Iniciem programa \n");
    System_flush();

    GPIO_setCallback(SW1, SW1Pressed);
    GPIO_setCallback(SW2, SW2Pressed);

    GPIO_enableInt(SW1);
    GPIO_enableInt(SW2);

    BIOS_start();

    return (0);
}


void SW1Pressed(UArg arg) {
    pressed = 1;
    Swi_post(swiCalc);
}

void SW2Pressed(UArg arg) {
    pressed = 2;
    Swi_post(swiCalc);
}



// SWI per dur a terme el calcul
void countPresses(UArg arg0, UArg arg1) {

    switch (pressed) {
    case 1:
        comptador++;
        break;
    case 2:
        comptador--;
        break;
    }

    // Habilitem la tasca
    Semaphore_post(semConsole);
}

// Task per mostrar el missatge per pantalla
void displayInfo(UArg arg0, UArg arg1) {

    while (1) {
        // La tasca està pendent de ser habilitada
        Semaphore_pend(semConsole, BIOS_WAIT_FOREVER);

        // Commutem LED
        GPIO_toggle(LED_out);

        // Imprimim info
        System_printf("Comptador = S1 - S2 = %d \n", comptador);

        // Sortida a consola
        System_flush();

    }



}

// Funcio idle per commutar el LED
void commuteLED(void);


void Wait_cycles(uint32_t numberCycles)
{
    volatile uint32_t i = 0; //variable comptador
    while (i < numberCycles)
    {
        i++;
    } //fem bucle buit
}
