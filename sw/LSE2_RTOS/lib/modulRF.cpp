/*
 * imu.c
 *
 *  Created on: 6 May 2026
 *      Author: Miquel Limon Valles
 */


#include "modulRF.h"
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h"
#include <ti/drivers/SPI.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>


#define NULL 0
#define RF_CH 0x07 // RF Chanel, to choose by the user

NRF24L01::NRF24L01(){
}


//-----------------------------------------------------------------------------------------------------------//
bool NRF24L01::init(SPI_Handle spiHandle, Semaphore_Handle sem_handle)
{
    handle = spiHandle;
    mutex = sem_handle;
    uint8_t data;

    data = nrf24l01_EN_AA_ENAA_P0;
    writeReg(nrf24l01_EN_AA, &data, 1);

    data = nrf24l01_EN_RXADDR_ERX_P0;
    writeReg(nrf24l01_EN_RXADDR, &data, 1);

    data = 0x01; // 3-byte address width
    writeReg(nrf24l01_SETUP_AW, &data, 1);

    uint8_t addr[3] = {0xE7, 0xE7, 0xE7};
    writeReg(nrf24l01_RX_ADDR_P0, addr, 3);
    writeReg(nrf24l01_TX_ADDR, addr, 3); // Good practice to set both

    data = (0x03<<4) | 0x08;
    writeReg(nrf24l01_SETUP_RETR, &data, 1);

    data = 0x07; // RF_CH
    writeReg(nrf24l01_RF_CH, &data, 1);

    data = 0x06; // 1 Mbps, 0dBm (Must match TX exactly)
    writeReg(nrf24l01_RF_SETUP, &data, 1);

    data = nrf24l01_STATUS_RX_DR | nrf24l01_STATUS_TX_DS | nrf24l01_STATUS_MAX_RT;
    writeReg(nrf24l01_STATUS, &data, 1);

    // --- STATIC PAYLOAD CONFIGURATION ---
    data = 0x00;
    writeReg(nrf24l01_FEATURE, &data , 1);  // Disable DPL
    writeReg(nrf24l01_DYNPD, &data, 1);

    data = 1;
    writeReg(nrf24l01_RX_PW_P0, &data, 1); // Expect exactly 1 byte
    // ------------------------------------

    flush_tx();
    flush_rx();

    return true;
}


void NRF24L01::SPI_send(uint8_t count, uint8_t *TxBuf, uint8_t *RxBuf) {
    SPI_Transaction spi_transaction = {0};
    spi_transaction.count = count;
    spi_transaction.txBuf = (void *)TxBuf;
    spi_transaction.rxBuf = (void *)RxBuf;

    // Perform the actual TI-RTOS hardware transfer
    SPI_transfer(handle, &spi_transaction);
}


void NRF24L01::powerUP(uint8_t mode)
{
    uint8_t data;
    if(mode==PTXmode){
//      data=nrf24l01_CONFIG_MASK_TX_DS|nrf24l01_CONFIG_MASK_MAX_RT|(0x01<<3)|(0x01<<2)|(0x01<<1)|(0x00);  //EN_CRC, CRC 2 Bytes, POWER_UP, TX
        data=nrf24l01_CONFIG_MASK_RX_DR|(0x01<<3)|(0x01<<2)|(0x01<<1)|(0x00);  //EN_CRC, CRC 2 Bytes, POWER_UP, TX
    } else if(mode==RTXmode){
//      data=nrf24l01_CONFIG_MASK_RX_DR|(0x01<<3)|(0x01<<2)|(0x01<<1)|(0x01);  //EN_CRC, CRC 2 Bytes, POWER_UP, RX
        data=nrf24l01_CONFIG_MASK_TX_DS|nrf24l01_CONFIG_MASK_MAX_RT|(0x01<<3)|(0x01<<2)|(0x01<<1)|(0x01);  //EN_CRC, CRC 2 Bytes, POWER_UP, RX
    } else{
        while(1); //error
    }
    writeReg(nrf24l01_CONFIG, &data, 1);
}

void NRF24L01::powerDown(void)
{
    uint8_t data;
    readReg(nrf24l01_CONFIG, &data, 1); //llegim conf actual

    if((data & nrf24l01_CONFIG_PWR_UP) == 0)
        return;

    data &= ~nrf24l01_CONFIG_PWR_UP;

    writeReg(nrf24l01_CONFIG, &data, 1);
}

//-----------------------------------------------------------------------------------------------------------//
/*uint8_t NRF24L01::writeReg(uint8_t regnumber, uint8_t * data, uint8_t len)
{
    uint8_t status;
    uint8_t dataR;

    Semaphore_pend(mutex, BIOS_WAIT_FOREVER);

    dataR=regnumber | nrf24l01_W_REGISTER; //volem escriure
    //baixem CS

    GPIO_write(SS_RF, 0);
    SPI_send(1, &dataR, &status);
    //ara enviem les dades
    SPI_send(len, data, NULL);
    //pujem CS
    GPIO_write(SS_RF, 1);

    Semaphore_post(mutex); // Unlock bus

    return status;
}

uint8_t NRF24L01::readReg(uint8_t regnumber, uint8_t * data, uint8_t len){
    uint8_t status;
    //baixem CS

    Semaphore_pend(mutex, BIOS_WAIT_FOREVER);
    GPIO_write(SS_RF, 0);
    SPI_send(1, &regnumber, &status); //escribim
    //ara enviem les dades
    SPI_send(len, NULL, data); //llegim
    //pujem CS
    GPIO_write(SS_RF, 1);

    Semaphore_post(mutex);

    return status;
}*/

uint8_t NRF24L01::writeReg(uint8_t regnumber, uint8_t * data, uint8_t len)
{
    uint8_t txBuf[10]; // Buffer large enough for cmd + payload
    uint8_t rxBuf[10];

    txBuf[0] = regnumber | nrf24l01_W_REGISTER; // Command byte
    for(int i = 0; i < len; i++) {
        txBuf[i+1] = data[i];                   // Data bytes
    }

    Semaphore_pend(mutex, BIOS_WAIT_FOREVER);
    GPIO_write(SS_RF, 0); // CS Low

    // Send everything in one single, uninterrupted transaction
    SPI_send(len + 1, txBuf, rxBuf);

    GPIO_write(SS_RF, 1); // CS High
    Semaphore_post(mutex);

    return rxBuf[0]; // Status is always returned on the first byte
}

uint8_t NRF24L01::readReg(uint8_t regnumber, uint8_t * data, uint8_t len)
{
    uint8_t txBuf[10];
    uint8_t rxBuf[10];

    txBuf[0] = regnumber; // Read command
    for(int i = 0; i < len; i++) {
        txBuf[i+1] = 0xFF; // Send NOPs (0xFF) to clock out the data
    }

    Semaphore_pend(mutex, BIOS_WAIT_FOREVER);
    GPIO_write(SS_RF, 0); // CS Low

    // Send command and clock out data in one transaction
    SPI_send(len + 1, txBuf, rxBuf);

    GPIO_write(SS_RF, 1); // CS High
    Semaphore_post(mutex);

    // Extract the actual data we requested
    for(int i = 0; i < len; i++) {
        data[i] = rxBuf[i+1];
    }

    return rxBuf[0];
}

bool NRF24L01::isDataAvailable(void)
{
    uint8_t status_reg = 0;

    // Read the 1-byte STATUS register
    readReg(nrf24l01_STATUS, &status_reg, 1);

    // Check if the 6th bit (RX_DR) is set
    if (status_reg & nrf24l01_STATUS_RX_DR)
    {
        return true; // Data is ready!
    }

    return false; // No new data
}


uint8_t NRF24L01::readData(void)
{
    uint8_t rxData = 0;
    uint8_t cmd = nrf24l01_R_RX_PAYLOAD;
    uint8_t status;

    Semaphore_pend(mutex, BIOS_WAIT_FOREVER);
    GPIO_write(SS_RF, 0);

    SPI_send(1, &cmd, &status);       // Send read payload command
    //int dummy = 0xFF;
    SPI_send(1, 0, &rxData);       // Clock out 1 byte of payload data

    GPIO_write(SS_RF, 1);
    Semaphore_post(mutex);

    // Clear the RX_DR flag in the STATUS register to clear the interrupt
    uint8_t clear_flag = nrf24l01_STATUS_RX_DR;
    writeReg(nrf24l01_STATUS, &clear_flag, 1);

    return rxData;
}


uint8_t NRF24L01::flush_tx(void)
{
    uint8_t status,data;
    data=nrf24l01_FLUSH_TX;

    Semaphore_pend(mutex, BIOS_WAIT_FOREVER);

    //baixem CS
    GPIO_write(SS_RF, 0);
    SPI_send(1, &data, &status); //escribim
    GPIO_write(SS_RF, 1);

    Semaphore_post(mutex);

    return status;
}
uint8_t NRF24L01::flush_rx(void)
{
    uint8_t status, data;
    data=nrf24l01_FLUSH_RX;
    //baixem CS

    Semaphore_pend(mutex, BIOS_WAIT_FOREVER);

    GPIO_write(SS_RF, 0);
    SPI_send(1, &data, &status); //escribim
    GPIO_write(SS_RF, 1);


    Semaphore_post(mutex);

    return status;
}






uint8_t NRF24L01::nop(void)
{
    uint8_t status, data;
    data=nrf24l01_NOP; //comandes SPI
    //baixem CS
    GPIO_write(SS_RF, 0);
    SPI_send(1, &data, &status); //escribim
    GPIO_write(SS_RF, 1);

    return status;
}

/*
void nRF24L01_clear_IRQ(void)
{
    uint8_t data;
    data=nrf24l01_STATUS_RX_DR|nrf24l01_STATUS_TX_DS|nrf24l01_STATUS_MAX_RT;
    writeReg(nrf24l01_STATUS, &data, 1);
}*/
