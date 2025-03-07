/*
 * \file        spi3w-ard-ino.cpp
 * \brief       Arduino 3wire SPI cover
 * \author      Infineon Technologies AG
 * \version     4.0.0
 * \copyright   2020-2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: MIT
 */

#include <SPI.h>
#include "spi3w-ino.hpp"
#include "pal-pin-types.hpp"

#if (SPI3W_INO == SPI3W_ARD)

using namespace tle5012;

/**
 * @addtogroup arduinoPal
 * @{
 *
 * @brief Arduino SPIClass extension to use 3wire SSC SPI interfaces
 */


 /**
 * @brief Construct a new SPIClass3W::SPIClass3W object
 *
 */

#if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
// This part is yet to be implemented
SPIClass3W::SPIClass3W(uint8_t spiNum):ArduinoSPI(MISO,MOSI,SCK,MODE_SPI)
#elif defined(ARDUINO_ARCH_RP2040)
// This part is yet to be implemented
SPIClass3W::SPIClass3W(uint8_t spiNum):MbedSPI(MISO,MOSI,SCK)
#elif defined(ARDUINO_ARCH_SAMD)
// This part is yet to be implemented
SPIClass3W::SPIClass3W(uint8_t spiNum):SPIClassSAMD(&PERIPH_SPI ,MISO,SCK,MOSI, PAD_SPI_TX,  PAD_SPI_RX)
#else
SPIClass3W::SPIClass3W(uint8_t spiNum):SPIClass()
#endif
{
    this->mCS = SS;
    this->mMISO = MISO;
    this->mMOSI = MOSI;
    this->mSCK = SCK;
    this->mSpiNum = spiNum;
}

/*!
 * @brief New for fetching SPI parameter
 *
 * @param miso [in] pin number for miso, on sensor2go boards the same than mosi
 * @param mosi [in] pin number for mosi, on sensor2go boards the same than miso
 * @param sck [in] the system clock pin for external clock driver
 * @param cs [in] chip select pin, up to four different cs pins can be used together with the slave number
 */
void SPIClass3W::begin(uint8_t miso, uint8_t mosi, uint8_t sck, uint8_t cs)
{
    this->mMOSI = mosi;
    this->mMISO = miso;
    this->mSCK = sck;
    setCSPin(cs);
    pinMode(this->mCS,OUTPUT);
    digitalWrite(this->mCS, HIGH);
    #if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
        ArduinoSPI::begin();
        Serial.println("Arduino Uno R4 3-wire SPI is yet not working");
    #elif defined(ARDUINO_ARCH_RP2040)
        MbedSPI::begin();
        Serial.println("Arduino Uno Nano RP2040 3-wire SPI is yet not working");
    #elif defined(ARDUINO_ARCH_SAMD)
        SPIClassSAMD::begin();
        Serial.println("Arduino SAMD 3-wire SPI is yet not working");
    #else
        SPIClass::begin();
    #endif
}


/**
 * @brief Destroy the SPIClass3W::SPIClass3W object
 *
 */
SPIClass3W::~SPIClass3W()
{
}

/*!
 * @brief Set the chip select pin. This function is needed if more than one
 * sensor is in the SPI bus
 *
 * @param cs [in] Pin number of the sensors chip select
 */
void SPIClass3W::setCSPin(uint8_t cs)
{
    this->mCS = cs;
}

/*!
 * @brief Main SPI three wire communication functions for sending and receiving data
 *
 * @param sent_data pointer two 2*unit16_t value for one command word and one data word if something should be written
 * @param size_of_sent_data the size of the command word default 1 = only command 2 = command and data word
 * @param received_data pointer to data structure buffer for the read data
 * @param size_of_received_data size of data words to be read
 */
void SPIClass3W::sendReceiveSpi(uint16_t* sent_data, uint16_t size_of_sent_data, uint16_t* received_data, uint16_t size_of_received_data)
{
    uint32_t data_index = 0;
    //send via TX
    digitalWrite(this->mCS, LOW);
    pinMode(this->mMISO,INPUT);
    pinMode(this->mMOSI,OUTPUT);
    beginTransaction(SPISettings(SPEED,MSBFIRST,SPI_MODE1));

    for(data_index = 0; data_index < size_of_sent_data; data_index++)
    {
        received_data[0] = transfer16(sent_data[data_index]);
    }
    // Some MCUs require a delay and CS high to low between the last byte and the CS high
    // digitalWrite(this->mCS, HIGH);

    // receive via RX
    // digitalWrite(this->mCS, LOW);
    pinMode(this->mMISO,OUTPUT);
    pinMode(this->mMOSI,INPUT);
    delayMicroseconds(5);

    for(data_index = 0; data_index < size_of_received_data; data_index++)
    {
        received_data[data_index] = transfer16(0x0000);
    }

    endTransaction();
    digitalWrite(this->mCS, HIGH);
}


// #endif


/** @} */

#endif /* SPI3W_INO */
