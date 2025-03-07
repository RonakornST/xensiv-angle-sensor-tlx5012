/**
 * \file        pal-pin-types.hpp
 * \brief       pin settings if not pins are defined
 * \author      Infineon Technologies AG
 * \version     4.0.0
 * \copyright   2020-2024 Infineon Technologies AG
 *
 * SPDX-License-Identifier: MIT
 */


#ifndef PAL_PIN_TYPES_HPP_
#define PAL_PIN_TYPES_HPP_

namespace tle5012
{

    /**
     * @addtogroup arduinoPal
     * @{
     * The pin definition here is only used when no PIN_SPI_xx is defined, otherwise the original setting is used.
     * Change them if you have other pin settings
     */
    #define UNUSED_PIN          0xFF


    /**
     * @brief Use the ALTERNATIVE_PINS macro to switch between different
     * default pin settings for the default SPI. This is ay needed for boards
     * which are not standard Arduino or Infineon XMC 
     * Use 0 for
     *  - default setting for all Infineon XMCs or PSoC boards
     *  - default setting for XMC 2go boards
     *  - default setting for all Arduino
     *  - for Arduino Nano classic with ATmega chipset
     * Use 1 for
     *  - ESP8266
     *  - ESP32 clones
     *  - NodeMCU
     *  - WEMOS mini and Lite Kits
     *  - MH-ET Live Kits
     * Use 2 for
     *  - pin settings based on Arduino pin names
     * Use 3 for
     *  - experimental settings including the spi3w-esp32.cpp file
     */
    #ifndef ALTERNATIVE_PINS
    #define ALTERNATIVE_PINS 0
    #endif


    #if (ALTERNATIVE_PINS == 0)
        /** 
         * @brief default pin setting is used no further define is needed 
         * - for all Infineon XMCs or PSoC boards
         * - for XMC 2go boards
         * - for all Arduino with classic ATmega chipset
         * - for all compatible boards using the same pin setting
         * 
         * #define SS     10
         * #define MISO   12
         * #define MOSI   11
         * #define SCK    13
         */
        #elif (ALTERNATIVE_PINS == 1)
        /**
         * @brief Use this pin setting for all Arduino EPS32 based clones including
         *  - ESP-WROM32
         *  - NodeMCU
         *  - ESP8266 and clones
        */
        #define SS     5
        #define MISO   19
        #define MOSI   23
        #define SCK    18
    #elif (ALTERNATIVE_PINS == 2)
        #define SS     D10
        #define MISO   D12
        #define MOSI   D11
        #define SCK    D13
    #elif (ALTERNATIVE_PINS == 3)
        #define SS     21
        #define MISO   47
        #define MOSI   38
        #define SCK    48
    #endif


    #ifndef PIN_SPI_EN
    #define PIN_SPI_EN          UNUSED_PIN  /*!< TLE5012 with any other PCB has no switch on/off */
    #endif

    #ifndef PIN_SPI_SS
    #define PIN_SPI_SS          SS  //D10
    #endif

    #ifndef PIN_SPI_MISO
    #define PIN_SPI_MISO        MISO //D12
    #endif

    #ifndef PIN_SPI_MOSI
    #define PIN_SPI_MOSI        MOSI //D11
    #endif

    #ifndef PIN_SPI_SCK
    #define PIN_SPI_SCK         SCK //D13
    #endif

}

#endif /** PAL_PIN_TYPES_HPP_ **/
