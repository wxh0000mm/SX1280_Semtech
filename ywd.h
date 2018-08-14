/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: ywd routines header

Maintainer: ywd
*/

#ifndef YWD_H
#define YWD_H


#include "DemoApplication.h"
#include "sx1280.h"




/*!
 * \brief Part of ywd to save or restore
 */
typedef enum
{
    ALL_DATA,
    SCREEN_DATA,
    DEMO_SETTINGS,
    RADIO_LORA_PARAMS,
    RADIO_RANGING_PARAMS,
    RADIO_FLRC_PARAMS,
    RADIO_GFSK_PARAMS,
    RADIO_BLE_PARAMS
}ywdDataSet_t;

/*!
 * \brief ywdData structure
 */
typedef struct
{
    DemoSettings_t DemoSettings;
    ModulationParams_t ModulationParams;
    PacketParams_t PacketParams;
    uint16_t CheckSum;
}ywdData_t;

/*!
 * \brief Local copy of ywd.
 */
extern ywdData_t ywd;

/*!
 * \brief Initialises the contents of EepromData
 */
void ywdInit( void );



/*!
 * \brief Initialises the contents of flash to default values & save to EEPROM
 */
void ywdSetDefaultSettings( void );


#endif
