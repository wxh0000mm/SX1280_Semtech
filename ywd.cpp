/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: EEPROM routines

Maintainer: Gregory Cristian & Gilbert Menth
*/

#include "mbed.h"
#include "string.h"
#include "ywd.h"
#include "DemoApplication.h"
#include "sx1280.h"

/*!
 * \brief Local copy of ywd.
 */
ywdData_t ywd;

void ywdInit( void )
{
   
  	ywdSetDefaultSettings( );
  
}


void ywdSetDefaultSettings( void )
{

    printf("Set Default Settings\n\r");

   
	ywd.DemoSettings.ModulationType = PACKET_TYPE_LORA;
    ywd.ModulationParams.PacketType = PACKET_TYPE_LORA;
    ywd.PacketParams.PacketType     = PACKET_TYPE_LORA;

    ywd.DemoSettings.ModulationParam1 = LORA_SF10;
    ywd.DemoSettings.ModulationParam2 = LORA_BW_1600;
    ywd.DemoSettings.ModulationParam3 = LORA_CR_4_5;

    ywd.DemoSettings.PacketParam1 = 12; // PreambleLength
    ywd.DemoSettings.PacketParam2 = LORA_PACKET_VARIABLE_LENGTH;
    ywd.DemoSettings.PacketParam3 = 30;
    ywd.DemoSettings.PacketParam4 = LORA_CRC_ON;
    ywd.DemoSettings.PacketParam5 = LORA_IQ_NORMAL;
	
	ywd.DemoSettings.Entity         = MASTER;
    ywd.DemoSettings.AntennaSwitch  = 0x00;
    ywd.DemoSettings.RadioPowerMode = USE_DCDC;
    ywd.DemoSettings.Frequency      = DEMO_CENTRAL_FREQ_PRESET1;
    ywd.DemoSettings.TxPower        = DEMO_POWER_TX_MAX;
    ywd.DemoSettings.MaxNumPacket   = 0x00; // infinite
    ywd.DemoSettings.ModulationType = PACKET_TYPE_LORA;

}

