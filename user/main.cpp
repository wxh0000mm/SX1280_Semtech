/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: Main program

Maintainer: Gregory Cristian & Gilbert Menth
*/

#include "mbed.h"
#include "Timers.h"
#include "ywd.h"

#define PING_PONG_STATE 0

// 定义stdio uart 参数
extern serial_t     stdio_uart; 
extern int          stdio_uart_inited; 


int main( )
{
    unsigned char currentstate=0;
	
	serial_init(&stdio_uart, PA_2, PA_3);  //重定向到 Serial1，也可以重定向到 Serial2
    stdio_uart_inited = 1; 
	
	baud( 115200 );


    ywdInit( );
	InitDemoApplication();

    TimersInit( );
   

    printf( "Start SX1280DevKit : %s\n\r", FIRMWARE_VERSION );

    while( 1 )
    {
    

        switch( currentstate )
        {
      
            case PING_PONG_STATE:
                RunDemoApplicationPingPong( );
                break;


            default:    // Any page not running a demo
                break;
        }
    }
}
