/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2016 Semtech

Description: PingPong, PER and Ranging demo implementation.

Maintainer: ywd
*/

#include "mbed.h"
#include <math.h>
#include "radio.h"
#include "sx1280-hal.h"
#include "ywd.h"
#include "DemoApplication.h"
#include "FreqLUT.h"

double t0 =       -0.016432807883697;                         // X0
double t1 =       0.323147003165358;                          // X1
double t2 =       0.014922061351196;                          // X1^2
double t3 =       0.000137832006285;                          // X1^3
double t4 =       0.536873856625399;                          // X2
double t5 =       0.040890089178579;                          // X2^2
double t6 =       -0.001074801048732;                         // X2^3
double t7 =       0.000009240142234;                          // X2^4



/*!
 * \brief Defines the local payload buffer size
 */
#define BUFFER_SIZE                     255

/*!
 * \brief Defines the size of the token defining message type in the payload
 *        cf. above.
 */
#define PINGPONG_SIZE                   4

/*!
 * \brief Define time used in PingPong demo to synch with cycle
 * RX_TIMEOUT_MARGIN is the free time between each cycle (time reserve)
 */
#define RX_TIMEOUT_MARGIN               50  // ms
#define RX_TX_TRANSITION_WAIT           5   // ms

/*!
 * \brief Size of ticks (used for Tx and Rx timeout)
 */
#define RX_TIMEOUT_TICK_SIZE            RADIO_TICK_SIZE_1000_US

#define RNG_TIMER_MS                    384 // ms
#define RNG_COM_TIMEOUT                 100 // ms


/*!
 * \brief Define the possible message type for the Ping-Pong and PER apps
 */
const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";



/*!
 * \brief Buffer and its size
 */
uint8_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

static uint8_t CurrentChannel;
static uint16_t MeasuredChannels;
int RngResultIndex;
double RawRngResults[DEMO_RNG_CHANNELS_COUNT_MAX];
double RssiRng[DEMO_RNG_CHANNELS_COUNT_MAX];


/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( void );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( IrqErrorCode_t );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRangingDone( IrqRangingCode_t );

/*!
 * \brief All the callbacks are stored in a structure
 */
RadioCallbacks_t Callbacks =
{
    &OnTxDone,        // txDone
    &OnRxDone,        // rxDone
    NULL,             // syncWordDone
    NULL,             // headerDone
    &OnTxTimeout,     // txTimeout
    &OnRxTimeout,     // rxTimeout
    &OnRxError,       // rxError
    &OnRangingDone,   // rangingDone
    NULL,             // cadDone
};

/*!
 * \brief Define IO and callbacks for radio
 * mosi, miso, sclk, nss, busy, dio1, dio2, dio3, rst, callbacks
 */
SX1280Hal Radio( PA_7, PA_6, PA_5, PA_4, PC_4, PC_5, NC, NC, PB_0, &Callbacks );


/*!
 * \brief Tx LED toggling on transmition success
 */
DigitalOut TX_LED( PD_2 );

/*!
 * \brief Rx LED toggling on reception success
 */
DigitalOut RX_LED( PD_2 );


Serial s( USBTX, USBRX );

/*!
 * \brief Mask of IRQs
 */
uint16_t IrqMask = 0x0000;

/*!
 * \brief Locals parameters and status for radio API
 * NEED TO BE OPTIMIZED, COPY OF STUCTURE ALREADY EXISTING
 */
PacketParams_t PacketParams;
PacketStatus_t PacketStatus;
ModulationParams_t ModulationParams;

/*!
 * \brief Flag to indicate if the demo is already running
 */
static bool DemoRunning = false;

/*!
 * \brief Flag holding the current internal state of the demo application
 */
static uint8_t DemoInternalState = APP_IDLE;

/*!
 * \brief Ticker for master to synch Tx frames. Flags for PER and PingPong demo
 * for Synch TX in cycle.
 */
Ticker SendNextPacket;
static bool SendNext = false;

/*!
 * \brief Hold last Rx packet number to compute PER in PER and PingPong demo
 */
static uint32_t PacketRxSequence = 0;
static uint32_t PacketRxSequencePrev = 0;


void LedBlink( void );
uint16_t GetTimeOnAir( uint8_t modulation );
void SendNextPacketEvent( void );
uint8_t CheckDistance( void );
void baud( int baudrate );



// ************************     Ping Pong Demo     *****************************
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************
void RunDemoApplicationPingPong( void )
{
    uint8_t i = 0;


    if( DemoRunning == false )
    {
        DemoRunning = true;
        TX_LED = 0;
        RX_LED = 0;
   		ywd.DemoSettings.CntPacketTx        = 0;
        ywd.DemoSettings.CntPacketRxOK      = 0;
        ywd.DemoSettings.CntPacketRxOKSlave = 0;
        ywd.DemoSettings.CntPacketRxKO      = 0;
        ywd.DemoSettings.CntPacketRxKOSlave = 0;
        ywd.DemoSettings.RxTimeOutCount     = 0;

		InitializeDemoParameters(ywd.DemoSettings.ModulationType);
		
        ywd.DemoSettings.InterPacketDelay = ( 2 * \
            GetTimeOnAir( ywd.DemoSettings.ModulationType ) ) + \
            RX_TIMEOUT_MARGIN;

        s.printf( "Start RunDemoApplicationPingPong.\n\r" );
        if( ywd.DemoSettings.Entity == MASTER )
        {
            DemoInternalState = SEND_PING_MSG;
            SendNextPacket.attach_us( &SendNextPacketEvent, \
                ( ywd.DemoSettings.InterPacketDelay * 1000 ) );
        }
        else
        {
            IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
            Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
            // Rx Single without timeout for the start
            RX_LED = !RX_LED;
            Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, 0x0000 } );
            DemoInternalState = APP_IDLE;
        }
    }

    if( ywd.DemoSettings.Entity == MASTER )
    {
        switch( DemoInternalState )
        {
            case SEND_PING_MSG:
                if( ( ywd.DemoSettings.MaxNumPacket != 0 ) \
                    && ( ywd.DemoSettings.CntPacketTx >= ywd.DemoSettings.MaxNumPacket ) )
                {
                    SendNextPacket.detach( );
                    SendNext = false;
                    RX_LED = 0;
                    TX_LED = 0;
                    DemoInternalState = APP_IDLE;
                    Radio.SetStandby( STDBY_RC );
                }
                else
                {
                    if( SendNext == true )
                    {
                        SendNext = false;

                        DemoInternalState = APP_IDLE;
                        ywd.DemoSettings.CntPacketTx++;
                        // Send the next PING frame
                        Buffer[0] = ( ywd.DemoSettings.CntPacketTx >> 24 ) & 0xFF;
                        Buffer[1] = ( ywd.DemoSettings.CntPacketTx >> 16 ) & 0xFF;
                        Buffer[2] = ( ywd.DemoSettings.CntPacketTx >> 8 )  & 0xFF;
                        Buffer[3] = ( ywd.DemoSettings.CntPacketTx & 0xFF );
                        Buffer[4] = PingMsg[0];
                        Buffer[5] = PingMsg[1];
                        Buffer[6] = PingMsg[2];
                        Buffer[7] = PingMsg[3];
                        for( i = 8; i < ywd.DemoSettings.PayloadLength; i++ )
                        {
                            Buffer[i] = i;
                        }
                        TX_LED = !TX_LED;
                        IrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
						
                        Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
//                        Radio.SendPayload( Buffer, ywd.DemoSettings.PayloadLength, \
//                                           ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, \
//                                           ywd.DemoSettings.InterPacketDelay - \
//                                           ( RX_TIMEOUT_MARGIN / 2 ) } );
						 Radio.SendPayload( Buffer, ywd.DemoSettings.PayloadLength, \
                                           ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, \
                                           10000} );
						
                    }
                }
                break;

            case APP_TX:
                DemoInternalState = APP_IDLE;
                s.printf("...ping\n");
			    TX_LED = !TX_LED;
                RX_LED = !RX_LED;
                IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
                Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, \
                             ywd.DemoSettings.InterPacketDelay - \
                             ( RX_TIMEOUT_MARGIN / 2 ) } );
                break;

            case APP_RX:
                RX_LED = !RX_LED;
                Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
                Radio.GetPacketStatus( &PacketStatus );
                if( ywd.ModulationParams.PacketType == PACKET_TYPE_LORA )
                {
                    ywd.DemoSettings.RssiValue = PacketStatus.LoRa.RssiPkt;
                    ywd.DemoSettings.SnrValue = PacketStatus.LoRa.SnrPkt;
                }
                else if( ywd.ModulationParams.PacketType == PACKET_TYPE_FLRC )
                {
                    ywd.DemoSettings.RssiValue = PacketStatus.Flrc.RssiSync;
                    ywd.DemoSettings.SnrValue = 0;
                }
                else if( ywd.ModulationParams.PacketType == PACKET_TYPE_GFSK )
                {
                    ywd.DemoSettings.RssiValue = PacketStatus.Gfsk.RssiSync;
                    ywd.DemoSettings.SnrValue = 0;
                }
                if( ( BufferSize >= PINGPONG_SIZE ) && ( strncmp( ( const char* )( Buffer + 8 ), ( const char* )PongMsg, PINGPONG_SIZE ) == 0 ) )
                {
                    ComputePingPongPayload( Buffer, BufferSize );
                }
                else
                {
                    ywd.DemoSettings.CntPacketRxKO++;
                }
                DemoInternalState = SEND_PING_MSG;
                break;

            case APP_RX_TIMEOUT:
            case APP_RX_ERROR:
                RX_LED = !RX_LED;
                ywd.DemoSettings.CntPacketRxKO++;
                DemoInternalState = SEND_PING_MSG;
                break;

            case APP_TX_TIMEOUT:
                s.printf( "Failure: timeout in Tx is shorter than the packet time on air\n\r" );
                DemoInternalState = APP_IDLE;
                break;

            case APP_IDLE: // do nothing
                break;

            default:
                break;
        }
    }
    else // SLAVE
    {
        switch( DemoInternalState )
        {
            case SEND_PONG_MSG:
                wait_ms( RX_TX_TRANSITION_WAIT );

                DemoInternalState = APP_IDLE;
                // Send the next PONG frame
                Buffer[0]  = ( ywd.DemoSettings.CntPacketTx >> 24 ) & 0xFF;
                Buffer[1]  = ( ywd.DemoSettings.CntPacketTx >> 16 ) & 0xFF;
                Buffer[2]  = ( ywd.DemoSettings.CntPacketTx >>  8 ) & 0xFF;
                Buffer[3]  = ( ywd.DemoSettings.CntPacketTx & 0xFF );
                Buffer[4]  = ( ( ywd.DemoSettings.CntPacketRxKO + \
                                 ywd.DemoSettings.RxTimeOutCount ) >> 24 ) & 0xFF;
                Buffer[5]  = ( ( ywd.DemoSettings.CntPacketRxKO + \
                                 ywd.DemoSettings.RxTimeOutCount ) >> 16 ) & 0xFF;
                Buffer[6]  = ( ( ywd.DemoSettings.CntPacketRxKO + \
                                 ywd.DemoSettings.RxTimeOutCount ) >> 8 ) & 0xFF;
                Buffer[7]  = ( ( ywd.DemoSettings.CntPacketRxKO + \
                                 ywd.DemoSettings.RxTimeOutCount ) & 0xFF );
                Buffer[8]  = PongMsg[0];
                Buffer[9]  = PongMsg[1];
                Buffer[10] = PongMsg[2];
                Buffer[11] = PongMsg[3];
                for( i = 12; i < ywd.DemoSettings.PayloadLength; i++ )
                {
                    Buffer[i] = i;
                }
                TX_LED = !TX_LED;
                IrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
				s.printf("pong...\n");
                Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                Radio.SendPayload( Buffer, ywd.DemoSettings.PayloadLength, \
                                   ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, \
                                   ywd.DemoSettings.InterPacketDelay } );
                break;

            case APP_TX:
                if( ( ywd.DemoSettings.MaxNumPacket != 0 ) \
                    && ( ( ywd.DemoSettings.CntPacketRxOK + ywd.DemoSettings.CntPacketRxKO + \
                           ywd.DemoSettings.RxTimeOutCount ) >= ywd.DemoSettings.MaxNumPacket ) )
                {
                    SendNextPacket.detach( ); 
                    SendNext = false;
                    RX_LED = 0;
                    TX_LED = 0;
                    DemoInternalState = APP_IDLE;
                    Radio.SetStandby( STDBY_RC );
                }
                else
                {
                    DemoInternalState = APP_IDLE;
                    TX_LED = !TX_LED;
                    RX_LED = !RX_LED;
                    IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
                    Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                    Radio.SetRx( ( TickTime_t ){ RX_TIMEOUT_TICK_SIZE, \
                                 ywd.DemoSettings.InterPacketDelay } );
                }
                break;

            case APP_RX:
                DemoInternalState = APP_IDLE;
                RX_LED = !RX_LED;
                Radio.GetPayload( Buffer, &BufferSize, BUFFER_SIZE );
                Radio.GetPacketStatus( &PacketStatus );
                if( ywd.ModulationParams.PacketType == PACKET_TYPE_LORA )
                {
                    ywd.DemoSettings.RssiValue = PacketStatus.LoRa.RssiPkt;
                    ywd.DemoSettings.SnrValue = PacketStatus.LoRa.SnrPkt;
                }
                else if( ywd.ModulationParams.PacketType == PACKET_TYPE_FLRC )
                {
                    ywd.DemoSettings.RssiValue = PacketStatus.Flrc.RssiSync;
                    ywd.DemoSettings.SnrValue = 0;
                }
                else if( ywd.ModulationParams.PacketType == PACKET_TYPE_GFSK  )
                {
                    ywd.DemoSettings.RssiValue = PacketStatus.Gfsk.RssiSync;
                    ywd.DemoSettings.SnrValue = 0;
                }
                if( ( BufferSize >= PINGPONG_SIZE ) && ( strncmp( ( const char* )( Buffer + 4 ), ( const char* )PingMsg, PINGPONG_SIZE ) == 0 ) )
                {
                    ComputePingPongPayload( Buffer, BufferSize );
                    DemoInternalState = SEND_PONG_MSG;
                }
                else
                {
                    ywd.DemoSettings.CntPacketRxKO++;
                    RX_LED = !RX_LED;
                    IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
                    Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                    Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, \
                        ywd.DemoSettings.InterPacketDelay } );
                }
                break;

            case APP_RX_TIMEOUT:
            case APP_RX_ERROR:
                DemoInternalState = APP_IDLE;
                ywd.DemoSettings.RxTimeOutCount++;
                IrqMask = IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT;
                Radio.SetDioIrqParams( IrqMask, IrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE );
                Radio.SetRx( ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, \
                        ywd.DemoSettings.InterPacketDelay } );
                break;

            case APP_TX_TIMEOUT:
                s.printf( "Failure: timeout in Tx is shorter than the packet time on air\n\r" );
                DemoInternalState = APP_IDLE;
                break;

            case APP_IDLE: // do nothing
                break;

            default:
                break;
        }
    }
}

void ComputePingPongPayload( uint8_t *buffer, uint8_t bufferSize )
{
    uint32_t i = 0;

    PacketRxSequence = ( ( uint32_t )buffer[0] << 24 ) | \
                       ( ( uint32_t )buffer[1] << 16 ) | \
                       ( ( uint32_t )buffer[2] << 8 )  | \
                                     buffer[3];

    if( ywd.DemoSettings.Entity == MASTER )
    {
        ywd.DemoSettings.CntPacketRxKOSlave = 
                       ( ( uint32_t )buffer[4] << 24 ) | \
                       ( ( uint32_t )buffer[5] << 16 ) | \
                       ( ( uint32_t )buffer[6] << 8 )  | \
                                     buffer[7];
        if( PacketRxSequence > ywd.DemoSettings.CntPacketRxKOSlave )
        {
            ywd.DemoSettings.CntPacketRxOKSlave = PacketRxSequence - \
                ywd.DemoSettings.CntPacketRxKOSlave;
        }
        else
        {
            ywd.DemoSettings.CntPacketRxOKSlave = 0;
        }
        
        if( PacketRxSequence == ywd.DemoSettings.CntPacketTx )
        {
            ywd.DemoSettings.CntPacketRxOK += 1;
        }
        else
        {
            ywd.DemoSettings.CntPacketRxKO += 1;
        }
    }
    else
    {
        ywd.DemoSettings.CntPacketRxOK += 1;
        if( ( PacketRxSequence <= PacketRxSequencePrev ) || \
            ( PacketRxSequencePrev == 0 ) )
        {
            // Sequence went back => resynchronization
            // Don't count missed packets this time
            i = 0;
        }
        else
        {
            // Determine number of missed packets
            i = PacketRxSequence - PacketRxSequencePrev - 1;
        }
        // Be ready for the next
        PacketRxSequencePrev = PacketRxSequence;
        ywd.DemoSettings.CntPacketTx = PacketRxSequence;
        // increment 'missed' counter for the RX session
        ywd.DemoSettings.CntPacketRxKO += i;
        ywd.DemoSettings.RxTimeOutCount = 0;
    }
}



// ************************        Utils            ****************************
// *                                                                           *
// *                                                                           *
// *                                                                           *
// *****************************************************************************

void InitDemoApplication( void )
{
    RX_LED = 1;
    TX_LED = 1;


    wait_ms( 500 ); // wait for on board DC/DC start-up time

    Radio.Init( );

    // Can also be set in LDO mode but consume more power
    Radio.SetRegulatorMode( ( RadioRegulatorModes_t )ywd.DemoSettings.RadioPowerMode );
    Radio.SetStandby( STDBY_RC );

    memset( &Buffer, 0x00, BufferSize );

    RX_LED = 0;
    TX_LED = 0;

    PacketRxSequence = 0;
    PacketRxSequencePrev = 0;
    ywd.DemoSettings.CntPacketTx    = 0;
    ywd.DemoSettings.CntPacketRxOK  = 0;
    ywd.DemoSettings.CntPacketRxKO  = 0;
    ywd.DemoSettings.RxTimeOutCount = 0;
}


/*
 * Function still being implemented >>> To be completed 
 * WARNING: Computation is in float and his really slow
 * LongInterLeaving vs LegacyInterLeaving has no influence on TimeOnAir.
 */
uint16_t GetTimeOnAir( uint8_t modulation )
{
    uint16_t result = 2000;

    if( modulation == PACKET_TYPE_LORA )
    {
        double bw = 0.0;

        switch( ywd.ModulationParams.Params.LoRa.Bandwidth )
        {
            case LORA_BW_0200:
                bw = 203e3;
                break;

            case LORA_BW_0400:
                bw = 406e3;
                break;

            case LORA_BW_0800:
                bw = 812e3;
                break;

            case LORA_BW_1600:
                bw = 1625e3;
                break;

            default:
                bw = 100e3;
                break;
        }
        double rs = bw / ( 1 << ( ywd.ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) );        // Symbol rate : time for one symbol (secs)
        double ts = 1 / rs;
        double tPreamble = ( ywd.PacketParams.Params.LoRa.PreambleLength + 4.25 ) * ts; // time of preamble
        uint8_t de = 1;    // always 1 on SX1280. "low data rate optimization" condition.
        double tmp = ceil( ( 8 * ywd.PacketParams.Params.LoRa.PayloadLength - 4 * ( ywd.ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) +
                             28 + 16 * ( ( ywd.PacketParams.Params.LoRa.Crc == 0x00 ) ? 0 : 1 ) - \
                             ( ( ywd.PacketParams.Params.LoRa.HeaderType >> 7 ) ? 20 : 0 ) ) / \
                           ( double )( 4 * ( ( ywd.ModulationParams.Params.LoRa.SpreadingFactor >> 4 ) - ( de * 2 ) ) ) * \
                           ( ( ywd.ModulationParams.Params.LoRa.CodingRate % 4 ) + 4 ) );                     // Symbol length of payload and time
        double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
        double tPayload = nPayload * ts;

        // Time on air [ms]
        result = floor( ( ( tPreamble + tPayload ) * 1000 * 1.2 ) + 0.999 ); // Set some margin
        result *= 1.8;   // Set some margin
    }
    else if( modulation == PACKET_TYPE_FLRC )
    {
        uint16_t packetBitCount;

        switch( ywd.PacketParams.Params.Flrc.PreambleLength )
        {
            case PREAMBLE_LENGTH_04_BITS:     // Preamble length: 04 bits
            case PREAMBLE_LENGTH_08_BITS:     // Preamble length: 08 bits
                packetBitCount = 1;
                break;

            case PREAMBLE_LENGTH_12_BITS:     // Preamble length: 12 bits
            case PREAMBLE_LENGTH_16_BITS:     // Preamble length: 16 bits
                packetBitCount = 2;
                break;

            case PREAMBLE_LENGTH_20_BITS:     // Preamble length: 20 bits
            case PREAMBLE_LENGTH_24_BITS:     // Preamble length: 24 bits
                packetBitCount = 3;
                break;

            case PREAMBLE_LENGTH_28_BITS:     // Preamble length: 28 bits
            case PREAMBLE_LENGTH_32_BITS:     // Preamble length: 32 bits
                packetBitCount = 4;
                break;

            default:
                packetBitCount = 4;
                break;
        }
        packetBitCount += 3;                  // Preamble 21 bits
        switch( ywd.PacketParams.Params.Flrc.SyncWordLength )
        {
            case FLRC_SYNCWORD_LENGTH_4_BYTE:
                packetBitCount += 4;
                break;

            case FLRC_NO_SYNCWORD:
            default:
                break;
        }
        switch( ywd.ModulationParams.Params.Flrc.CodingRate )
        {
            // += 7;  // CRC length is maximum 4 bytes (short-cut) + 2 Header + 6 bits Tail
            case FLRC_CR_3_4:
                packetBitCount += ( uint16_t )( ceil( ( ( float )( ywd.PacketParams.Params.Flrc.PayloadLength + 7 ) * 4 ) / 3 ) );
                break;

            case FLRC_CR_1_0:
                packetBitCount += ywd.PacketParams.Params.Flrc.PayloadLength + 7;
                break;

            default:
            case FLRC_CR_1_2:
                packetBitCount += ( ywd.PacketParams.Params.Flrc.PayloadLength + 7 ) * 2;
                break;
        }
        packetBitCount *= 8;
        switch( ywd.ModulationParams.Params.Flrc.BitrateBandwidth )
        {
            case FLRC_BR_1_300_BW_1_2:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 1300 ) );
                break;

            case FLRC_BR_1_040_BW_1_2:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 1040 ) );
                break;

            case FLRC_BR_0_650_BW_0_6:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 650 ) );
                break;

            case FLRC_BR_0_520_BW_0_6:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 520 ) );
                break;

            case FLRC_BR_0_325_BW_0_3:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 325 ) );
                break;

            default:
            case FLRC_BR_0_260_BW_0_3:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 260 ) );
                break;
        }
        result *= 2; // Set some margin
    }
    else if( modulation == PACKET_TYPE_GFSK )
    {
        uint16_t packetBitCount;

        switch( ywd.PacketParams.Params.Gfsk.PreambleLength )
        {
            case PREAMBLE_LENGTH_04_BITS:     // Preamble length: 04 bits
            case PREAMBLE_LENGTH_08_BITS:     // Preamble length: 08 bits
                packetBitCount = 1;
                break;

            case PREAMBLE_LENGTH_12_BITS:     // Preamble length: 12 bits
            case PREAMBLE_LENGTH_16_BITS:     // Preamble length: 16 bits
                packetBitCount = 2;
                break;

            case PREAMBLE_LENGTH_20_BITS:     // Preamble length: 20 bits
            case PREAMBLE_LENGTH_24_BITS:     // Preamble length: 24 bits
                packetBitCount = 3;
                break;

            case PREAMBLE_LENGTH_28_BITS:     // Preamble length: 28 bits
            case PREAMBLE_LENGTH_32_BITS:     // Preamble length: 32 bits
                packetBitCount = 4;
                break;

            default:
                packetBitCount = 4;
                break;
        }
        switch( ywd.PacketParams.Params.Gfsk.SyncWordLength )
        {
            case GFSK_SYNCWORD_LENGTH_1_BYTE:      // Sync word length: 1 byte
                packetBitCount += 1;
                break;

            case GFSK_SYNCWORD_LENGTH_2_BYTE:      // Sync word length: 2 bytes
                packetBitCount += 2;
                break;

            case GFSK_SYNCWORD_LENGTH_3_BYTE:      // Sync word length: 3 bytes
                packetBitCount += 3;
                break;

            case GFSK_SYNCWORD_LENGTH_4_BYTE:      // Sync word length: 4 bytes
                packetBitCount += 4;
                break;

            case GFSK_SYNCWORD_LENGTH_5_BYTE:      // Sync word length: 5 bytes
                packetBitCount += 5;
                break;

            default:
                packetBitCount += 5;
                break;
        }
        packetBitCount += ywd.PacketParams.Params.Gfsk.PayloadLength + 3;
        packetBitCount *= 8;
        switch( ywd.ModulationParams.Params.Gfsk.BitrateBandwidth )
        {
            case GFSK_BLE_BR_2_000_BW_2_4:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 2000 ) );
                break;

            case GFSK_BLE_BR_1_600_BW_2_4:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 1600 ) );
                break;

            case GFSK_BLE_BR_1_000_BW_2_4:
            case GFSK_BLE_BR_1_000_BW_1_2:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 1000 ) );
                break;

            case GFSK_BLE_BR_0_800_BW_2_4:
            case GFSK_BLE_BR_0_800_BW_1_2:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 800 ) );
                break;

            case GFSK_BLE_BR_0_500_BW_1_2:
            case GFSK_BLE_BR_0_500_BW_0_6:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 500 ) );
                break;

            case GFSK_BLE_BR_0_400_BW_1_2:
            case GFSK_BLE_BR_0_400_BW_0_6:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 400 ) );
                break;

            case GFSK_BLE_BR_0_250_BW_0_6:
            case GFSK_BLE_BR_0_250_BW_0_3:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 250 ) );
                break;

            case GFSK_BLE_BR_0_125_BW_0_3:
                result = ( uint16_t )( ceil( ( float )packetBitCount / 125 ) );
                break;

            default:
                result = 100;
                break;
        }
        result *= 1.5; // Set 50% margin
    }
    return result;
}

void InitializeDemoParameters( uint8_t modulation )
{
    Radio.SetStandby( STDBY_RC );

    Radio.SetRegulatorMode( ( RadioRegulatorModes_t )ywd.DemoSettings.RadioPowerMode );

    s.printf("> InitializeDemoParameters\n\r");
    if( modulation == PACKET_TYPE_LORA )
    {
        s.printf("set param LORA for demo\n\r");
        ModulationParams.PacketType = PACKET_TYPE_LORA;
        PacketParams.PacketType     = PACKET_TYPE_LORA;

        ModulationParams.Params.LoRa.SpreadingFactor = ( RadioLoRaSpreadingFactors_t )  ywd.DemoSettings.ModulationParam1;
        ModulationParams.Params.LoRa.Bandwidth       = ( RadioLoRaBandwidths_t )        ywd.DemoSettings.ModulationParam2;
        ModulationParams.Params.LoRa.CodingRate      = ( RadioLoRaCodingRates_t )       ywd.DemoSettings.ModulationParam3;
        PacketParams.Params.LoRa.PreambleLength      =                                  ywd.DemoSettings.PacketParam1;
        PacketParams.Params.LoRa.HeaderType          = ( RadioLoRaPacketLengthsModes_t )ywd.DemoSettings.PacketParam2;
        PacketParams.Params.LoRa.PayloadLength       =                                  ywd.DemoSettings.PacketParam3;
        PacketParams.Params.LoRa.Crc                 = ( RadioLoRaCrcModes_t )          ywd.DemoSettings.PacketParam4;
        PacketParams.Params.LoRa.InvertIQ            = ( RadioLoRaIQModes_t )           ywd.DemoSettings.PacketParam5;

        ywd.DemoSettings.PayloadLength = PacketParams.Params.LoRa.PayloadLength;
    }
  
    
	Radio.SetStandby( STDBY_RC );
	Radio.SetPacketType( ModulationParams.PacketType );
	Radio.SetRfFrequency( ywd.DemoSettings.Frequency );
	Radio.SetBufferBaseAddresses( 0x00, 0x00 );
	Radio.SetModulationParams( &ModulationParams );
	Radio.SetPacketParams( &PacketParams );
	// only used in GFSK, FLRC (4 bytes max) and BLE mode
	Radio.SetSyncWord( 1, ( uint8_t[] ){ 0xDD, 0xA0, 0x96, 0x69, 0xDD } );
	// only used in GFSK, FLRC
	uint8_t crcSeedLocal[3] = { 0x00, 0x45, 0x67 };
	Radio.SetCrcSeed( crcSeedLocal );
	Radio.SetCrcPolynomial( 0x0123 );
	Radio.SetTxParams( ywd.DemoSettings.TxPower, RADIO_RAMP_20_US );

}

/*!
 * \brief Callback of ticker PerSendNextPacket
 */
void SendNextPacketEvent( void )
{
    SendNext = true;
   
}

uint8_t CheckDistance( void )
{
    double displayRange = 0.0;
    double rssi = ywd.DemoSettings.RssiValue;

    uint16_t j = 0;
    uint16_t i;

    s.printf( "#id: %d", ywd.DemoSettings.CntPacketTx );
    if( RngResultIndex > 0 )
    {
        for( i = 0; i < RngResultIndex; ++i )
        {
            RawRngResults[i] = RawRngResults[i] - ( ywd.DemoSettings.RngFeiFactor * ywd.DemoSettings.RngFei / 1000 );
        }

        for (int i = RngResultIndex - 1; i > 0; --i) 
        {
            for (int j = 0; j < i; ++j) 
            {
                if (RawRngResults[j] > RawRngResults[j+1]) 
                {
                    int temp = RawRngResults[j];
                    RawRngResults[j] = RawRngResults[j+1];
                    RawRngResults[j+1] = temp;
                }
            }
        }
        double median;
        if ((RngResultIndex % 2) == 0) 
        {
            median = (RawRngResults[RngResultIndex/2] + RawRngResults[(RngResultIndex/2) - 1])/2.0;
        } 
        else 
        {
            median = RawRngResults[RngResultIndex/2];
        }

        if( median < 50 )
        {
            // Apply the short range correction and RSSI short range improvement below 50 m
            displayRange = t0 + t1 * rssi + t2 * pow(rssi,2) + t3 * pow(rssi, 3) +t4 * median + t5 * pow(median,2) + t6 * pow(median, 3) + t7 * pow(median, 4) ;       
        }
        else
        {
            displayRange = median;
        }

        if( j < DEMO_RNG_CHANNELS_COUNT_MIN )
        {
            ywd.DemoSettings.RngStatus = RNG_PER_ERROR;
        }
        else
        {
            ywd.DemoSettings.RngStatus = RNG_VALID;
        }

        if( displayRange < 0 )
        {
            ywd.DemoSettings.RngDistance = 0.0;
        }
        else
        {
            switch( ywd.DemoSettings.RngUnit )
            {
                case DEMO_RNG_UNIT_SEL_M:
                    ywd.DemoSettings.RngDistance = displayRange;
                    break;

                case DEMO_RNG_UNIT_SEL_YD:
                    ywd.DemoSettings.RngDistance = displayRange * DEMO_RNG_UNIT_CONV_YD;
                    break;

                case DEMO_RNG_UNIT_SEL_MI:
                    ywd.DemoSettings.RngDistance = displayRange * DEMO_RNG_UNIT_CONV_MI;
                    break;
            }
        }
    }
    s.printf( ", Rssi: %d, Zn: %3d, Zmoy: %5.1f, FEI: %d\r\n", ywd.DemoSettings.RssiValue, j, displayRange, ( int32_t )ywd.DemoSettings.RngFei );

    return j;
}

void LedBlink( void )
{
    if( ( TX_LED == 0 ) && ( RX_LED == 0 ) )
    {
        TX_LED = 1;
    }
    else if( ( TX_LED == 1 ) && ( RX_LED == 0 ) )
    {
        RX_LED = 1;
    }
    else if( ( TX_LED == 1 ) && ( RX_LED == 1 ) )
    {
        TX_LED = 0;
    }
    else
    {
        RX_LED = 0;
    }
}


// ************************     Radio Callbacks     ****************************
// *                                                                           *
// * These functions are called through function pointer by the Radio low      *
// * level drivers                                                             *
// *                                                                           *
// *****************************************************************************
void OnTxDone( void )
{
    DemoInternalState = APP_TX;
}

void OnRxDone( void )
{
    DemoInternalState = APP_RX;
}

void OnTxTimeout( void )
{
    DemoInternalState = APP_TX_TIMEOUT;
}

void OnRxTimeout( void )
{
    DemoInternalState = APP_RX_TIMEOUT;
}

void OnRxError( IrqErrorCode_t errorCode )
{
    DemoInternalState = APP_RX_ERROR;
}

void OnRangingDone( IrqRangingCode_t val )
{
    if( val == IRQ_RANGING_MASTER_VALID_CODE || val == IRQ_RANGING_SLAVE_VALID_CODE )
    {
        DemoInternalState = APP_RANGING_DONE;
    }
    else if( val == IRQ_RANGING_MASTER_ERROR_CODE || val == IRQ_RANGING_SLAVE_ERROR_CODE )
    {
        DemoInternalState = APP_RANGING_TIMEOUT;
    }
    else
    {
        DemoInternalState = APP_RANGING_TIMEOUT;
    }
}

void OnCadDone( bool channelActivityDetected )
{
}

/*!
 * \brief Specify serial datarate for UART debug output
 */
void baud( int baudrate )
{
    s.baud( baudrate );
}
